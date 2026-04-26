#!/usr/bin/env python3
"""LiDAR-based opponent detector.

Pipeline:
  LaserScan -> ego-frame occupancy grid -> transform to map frame
  -> filter to points between inner_bound / outer_bound polygons (track filter)
  -> union-find clustering -> publish centroid of largest cluster as PoseStamped

Adapted from f1tenth-racing-stack-ICRA22/opponent_predictor/scripts/opponent_predictor_node.py.
Changes:
  - Hard-coded `src/opponent_predictor/...` paths replaced with parameterized
    `bound_dir` / `map_yaml_path` resolved via ament share dir.
  - Topic names parameterized.
  - Fixed bug: original wrote `opp_state.position.x` on PoseStamped (only
    `opp_state.pose.position.x` is valid).
  - Dropped unused track_file load.
  - Added `disable_track_filter` escape hatch for testing without bound files.
"""
import math
import os

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.spatial import distance

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Int16

from ament_index_python.packages import get_package_share_directory


class OpponentPredictor(Node):
    def __init__(self):
        super().__init__("opp_predictor_node")

        # Parameters (with defaults so a YAML file is optional)
        self.declare_parameter("real_test", True)
        self.declare_parameter("map_name", "racetrack_test")
        self.declare_parameter("map_img_ext", ".pgm")

        # Paths: when empty, resolved from share/opp_predictor/{maps,csv}/
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("map_img_path", "")
        self.declare_parameter("bound_dir", "")

        self.declare_parameter("inner_bound_file", "inner_bound")
        self.declare_parameter("outer_bound_file", "outer_bound")

        # Sensor / pose topics
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("ego_pose_topic_real", "/pf/viz/inferred_pose")
        self.declare_parameter("ego_pose_topic_sim", "/ego_racecar/odom")

        # Output topics
        self.declare_parameter("opp_state_topic", "/opp_predict/state")
        self.declare_parameter("opp_bbox_topic", "/opp_predict/bbox")
        self.declare_parameter("opp_viz_pose_topic", "/opp_predict/viz/pose")
        self.declare_parameter("opp_viz_bbox_topic", "/opp_predict/viz/bbox")
        self.declare_parameter("grid_topic", "/grid")
        self.declare_parameter("obstacle_topic", "/obstacle")
        self.declare_parameter("fps_topic", "/fps")

        # Detection grid (ego frame, meters)
        self.declare_parameter("grid_xmin", 0.0)
        self.declare_parameter("grid_xmax", 5.0)
        self.declare_parameter("grid_ymin", -2.5)
        self.declare_parameter("grid_ymax", 2.5)
        self.declare_parameter("grid_resolution", 0.04)
        self.declare_parameter("plot_resolution", 0.1)
        self.declare_parameter("grid_safe_dist", 0.1)

        # Clustering
        self.declare_parameter("cluster_dist_tol", 0.27)   # ~ car width
        self.declare_parameter("cluster_size_tol", 15)     # min points to be opp

        # Visualization toggles
        self.declare_parameter("visualize", True)
        self.declare_parameter("visualize_grid", False)
        self.declare_parameter("visualize_obstacle", False)
        self.declare_parameter("visualize_opp_pose", True)
        self.declare_parameter("visualize_opp_bbox", True)

        # Escape hatch: skip the track filter (works without bound files but
        # walls become false positives — use grid bounds to compensate)
        self.declare_parameter("disable_track_filter", False)

        # Resolve parameters
        self.real_test = self.get_parameter("real_test").get_parameter_value().bool_value
        map_name = self.get_parameter("map_name").get_parameter_value().string_value
        map_img_ext = self.get_parameter("map_img_ext").get_parameter_value().string_value
        self.disable_track_filter = (
            self.get_parameter("disable_track_filter").get_parameter_value().bool_value
        )

        # Map paths (defaults are share-dir relative)
        share = get_package_share_directory("opp_predictor")
        map_yaml = self.get_parameter("map_yaml_path").get_parameter_value().string_value \
            or os.path.join(share, "maps", map_name + ".yaml")
        map_img = self.get_parameter("map_img_path").get_parameter_value().string_value \
            or os.path.join(share, "maps", map_name + map_img_ext)
        bound_dir = self.get_parameter("bound_dir").get_parameter_value().string_value \
            or os.path.join(share, "csv", map_name)

        inner_name = self.get_parameter("inner_bound_file").get_parameter_value().string_value
        outer_name = self.get_parameter("outer_bound_file").get_parameter_value().string_value

        self.map, self.map_metadata = self._read_map(map_img, map_yaml)

        if not self.disable_track_filter:
            self.inner_bound = np.load(os.path.join(bound_dir, inner_name + ".npy"))
            self.outer_bound = np.load(os.path.join(bound_dir, outer_name + ".npy"))
            self.get_logger().info(
                f"Loaded bounds from {bound_dir}: inner={len(self.inner_bound)} pts, "
                f"outer={len(self.outer_bound)} pts")
        else:
            self.inner_bound = None
            self.outer_bound = None
            self.get_logger().warn("Track filter disabled — walls may register as opponents")

        # Runtime state
        self.grid = None
        self.obstacle = None
        self.opponent = None
        self.ego_global_pos = None
        self.ego_global_yaw = None
        self.opp_global_pos = None
        self.scan_timestamp = None
        self.frame_cnt = 0

        # Subscriptions / publications
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        ego_pose_topic = (
            self.get_parameter("ego_pose_topic_real").get_parameter_value().string_value
            if self.real_test
            else self.get_parameter("ego_pose_topic_sim").get_parameter_value().string_value
        )

        if self.real_test:
            self.pose_sub = self.create_subscription(
                PoseStamped, ego_pose_topic, self.pose_callback, 1)
        else:
            self.pose_sub = self.create_subscription(
                Odometry, ego_pose_topic, self.pose_callback, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 1)

        gt = lambda k: self.get_parameter(k).get_parameter_value().string_value
        self.opp_state_pub = self.create_publisher(PoseStamped, gt("opp_state_topic"), 10)
        self.opp_bbox_pub = self.create_publisher(PoseArray, gt("opp_bbox_topic"), 10)
        self.opp_viz_pose_pub = self.create_publisher(Marker, gt("opp_viz_pose_topic"), 10)
        self.opp_viz_bbox_pub = self.create_publisher(MarkerArray, gt("opp_viz_bbox_topic"), 10)
        self.grid_pub = self.create_publisher(MarkerArray, gt("grid_topic"), 10)
        self.obstacle_pub = self.create_publisher(MarkerArray, gt("obstacle_topic"), 10)
        self.fps_pub = self.create_publisher(Int16, gt("fps_topic"), 10)

        self.timer = self.create_timer(1.0, self._fps_tick)

        self.get_logger().info(
            f"opp_predictor_node up: scan='{scan_topic}', pose='{ego_pose_topic}', "
            f"real_test={self.real_test}, map='{map_name}'")

    # ---------- map loading ----------

    @staticmethod
    def _read_map(map_img_path, map_yaml_path):
        img = Image.open(map_img_path).transpose(Image.FLIP_TOP_BOTTOM)
        img = np.asarray(img).astype(np.float64)
        img[img <= 128.0] = 0.0
        img[img > 128.0] = 255.0
        h, w = img.shape[:2]

        with open(map_yaml_path, "r") as f:
            meta = yaml.safe_load(f)
        res = float(meta["resolution"])
        ox, oy = float(meta["origin"][0]), float(meta["origin"][1])

        ix, iy = np.meshgrid(np.arange(w), np.arange(h))
        mxs = ix * res + ox
        mys = iy * res + oy
        mvs = np.where(img > 0, 0.0, 1.0)  # 1 = wall, 0 = free
        return np.dstack((mvs, mxs, mys)), (h, w, res, ox, oy)

    @staticmethod
    def _to_img_coords(pts, height, scale, tx, ty):
        nx = (pts[:, 0] - tx) / scale
        ny = height - (pts[:, 1] - ty) / scale
        return np.vstack((nx, ny)).T.astype(np.int32)

    def _fps_tick(self):
        msg = Int16()
        msg.data = self.frame_cnt
        self.frame_cnt = 0
        self.fps_pub.publish(msg)

    # ---------- scan / pose ----------

    def scan_callback(self, msg: LaserScan):
        ranges = np.clip(np.array(msg.ranges), msg.range_min, msg.range_max)

        gp = self.get_parameter
        xmin = gp("grid_xmin").get_parameter_value().double_value
        xmax = gp("grid_xmax").get_parameter_value().double_value
        ymin = gp("grid_ymin").get_parameter_value().double_value
        ymax = gp("grid_ymax").get_parameter_value().double_value
        res = gp("grid_resolution").get_parameter_value().double_value
        safe = gp("grid_safe_dist").get_parameter_value().double_value

        nx = int((xmax - xmin) / res) + 1
        ny = int((ymax - ymin) / res) + 1
        x = np.linspace(xmin, xmax, nx)
        y = np.linspace(ymin, ymax, ny)
        Y, X = np.meshgrid(y, x)
        rho = np.sqrt(X ** 2 + Y ** 2)
        phi = np.arctan2(Y, X)

        ray_idx = ((phi - msg.angle_min) / msg.angle_increment).astype(int)
        ray_idx = np.clip(ray_idx, 0, len(ranges) - 1)
        obs_rho = ranges[ray_idx]

        occ = np.where(np.abs(rho - obs_rho) < safe, 1.0, 0.0)
        self.grid = np.dstack((occ, X, Y))
        self.scan_timestamp = msg.header.stamp

    def pose_callback(self, msg):
        if self.real_test:
            cx = msg.pose.position.x
            cy = msg.pose.position.y
            q = msg.pose.orientation
        else:
            cx = msg.pose.pose.position.x
            cy = msg.pose.pose.position.y
            q = msg.pose.pose.orientation

        self.ego_global_pos = np.array([cx, cy])
        self.ego_global_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z))

        self._detect_opponent()

        if self.get_parameter("visualize").get_parameter_value().bool_value:
            if self.get_parameter("visualize_grid").get_parameter_value().bool_value:
                self._viz_grid()
            if self.get_parameter("visualize_obstacle").get_parameter_value().bool_value:
                self._viz_obstacle()
            if self.get_parameter("visualize_opp_pose").get_parameter_value().bool_value:
                self._viz_opp_pose()
            if self.get_parameter("visualize_opp_bbox").get_parameter_value().bool_value:
                self._viz_opp_bbox()

        self.frame_cnt += 1

    # ---------- detection ----------

    def _publish_no_opp(self):
        msg = PoseStamped()
        msg.pose.position.x = float("inf")
        msg.pose.position.y = float("inf")
        if self.scan_timestamp is not None:
            msg.header.stamp = self.scan_timestamp
        self.opp_state_pub.publish(msg)
        self.opp_bbox_pub.publish(PoseArray())
        self.opp_global_pos = None
        self.opponent = None

    def _detect_opponent(self):
        if self.grid is None or self.ego_global_pos is None:
            return

        # Project occupied grid cells into the map frame
        gv = self.grid[:, :, 0].flatten()
        gx = self.grid[:, :, 1].flatten()
        gy = self.grid[:, :, 2].flatten()
        gx = gx[gv > 0]
        gy = gy[gv > 0]
        if len(gx) == 0:
            self._publish_no_opp()
            return

        cs, sn = np.cos(self.ego_global_yaw), np.sin(self.ego_global_yaw)
        R = np.array([[cs, -sn], [sn, cs]])
        gx, gy = R @ np.vstack((gx, gy)) + self.ego_global_pos.reshape(-1, 1)
        grid_pts = np.vstack((gx, gy)).T

        if self.disable_track_filter:
            obstacle_pts = grid_pts
        else:
            img_pts = self._to_img_coords(
                grid_pts, self.map_metadata[0], self.map_metadata[2],
                self.map_metadata[3], self.map_metadata[4])
            keep = []
            for i in range(len(img_pts)):
                pt = (int(img_pts[i, 0]), int(img_pts[i, 1]))
                if cv2.pointPolygonTest(self.outer_bound, pt, False) != 1:
                    continue
                if cv2.pointPolygonTest(self.inner_bound, pt, False) != -1:
                    continue
                keep.append(i)
            if not keep:
                self._publish_no_opp()
                return
            obstacle_pts = grid_pts[keep]

        self.obstacle = obstacle_pts

        # Cluster, keep largest, threshold by min size
        gp = self.get_parameter
        dist_tol = gp("cluster_dist_tol").get_parameter_value().double_value
        size_tol = gp("cluster_size_tol").get_parameter_value().integer_value

        clusters = self._cluster(obstacle_pts, dist_tol)
        if not clusters:
            self._publish_no_opp()
            return
        sizes = [len(c) for c in clusters]
        if max(sizes) < size_tol:
            self._publish_no_opp()
            return

        opp_idx = clusters[int(np.argmax(sizes))]
        self.opponent = obstacle_pts[opp_idx]
        self.opp_global_pos = np.mean(self.opponent, axis=0).flatten()

        out = PoseStamped()
        out.pose.position.x = float(self.opp_global_pos[0])
        out.pose.position.y = float(self.opp_global_pos[1])
        if self.scan_timestamp is not None:
            out.header.stamp = self.scan_timestamp
        out.header.frame_id = "map"
        self.opp_state_pub.publish(out)

        bbox = PoseArray()
        bbox.header.frame_id = "map"
        if self.scan_timestamp is not None:
            bbox.header.stamp = self.scan_timestamp
        for pt in self.opponent:
            p = Pose(); p.position.x = float(pt[0]); p.position.y = float(pt[1])
            bbox.poses.append(p)
        self.opp_bbox_pub.publish(bbox)

    @staticmethod
    def _cluster(points, tol):
        n = len(points)
        if n == 0:
            return []
        parents = list(range(n))

        def find(i):
            while parents[i] != i:
                parents[i] = parents[parents[i]]
                i = parents[i]
            return i

        def union(i, j):
            ri, rj = find(i), find(j)
            if ri != rj:
                parents[ri] = rj

        d = distance.cdist(points, points)
        for i in range(n):
            for j in range(i):
                if d[i, j] <= tol:
                    union(i, j)

        groups = {}
        for i in range(n):
            r = find(i)
            groups.setdefault(r, []).append(i)
        return list(groups.values())

    # ---------- visualization ----------

    def _viz_grid(self):
        if self.grid is None or self.ego_global_pos is None:
            return
        gp = self.get_parameter
        res = gp("grid_resolution").get_parameter_value().double_value
        plot_res = gp("plot_resolution").get_parameter_value().double_value
        ds = max(1, int(plot_res / res))
        g = self.grid[::ds, ::ds, :]
        gv = g[:, :, 0].flatten()
        gx = g[:, :, 1].flatten()
        gy = g[:, :, 2].flatten()
        cs, sn = np.cos(self.ego_global_yaw), np.sin(self.ego_global_yaw)
        R = np.array([[cs, -sn], [sn, cs]])
        gx, gy = R @ np.vstack((gx, gy)) + self.ego_global_pos.reshape(-1, 1)

        arr = MarkerArray()
        for i in range(len(gv)):
            if gv[i] == 0:
                continue
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.ns = "occ_grid"
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(gx[i])
            m.pose.position.y = float(gy[i])
            m.color.r = 1.0; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.grid_pub.publish(arr)

    def _viz_obstacle(self):
        if self.obstacle is None:
            return
        arr = MarkerArray()
        for i, pt in enumerate(self.obstacle):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.ns = "obstacle"
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(pt[0]); m.pose.position.y = float(pt[1])
            m.color.r = m.color.g = m.color.b = 0.5; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.obstacle_pub.publish(arr)

    def _viz_opp_pose(self):
        if self.opp_global_pos is None:
            return
        m = Marker()
        m.header.frame_id = "map"
        m.id = 0
        m.ns = "opp_pose"
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(self.opp_global_pos[0])
        m.pose.position.y = float(self.opp_global_pos[1])
        m.pose.orientation.w = 1.0
        m.color.r = 1.0; m.color.b = 1.0; m.color.a = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.25
        m.lifetime.nanosec = int(2e8)
        self.opp_viz_pose_pub.publish(m)

    def _viz_opp_bbox(self):
        if self.opponent is None:
            return
        arr = MarkerArray()
        for i, pt in enumerate(self.opponent):
            m = Marker()
            m.header.frame_id = "map"
            m.id = i
            m.ns = "opp_pts"
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(pt[0]); m.pose.position.y = float(pt[1])
            m.color.r = m.color.g = m.color.b = 0.5; m.color.a = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.1
            m.lifetime.nanosec = int(1e8)
            arr.markers.append(m)
        self.opp_viz_bbox_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = OpponentPredictor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
