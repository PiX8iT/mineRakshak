#!/usr/bin/env python3
"""
truck_detector.py - SIH 2026 PS 26007 demo node

Subscribes to a 3D lidar PointCloud2, finds truck-sized obstacles
(ground removal -> voxel clustering -> size filter), and publishes:
  /truck_markers            visualization_msgs/MarkerArray  (RViz: box + distance, coloured by zone)
  /nearest_truck_distance   std_msgs/Float32                (metres, -1.0 if none)
  /truck_alert              std_msgs/String                 (CLEAR / CAUTION / WARNING / STOP)
  /cmd_vel                  geometry_msgs/Twist             (zero velocity while in STOP zone)

Run:
  source /opt/ros/<distro>/setup.bash
  python3 truck_detector.py --ros-args -p lidar_height:=1.5

Only depends on rclpy, numpy and standard ROS message packages.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

ZONE_COLORS = {
    "CAUTION": (0.1, 0.9, 0.2),   # green  : truck detected, far
    "WARNING": (1.0, 0.85, 0.0),  # yellow : getting close
    "STOP": (1.0, 0.1, 0.1),      # red    : too close
}


class TruckDetector(Node):
    def __init__(self):
        super().__init__("truck_detector")

        # ---- topics ----
        self.declare_parameter("cloud_topic", "/scan/points")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_stop", True)

        # ---- ground removal ----
        # Sensor height above ground (m). Points below (-lidar_height + ground_margin)
        # in the sensor frame are treated as ground.
        self.declare_parameter("lidar_height", 1.5)
        self.declare_parameter("ground_margin", 0.7)
        self.declare_parameter("min_range", 2.0)      # ignore returns from own body
        self.declare_parameter("max_range", 50.0)

        # ---- clustering ----
        self.declare_parameter("voxel_size", 1.0)     # clusters merge if within ~1 voxel

        # ---- truck size filter (bounding box of cluster, metres) ----
        self.declare_parameter("min_extent_xy", 4.0)  # longest horizontal side >= this
        self.declare_parameter("max_extent_xy", 14.0) # ...and <= this (rejects walls)
        self.declare_parameter("min_height", 2.5)     # cluster height above ground
        self.declare_parameter("max_height", 7.0)     # truck ~6.2 m, bench walls ~8 m
        self.declare_parameter("min_points", 15)
        # Reject clusters that fit a single flat plane (walls, bench faces).
        # RMS distance (m) of points from their best-fit plane; trucks have wheels,
        # a recessed cab etc. so they score higher than a wall (~0.0-0.1).
        self.declare_parameter("min_plane_rms", 0.2)
        # Log every cluster with its stats and why it was accepted/rejected.
        self.declare_parameter("debug", False)

        # ---- warning zones (distance to nearest point of the truck, metres) ----
        self.declare_parameter("warning_dist", 30.0)
        self.declare_parameter("stop_dist", 15.0)

        p = self.get_parameter
        self.publish_stop = p("publish_stop").value

        self.sub = self.create_subscription(
            PointCloud2, p("cloud_topic").value, self.cloud_cb, qos_profile_sensor_data
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/truck_markers", 10)
        self.dist_pub = self.create_publisher(Float32, "/nearest_truck_distance", 10)
        self.alert_pub = self.create_publisher(String, "/truck_alert", 10)
        self.cmd_pub = self.create_publisher(Twist, p("cmd_vel_topic").value, 10)

        self.get_logger().info(f"Listening on {p('cloud_topic').value}")

    # ------------------------------------------------------------------
    def read_xyz(self, msg):
        arr = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if arr.dtype.names:  # structured array (newer ROS 2)
            pts = np.column_stack([arr["x"], arr["y"], arr["z"]])
        else:                # plain (N,3) array (older ROS 2)
            pts = np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        return pts.astype(np.float64)

    def voxel_cluster(self, pts, voxel):
        """Connected components over occupied voxels (26-neighbourhood)."""
        vox = np.floor(pts / voxel).astype(np.int64)
        uniq, inv = np.unique(vox, axis=0, return_inverse=True)
        inv = inv.ravel()
        index = {tuple(v): i for i, v in enumerate(uniq)}
        labels = np.full(len(uniq), -1, dtype=np.int64)
        offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                   for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]
        n_clusters = 0
        for start in range(len(uniq)):
            if labels[start] != -1:
                continue
            labels[start] = n_clusters
            stack = [start]
            while stack:
                cur = uniq[stack.pop()]
                for dx, dy, dz in offsets:
                    nb = index.get((cur[0] + dx, cur[1] + dy, cur[2] + dz))
                    if nb is not None and labels[nb] == -1:
                        labels[nb] = n_clusters
                        stack.append(nb)
            n_clusters += 1
        return labels[inv], n_clusters

    # ------------------------------------------------------------------
    def cloud_cb(self, msg):
        p = self.get_parameter
        pts = self.read_xyz(msg)
        if pts.shape[0] == 0:
            self.publish_result(msg, [], [])
            return

        # range gate
        rng = np.hypot(pts[:, 0], pts[:, 1])
        pts = pts[(rng > p("min_range").value) & (rng < p("max_range").value)]

        # ground removal (simple height threshold in sensor frame)
        ground_z = -p("lidar_height").value + p("ground_margin").value
        pts = pts[pts[:, 2] > ground_z]
        if pts.shape[0] < p("min_points").value:
            self.publish_result(msg, [], [])
            return

        labels, n = self.voxel_cluster(pts, p("voxel_size").value)

        trucks = []
        ground_level = -p("lidar_height").value
        for k in range(n):
            c = pts[labels == k]
            if c.shape[0] < p("min_points").value:
                continue
            mn, mx = c.min(axis=0), c.max(axis=0)
            ext = mx - mn
            longest_xy = max(ext[0], ext[1])
            height = mx[2] - ground_level  # top of cluster above ground
            # flatness: RMS distance from best-fit plane (smallest singular value)
            centred = c - c.mean(axis=0)
            plane_rms = float(np.linalg.svd(centred, compute_uv=False)[-1] / math.sqrt(c.shape[0]))
            nearest = float(np.min(np.hypot(c[:, 0], c[:, 1])))

            reason = None
            if not (p("min_extent_xy").value <= longest_xy <= p("max_extent_xy").value):
                reason = "extent"
            elif not (p("min_height").value <= height <= p("max_height").value):
                reason = "height"
            elif plane_rms < p("min_plane_rms").value:
                reason = "flat (wall)"

            if p("debug").value:
                self.get_logger().info(
                    f"cluster {k}: pts={c.shape[0]} extent_xy={longest_xy:.1f} "
                    f"height={height:.1f} plane_rms={plane_rms:.2f} dist={nearest:.1f} -> "
                    f"{'REJECT ' + reason if reason else 'TRUCK'}"
                )
            if reason:
                continue
            trucks.append({"min": mn, "max": mx, "nearest": nearest, "n": c.shape[0]})

        trucks.sort(key=lambda t: t["nearest"])
        self.publish_result(msg, trucks, [])

    # ------------------------------------------------------------------
    def zone_for(self, dist):
        p = self.get_parameter
        if dist < p("stop_dist").value:
            return "STOP"
        if dist < p("warning_dist").value:
            return "WARNING"
        return "CAUTION"

    def publish_result(self, msg, trucks, _unused):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        worst = "CLEAR"
        order = {"CLEAR": 0, "CAUTION": 1, "WARNING": 2, "STOP": 3}

        for i, t in enumerate(trucks):
            zone = self.zone_for(t["nearest"])
            if order[zone] > order[worst]:
                worst = zone
            r, g, b = ZONE_COLORS[zone]
            centre = (t["min"] + t["max"]) / 2.0
            size = np.maximum(t["max"] - t["min"], 0.2)

            box = Marker()
            box.header = msg.header
            box.ns, box.id = "truck_box", i
            box.type, box.action = Marker.CUBE, Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = map(float, centre)
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = map(float, size)
            box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.35
            markers.markers.append(box)

            label = Marker()
            label.header = msg.header
            label.ns, label.id = "truck_label", i
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x = float(centre[0])
            label.pose.position.y = float(centre[1])
            label.pose.position.z = float(t["max"][2]) + 1.5
            label.scale.z = 1.6
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = f"TRUCK {i + 1}: {t['nearest']:.1f} m [{zone}]"
            markers.markers.append(label)

        self.marker_pub.publish(markers)

        d = Float32()
        d.data = float(trucks[0]["nearest"]) if trucks else -1.0
        self.dist_pub.publish(d)

        a = String()
        a.data = worst
        self.alert_pub.publish(a)

        if trucks:
            self.get_logger().info(
                f"{len(trucks)} truck(s), nearest {trucks[0]['nearest']:.1f} m -> {worst}",
                throttle_duration_sec=1.0,
            )

        # auto-stop: zero velocity while a truck is inside the stop zone
        if self.publish_stop and worst == "STOP":
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = TruckDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()