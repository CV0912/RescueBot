#!/usr/bin/env python3
"""
tag_marker_node
----------------
Watches TF for AprilTag frames published by apriltag_ros (e.g. "tag_0",
"tag_1", ... one distinct frame per physical tag id), transforms each into
the map frame, and:
  1. Publishes a MarkerArray on /detected_tags_markers for RViz visualization.
  2. Logs the map-frame (x, y, z) of each tag id to a YAML file, once per
     run.

Dedup is strictly by tag id (parsed from the TF frame name, e.g. "tag_3" ->
id 3) -- NOT by proximity. Each id is only ever placed once: the first
confirm_detections_required consistent readings for that id are averaged
together, the id is then marked 'confirmed', and its position is locked for
the rest of the run. Any further detections of that same id are ignored
positionally (only last_seen/detections counters are refreshed).

Assumes a TF tree of: map -> odom -> base_link -> ... -> camera_link_optical -> tag_<id>
i.e. slam_toolbox (or equivalent) publishing map->odom, and robot_state_publisher
publishing the rest from the URDF.
"""
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time as RclpyTime

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from visualization_msgs.msg import Marker, MarkerArray

import yaml


class TagMarkerNode(Node):
    def __init__(self):
        super().__init__('tag_marker_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_frame_prefix', 'tag_')
        self.declare_parameter('log_dir', os.path.expanduser('~/rescuebot_runs'))
        self.declare_parameter('poll_rate_hz', 2.0)
        # New-id confirmation: an id isn't finalized/locked until this many
        # detections in a row land within confirm_tolerance_m of each other.
        # This protects against a single noisy TF lookup permanently pinning
        # a marker in the wrong spot.
        self.declare_parameter('confirm_detections_required', 3)
        self.declare_parameter('confirm_tolerance_m', 0.15)

        self.map_frame = self.get_parameter('map_frame').value
        self.tag_prefix = self.get_parameter('tag_frame_prefix').value
        self.log_dir = self.get_parameter('log_dir').value
        self.confirm_detections_required = int(self.get_parameter('confirm_detections_required').value)
        self.confirm_tolerance = float(self.get_parameter('confirm_tolerance_m').value)

        # Fresh log file per run (per launch), so past hackathon runs never block new detections.
        os.makedirs(self.log_dir, exist_ok=True)
        run_id = self.get_clock().now().nanoseconds
        self.log_path = os.path.join(self.log_dir, f'run_{run_id}.yaml')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/detected_tags_markers', 10)

        # tag_id (int, from frame name) -> dict. While a tag id is being
        # confirmed it carries status='pending' and a running list of raw
        # samples; once enough consistent samples arrive it's finalized to
        # status='confirmed' with an averaged (x, y, z) that is then locked
        # and never moved again, no matter how many more times that id is
        # re-detected.
        self.known_tags = {}

        rate = float(self.get_parameter('poll_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.check_tags)

        self.get_logger().info(
            f'New run started. Watching TF for frames starting with "{self.tag_prefix}", '
            f'marking positions in "{self.map_frame}". This run\'s log: {self.log_path}'
        )

    # ------------------------------------------------------------------ #
    # Persistence -- always starts empty; writes to a new file each run
    # ------------------------------------------------------------------ #
    def _save_log(self):
        # Only confirmed ids are worth persisting; pending ones are still
        # unverified and may move before they're confirmed.
        data = {
            tid: {
                'x': info['x'], 'y': info['y'], 'z': info['z'],
                'first_seen_sec': info['first_seen'],
                'last_seen_sec': info['last_seen'],
                'detections': info['detections'],
            }
            for tid, info in self.known_tags.items()
            if info['status'] == 'confirmed'
        }
        try:
            with open(self.log_path, 'w') as f:
                yaml.safe_dump(data, f)
        except Exception as e:
            self.get_logger().warn(f'Could not write log file: {e}')

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def check_tags(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        changed = False

        for frame_id, tag_id in self._get_tag_frames():
            info = self.known_tags.get(tag_id)

            # Already confirmed: position is locked. Skip the TF lookup
            # entirely for this id -- we don't want it, and we especially
            # don't want it factoring into anything.
            if info is not None and info['status'] == 'confirmed':
                info['last_seen'] = now_sec
                info['detections'] += 1
                changed = True
                continue

            try:
                t = self.tf_buffer.lookup_transform(
                    self.map_frame, frame_id, RclpyTime(),
                    timeout=Duration(seconds=0.2)
                )
            except (LookupException, ConnectivityException, ExtrapolationException):
                continue

            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z

            if info is None:
                self.get_logger().info(
                    f'Candidate tag {tag_id} detected this run (pending confirmation) '
                    f'at map ({x:.2f}, {y:.2f}, {z:.2f})'
                )
                self.known_tags[tag_id] = {
                    'status': 'pending',
                    'samples': [(x, y, z)],
                    'x': x, 'y': y, 'z': z,  # provisional, until confirmed
                    'first_seen': now_sec, 'last_seen': now_sec,
                    'detections': 1,
                }
            else:
                info['last_seen'] = now_sec
                info['detections'] += 1
                self._handle_pending_sample(tag_id, info, x, y, z)

            changed = True

        if changed:
            self._save_log()

        self._publish_markers()

    def _handle_pending_sample(self, tag_id, info, x, y, z):
        """Add a new sample to a pending tag id and confirm+lock it once
        enough consecutive samples agree with each other."""
        cx, cy, cz = self._average(info['samples'])
        dist = ((cx - x) ** 2 + (cy - y) ** 2 + (cz - z) ** 2) ** 0.5

        if dist <= self.confirm_tolerance:
            info['samples'].append((x, y, z))
        else:
            # Outlier relative to the samples gathered so far -- restart the
            # window from this new reading rather than letting one bad
            # sample drag the running average around.
            self.get_logger().debug(
                f'Tag {tag_id}: sample {dist:.2f} m from running average, '
                f'restarting confirmation window'
            )
            info['samples'] = [(x, y, z)]

        if len(info['samples']) >= self.confirm_detections_required:
            fx, fy, fz = self._average(info['samples'])
            info['x'], info['y'], info['z'] = fx, fy, fz
            info['status'] = 'confirmed'
            info['samples'] = None  # no longer needed
            self.get_logger().info(
                f'Tag {tag_id} confirmed and LOCKED at map ({fx:.2f}, {fy:.2f}, {fz:.2f}) '
                f'after {self.confirm_detections_required} consistent detections'
            )
        else:
            # keep provisional position updated to the running average so
            # it visualizes sensibly even before confirmation
            info['x'], info['y'], info['z'] = self._average(info['samples'])

    @staticmethod
    def _average(samples):
        n = len(samples)
        sx = sum(s[0] for s in samples) / n
        sy = sum(s[1] for s in samples) / n
        sz = sum(s[2] for s in samples) / n
        return sx, sy, sz

    def _get_tag_frames(self):
        """Discover currently-broadcast TF frames matching our tag prefix,
        returning (frame_id, tag_id) pairs, e.g. ('tag_3', 3)."""
        results = []
        try:
            frames = yaml.safe_load(self.tf_buffer.all_frames_as_yaml()) or {}
        except Exception:
            return results

        for frame_id in frames.keys():
            if frame_id.startswith(self.tag_prefix):
                suffix = frame_id[len(self.tag_prefix):]
                match = re.match(r'(\d+)', suffix)
                if match:
                    results.append((frame_id, int(match.group(1))))
        return results

    # ------------------------------------------------------------------ #
    # Visualization
    # ------------------------------------------------------------------ #
    def _publish_markers(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for tag_id, info in self.known_tags.items():
            x, y, z = info['x'], info['y'], info['z']
            pending = info['status'] == 'pending'

            cube = Marker()
            cube.header.frame_id = self.map_frame
            cube.header.stamp = stamp
            cube.ns = 'detected_tags'
            cube.id = tag_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = x
            cube.pose.position.y = y
            cube.pose.position.z = z
            cube.pose.orientation.w = 1.0
            cube.scale.x = 0.2
            cube.scale.y = 0.2
            cube.scale.z = 0.05
            # Pending (unconfirmed) tags are shown dimmer/yellow-ish so it's
            # visually obvious in RViz which are still settling.
            if pending:
                cube.color.r = 1.0
                cube.color.g = 0.85
                cube.color.b = 0.1
                cube.color.a = 0.5
            else:
                cube.color.r = 1.0
                cube.color.g = 0.1
                cube.color.b = 0.1
                cube.color.a = 0.9
            marker_array.markers.append(cube)

            label = Marker()
            label.header.frame_id = self.map_frame
            label.header.stamp = stamp
            label.ns = 'detected_tags_labels'
            label.id = tag_id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + 0.3
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'Tag {tag_id}' + (' (pending)' if pending else '')
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TagMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()