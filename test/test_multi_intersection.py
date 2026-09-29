# Copyright 2026 Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

import os
from pathlib import Path
import signal
import statistics
import subprocess
import time
import uuid

from autoware_map_msgs.msg import LaneletMapBin
from autoware_perception_msgs.msg import TrackedObject, TrackedObjects
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile
from tier4_v2x_msgs.msg import VirtualTrafficLightStateArray
import yaml


class Harness:
    def __init__(self, tmp_path, parameters=None):
        self.tmp_path = tmp_path
        self.context = rclpy.context.Context()
        self.env = dict(os.environ, ROS_LOCALHOST_ONLY='1')
        # Topic isolation also permits parallel test runs on the same ROS domain.
        prefix = '/intersection_test_' + uuid.uuid4().hex
        self.node = rclpy.create_node('fixture', context=self._init_context())
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.params = yaml.safe_load(Path(os.environ['VTL_CONFIG']).read_text())
        params = self.params['/**']['ros__parameters']
        for key, suffix in (
            ('vector_map_topic', '/map'), ('tracked_objects_topic', '/objects'),
            ('odometry_topic', '/odom'), ('virtual_traffic_light_state_topic', '/vtl'),
        ):
            params[key] = prefix + suffix
        params.update(tracked_object_timeout_sec=0.4, tracked_objects_freshness_sec=0.25)
        if parameters:
            params.update(parameters)
        if not params.get('intersection_ids'):
            params.pop('intersection_ids', None)
        self.expected_ids = (
            {params['intersections'][i]['virtual_traffic_light_id']
             for i in params['intersection_ids']}
            if params.get('intersection_ids') else {params['virtual_traffic_light_id']}
        )
        param_file = tmp_path / 'params.yaml'
        param_file.write_text(yaml.safe_dump(self.params))
        self.samples = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(LaneletMapBin, prefix + '/map', qos)
        self.objects_pub = self.node.create_publisher(TrackedObjects, prefix + '/objects', 1)
        self.odom_pub = self.node.create_publisher(Odometry, prefix + '/odom', 1)
        self.sub = self.node.create_subscription(
            VirtualTrafficLightStateArray, prefix + '/vtl', self.capture, 10)
        self.log_path = tmp_path / 'node.log'
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen(
            [os.environ['VTL_NODE'], '--ros-args', '--params-file', str(param_file)],
            env=self.env, stdout=self.log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 8.0
            while (not self.samples or self.map_pub.get_subscription_count() == 0 or
                   self.objects_pub.get_subscription_count() == 0 or
                   self.odom_pub.get_subscription_count() == 0):
                assert time.monotonic() < deadline, self.log_path.read_text()
                assert self.process.poll() is None, self.log_path.read_text()
                self.pump(0.05)
        except BaseException:
            self.close()
            raise

    def _init_context(self):
        rclpy.init(context=self.context)
        return self.context

    def capture(self, message):
        assert len(message.states) == len(self.expected_ids)
        assert {s.id for s in message.states} == self.expected_ids
        assert all(s.type == 'virtual' and not s.is_finalized for s in message.states)
        assert all(s.stamp == message.stamp for s in message.states)
        self.samples.append((time.monotonic(), {s.id: s.approval for s in message.states}))

    def load_map(self, kind='separate'):
        path = self.tmp_path / (kind + '.bin')
        subprocess.run([os.environ['VTL_MAP_FIXTURE'], str(path), kind], check=True)
        message = LaneletMapBin()
        message.header.frame_id = 'map'
        message.data = path.read_bytes()
        self.map_pub.publish(message)
        self.pump(0.2)

    def pump(self, duration, objects=None, ego=None):
        deadline = time.monotonic() + duration
        due = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() >= due:
                stamp = self.node.get_clock().now().to_msg()
                if objects is not None:
                    message = TrackedObjects()
                    message.header.frame_id = 'map'
                    message.header.stamp = stamp
                    for ident, x in objects:
                        obj = TrackedObject()  # Missing classification participates as UNKNOWN.
                        obj.object_id.uuid = [0] * 15 + [ident]
                        obj.kinematics.pose_with_covariance.pose.position.x = float(x)
                        obj.kinematics.pose_with_covariance.pose.position.y = 2.0
                        message.objects.append(obj)
                    self.objects_pub.publish(message)
                if ego is not None:
                    message = Odometry()
                    message.header.frame_id = 'map'
                    message.header.stamp = stamp
                    message.pose.pose.position.x = float(ego)
                    message.pose.pose.position.y = 2.0
                    self.odom_pub.publish(message)
                due = time.monotonic() + 0.03
            self.executor.spin_once(timeout_sec=0.01)

    def check(self, first, second):
        expected = {'2012980': first, '2013058': second}
        assert len(self.samples) >= 2
        assert all(s == expected for _, s in self.samples[-2:]), self.log_path.read_text()

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()


@pytest.fixture
def harness(tmp_path):
    fixture = Harness(tmp_path)
    try:
        yield fixture
    finally:
        fixture.close()


def test_independent_intersections(harness):
    h = harness
    h.pump(0.25)
    h.check(False, False)
    h.load_map()
    h.pump(0.7, ego=2)  # No perception data: no grant.
    h.check(False, False)
    h.pump(0.8, objects=[(1, 39)], ego=2)
    h.check(True, False)  # Occupancy in intersection 2 cannot block intersection 1.
    h.pump(0.4, objects=[(1, 39), (2, 9)], ego=9)
    h.pump(0.6, ego=9)  # Even stale perception cannot revoke the crossing latch.
    h.check(True, False)
    h.pump(0.3, objects=[(1, 39)], ego=55)
    h.check(False, False)
    h.pump(0.8, objects=[(1, 39)], ego=32)
    h.check(False, False)  # Direct CONFLICT entry blocks even without a queue entry.
    h.pump(0.3, objects=[], ego=32)
    h.check(False, False)
    h.pump(1.0, objects=[], ego=32)  # Timeout plus a new continuous clear interval.
    h.check(False, True)
    h.load_map()
    h.pump(0.3, objects=[(3, 39)], ego=39)
    h.check(False, True)  # Grant survives map replacement and a new blocker.
    h.pump(0.3, objects=[], ego=55)
    h.check(False, False)

    # Queue order and PRIORITY -> OUTSIDE retention still apply independently.
    h.pump(0.3, objects=[(4, 2)], ego=55)
    h.pump(0.3, objects=[(4, 55)], ego=55)
    h.pump(0.8, objects=[(4, 55)], ego=2)
    h.check(False, False)
    h.pump(0.3, objects=[(4, 9)], ego=2)
    h.pump(0.8, objects=[(4, 55)], ego=2)
    h.check(True, False)
    h.pump(0.3, objects=[], ego=55)
    h.check(False, False)
    log = h.log_path.read_text()
    assert 'TRACKED_OBJECT_TIMEOUT:' in log
    assert 'WAITING: intersection_id=2, reason=CONFLICT_OCCUPIED' in log
    assert log.count('RIGHT_OF_WAY_GRANTED: intersection_id=1,') == 2
    assert log.count('RIGHT_OF_WAY_GRANTED: intersection_id=2,') == 1
    assert log.count('RIGHT_OF_WAY_REVOKED: intersection_id=1,') == 2
    assert log.count('RIGHT_OF_WAY_REVOKED: intersection_id=2,') == 1
    period = statistics.median(b[0] - a[0] for a, b in zip(h.samples, h.samples[1:]))
    assert 0.08 < period < 0.12


def test_independent_latches_and_timers(harness):
    h = harness
    h.load_map('overlap')
    h.pump(0.2, objects=[], ego=2)  # Ego first in both queues, dwell not complete yet.
    h.pump(0.8, objects=[(1, 12)], ego=2)
    h.check(False, True)  # Blocker resets only intersection 1's timer.
    h.pump(0.25, objects=[(1, 55)], ego=2)
    h.check(False, True)
    h.pump(0.6, objects=[(1, 55)], ego=2)
    h.check(True, True)
    h.load_map('overlap')
    h.pump(0.3, objects=[], ego=12)
    h.check(True, True)
    h.pump(0.3, objects=[], ego=25)  # Exit 1 while crossing 2.
    h.check(False, True)
    h.pump(0.3, objects=[], ego=55)
    h.check(False, False)


def test_third_intersection_missing_geometry(tmp_path):
    h = Harness(tmp_path, {
        'intersection_ids': ['1', '2', '3'],
        'intersections': {
            '1': {'virtual_traffic_light_id': '2012980'},
            '2': {'virtual_traffic_light_id': '2013058'},
            '3': {'virtual_traffic_light_id': 'test-third'},
        },
    })
    try:
        h.load_map()
        h.pump(0.8, objects=[], ego=62)
        assert h.samples[-1][1] == {'2012980': False, '2013058': False, 'test-third': False}
        assert 'reason=MAP_POLYGONS_MISSING' in h.log_path.read_text()
    finally:
        h.close()


def test_legacy_single_intersection(tmp_path):
    h = Harness(tmp_path, {
        'intersection_ids': [], 'intersection_id': '2', 'virtual_traffic_light_id': 'legacy-vtl',
    })
    try:
        h.load_map()
        h.pump(0.8, objects=[], ego=32)
        assert h.samples[-1][1] == {'legacy-vtl': True}
        h.pump(0.3, objects=[], ego=55)
        assert h.samples[-1][1] == {'legacy-vtl': False}
    finally:
        h.close()


@pytest.mark.parametrize('ids, mappings', [
    (['1', '1'], {'1': {'virtual_traffic_light_id': 'one'}}),
    (['1', '2'], {'1': {'virtual_traffic_light_id': 'same'},
                  '2': {'virtual_traffic_light_id': 'same'}}),
    (['1', '2'], {'1': {'virtual_traffic_light_id': 'one'}}),
    ([''], {}),
])
def test_invalid_mapping_rejected(tmp_path, ids, mappings):
    config = tmp_path / 'invalid.yaml'
    params = {'intersection_ids': ids}
    if mappings:
        params['intersections'] = mappings
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    result = subprocess.run(
        [os.environ['VTL_NODE'], '--ros-args', '--params-file', str(config)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=8)
    assert result.returncode != 0
    assert 'CONFIGURATION:' not in result.stdout
    assert 'ID' in result.stdout
