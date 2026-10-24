#!/usr/bin/env python3
"""Test trans_node2 through a PTY; never opens a physical serial device."""
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '231' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Require isolated ROS domain 231')
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    if not path.startswith('/dev/pts/'):
        raise RuntimeError('PTY only')
    os.set_blocking(master, False)
    rclpy.init()
    node = rclpy.create_node('serial_watchdog_test')
    publisher = node.create_publisher(Twist, '/serial_watchdog_test/command', 1)
    ready = node.create_publisher(UInt8, '/serial_watchdog_test/ready', 1)
    proc = subprocess.Popen([sys.argv[1], '--ros-args',
        '-p', 'serial_port:='+path, '-p', 'command_timeout:=0.25',
        '-r', '/cmd_vel:=/serial_watchdog_test/command',
        '-r', '/nav_ready:=/serial_watchdog_test/ready'])
    buffer = bytearray()
    frames = []

    def collect(duration, command=None, ready_value=None):
        start = time.monotonic()
        next_publish = start
        while time.monotonic()-start < duration:
            if command is not None and time.monotonic() >= next_publish:
                publisher.publish(command)
                if ready_value is not None:
                    ready.publish(UInt8(data=ready_value))
                next_publish = time.monotonic()+.04
            if select.select([master], [], [], .01)[0]:
                buffer.extend(os.read(master, 4096))
            while len(buffer) >= 30:
                if buffer[:2] != b'\xa5\x09':
                    del buffer[0]
                    continue
                frame = bytes(buffer[:30])
                del buffer[:30]
                frames.append((time.monotonic(), struct.unpack('<fff', frame[2:14])))
            rclpy.spin_once(node, timeout_sec=0)
            if proc.poll() is not None:
                raise AssertionError('serial process exited')

    try:
        collect(1.)
        assert frames and all(all(v == 0 for v in f[1]) for f in frames), 'startup nonzero'
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = .12, -.08, .1
        begin = len(frames)
        collect(.6, command)
        assert sum(f[1][0] > .1 for f in frames[begin:]) >= 10, 'output blocked by empty RX'
        stopped_at = time.monotonic()
        collect(.7)
        assert all(all(v == 0 for v in values) for stamp, values in frames
                   if stamp > stopped_at+.35), 'stale velocity repeated beyond watchdog'
        collect(.3, command)
        assert frames[-1][1][0] > .1, 'fresh command did not recover'
        collect(.3, command, ready_value=0)
        assert all(v == 0 for v in frames[-1][1]), 'nav_ready=0 did not stop'
        command.linear.x = float('nan')
        collect(.3, command, ready_value=1)
        assert all(v == 0 for v in frames[-1][1]), 'nonfinite command was sent'
        print('PASS actual serial frames: startup zero, idle RX nonblocking, 250ms watchdog, ready veto, NaN rejection')
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
