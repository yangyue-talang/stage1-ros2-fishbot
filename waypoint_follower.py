#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.w = 1.0
    return p


def main():
    rclpy.init()
    nav = BasicNavigator()
    print('[1] BasicNavigator 已建，等待 Nav2 激活…', flush=True)

    nav.waitUntilNav2Active()
    print('[2] Nav2 已激活，开始发 4 个路点', flush=True)

    points = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    poses = [make_pose(nav, x, y) for x, y in points]
    nav.followWaypoints(poses)

    last_wp, t0 = -1, time.time()
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None and fb.current_waypoint != last_wp:
            last_wp = fb.current_waypoint
            print('[3] 前往第 %d 个点  t=%.1fs' % (last_wp + 1, time.time() - t0),
                  flush=True)
        time.sleep(0.5)

    result = nav.getResult()
    print('[4] 结果: %s  总耗时 %.1fs' % (result, time.time() - t0), flush=True)
    print('[5] 成功' if result == TaskResult.SUCCEEDED else '[5] 失败', flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
