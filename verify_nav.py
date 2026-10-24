#!/usr/bin/env python3
"""
导航结果验证脚本：证明「SUCCEEDED」不是假的

做的事：
  1. 一边发 4 个路点，一边把 /amcl_pose 全部录下来（约 20 Hz）
  2. 每个目标点算「轨迹离它最近有多近」
  3. 检查 4 个点是不是按顺序被经过的
  4. 检查总路径长度是否合理（有没有原地打转 / 绕远路）
  5. 打印结论 + 存 CSV 供事后复查

用法：
  ros2 launch ~/fishbot_bringup.launch.py        # 另一个终端
  等 "Managed nodes are active" 和 5 个 active[3]
  python3 -u ~/verify_nav.py
"""

import math
import os
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ---------- 判据（可按需改） ----------
TARGETS = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
TOL = 0.25      # Nav2 默认 xy_goal_tolerance：误差在此以内才算"到点"
NEAR = 0.40     # 判定"经过该点"的宽松半径
PATH_MIN = 3.0  # 理论 4 个点走一圈 ≈ 4 m，低于 3 m 说明没真走
PATH_MAX = 12.0 # 高于 12 m 说明在绕路或反复 recovery


class Recorder(Node):
    """只负责录 /amcl_pose"""

    def __init__(self):
        super().__init__('traj_recorder')
        self.traj = []          # [(t, x, y)]
        self.t0 = None
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb, 10)

    def cb(self, msg):
        p = msg.pose.pose.position
        now = self.get_clock().now().nanoseconds / 1e9   # 仿真时间，秒
        if self.t0 is None:
            self.t0 = now
        self.traj.append((now - self.t0, p.x, p.y))


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
    rec = Recorder()
    nav = BasicNavigator()

    print('[1] 等待 Nav2 激活…', flush=True)
    nav.waitUntilNav2Active()

    # 等第一条位姿（否则没法算起点）
    t_wait = 0.0
    while not rec.traj:
        rclpy.spin_once(rec, timeout_sec=0.2)
        t_wait += 0.2
        if t_wait > 10:
            print('!! 10 秒内没收到 /amcl_pose。'
                  '检查：ros2 topic echo /amcl_pose --once', flush=True)
            return

    print('[2] 起点 (%.2f, %.2f)' % (rec.traj[0][1], rec.traj[0][2]), flush=True)
    poses = [make_pose(nav, x, y) for x, y in TARGETS]
    print('[3] 发 %d 个路点' % len(poses), flush=True)

    # ---------- 一边跑一边录 ----------
    nav.followWaypoints(poses)
    while not nav.isTaskComplete():
        rclpy.spin_once(rec, timeout_sec=0.1)

    result = nav.getResult()
    print('[4] 结果: %s   轨迹点数 %d' % (result, len(rec.traj)), flush=True)

    if len(rec.traj) < 10:
        print('!! 轨迹点太少，无法判定。确认 /amcl_pose 在正常发布。')
        return

    # ---------- 逐点分析 ----------
    rows = []
    hit_idx = []
    for i, (tx, ty) in enumerate(TARGETS):
        dists = [math.hypot(x - tx, y - ty) for _, x, y in rec.traj]
        dmin = min(dists)
        k = dists.index(dmin)
        hit_idx.append(k if dmin <= NEAR else -1)
        rows.append((i + 1, tx, ty, rec.traj[k][0],
                     rec.traj[k][1], dmin, rec.traj[k][0]))

    total = sum(
        math.hypot(rec.traj[i][1] - rec.traj[i - 1][1],
                   rec.traj[i][2] - rec.traj[i - 1][2])
        for i in range(1, len(rec.traj)))

    print('\n序号   目标点          最近实际点       误差(m)   到达时刻(s)  判定')
    print('-' * 72)
    for i, tx, ty, mx, my, dmin, t in rows:
        if dmin <= TOL:
            ok = 'OK'
        elif dmin <= NEAR:
            ok = '近但超差'
        else:
            ok = '未到达'
        print('%3d  (%5.2f,%5.2f)   (%5.2f,%5.2f)   %7.3f   %8.1f   %s'
              % (i, tx, ty, mx, my, dmin, t, ok))

    order_ok = all(hit_idx[i] < hit_idx[i + 1]
                   for i in range(len(hit_idx) - 1)
                   if hit_idx[i] >= 0 and hit_idx[i + 1] >= 0)
    all_hit = all(k >= 0 for k in hit_idx)
    path_ok = PATH_MIN <= total <= PATH_MAX

    print('\n总路径长度 %.2f m（理论 ≈4 m，合理区间 %.1f–%.1f）'
          % (total, PATH_MIN, PATH_MAX))
    print('按顺序经过: %s' % ('是' if order_ok else '否'))
    print('每点都经过: %s' % ('是' if all_hit else '否'))
    print('路径长度合理: %s' % ('是' if path_ok else '否'))

    ok_all = (result == TaskResult.SUCCEEDED and all_hit
              and order_ok and path_ok and all(r[5] <= TOL for r in rows))

    print('\n' + '=' * 60)
    if ok_all:
        print('结论：通过 —— 导航确实按顺序走到了 4 个点，误差都在 %.2f m 内' % TOL)
    else:
        print('结论：不通过 —— 看上面哪一行是「未到达 / 超差 / 否」')
        if result != TaskResult.SUCCEEDED:
            print('  原因1：action 本身没成功（%s）' % result)
        if not all_hit:
            print('  原因2：有路点压根没经过（轨迹离它 > %.1f m）' % NEAR)
        if not order_ok:
            print('  原因3：经过了但顺序不对（可能串点或跳点）')
        if not path_ok:
            print('  原因4：路径长度 %.1f m 不合理（打转或绕路）' % total)
        if not all(r[5] <= TOL for r in rows):
            print('  原因5：到达误差超过 tolerance %.2f m' % TOL)
    print('=' * 60)

    # ---------- 存 CSV ----------
    out = os.path.expanduser(
        '~/traj_%s.csv' % datetime.now().strftime('%m%d_%H%M'))
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_sec', 'x', 'y'])
        w.writerows(rec.traj)
    print('\n轨迹已存: %s  （%d 点，可用 Excel / python 画图）'
          % (out, len(rec.traj)))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
