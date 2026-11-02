#!/usr/bin/env python3
"""Send a goal, show feedback/result, and explicitly request cancellation."""
import argparse
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.srv import CancelGoal
from rcl_interfaces.srv import GetParameters
from motor_action_interfaces.action import MoveWheel


def wait(node, future, seconds):
    deadline = time.monotonic()+seconds
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if not future.done():
        raise TimeoutError('ROS response timed out; check the server terminal.')
    return future.result()


def feedback(message):
    row = message.feedback
    position = (f'位置={row.current_turns:+.4f}/{row.target_turns:+g}圈' if row.target_turns else
                f'时间进度；累计位置={row.current_turns:+.4f}圈')
    print(f'{row.progress_percent:5.1f}%  {position}  '
          f'RPM={row.rpm:6.1f}  PWM={row.pwm_percent:5.1f}%  {row.phase}', flush=True)


def configured_goal(node):
    service = node.create_client(GetParameters, '/motor_action_server/get_parameters')
    try:
        if not service.wait_for_service(timeout_sec=5):
            raise RuntimeError('Parameter service not found; start the new Action server first.')
        response = wait(node, service.call_async(GetParameters.Request(
            names=['target_turns','target_rpm','run_seconds'])),5)
        values = response.values
        if len(values) != 3 or [v.type for v in values] != [3,2,2]:
            raise RuntimeError('Parameter schema mismatch; update/restart motor_action_server.py.')
        return values[0].double_value, values[1].integer_value, values[2].integer_value
    finally:
        node.destroy_client(service)


def main():
    parser = argparse.ArgumentParser()
    goal = parser.add_mutually_exclusive_group(required=True)
    goal.add_argument('--turns', type=float)
    goal.add_argument('--degrees', type=float)
    goal.add_argument('--cancel-all', action='store_true', help='Request cancellation of the active goal')
    goal.add_argument('--configured', action='store_true', help='Read goal defaults from the server ROS parameters')
    parser.add_argument('--rpm', type=int, default=None)
    parser.add_argument('--timeout', type=int, default=None)
    parser.add_argument('--cancel-after', type=float, help='Seconds after goal acceptance')
    args, ros_args = parser.parse_known_args()
    if args.cancel_after is not None and not 0 <= args.cancel_after <= 60:
        parser.error('--cancel-after must be 0..60 seconds')
    if args.configured and (args.rpm is not None or args.timeout is not None):
        parser.error('--configured reads RPM/time from server parameters; omit --rpm and --timeout')
    args.rpm = 60 if args.rpm is None else args.rpm
    args.timeout = 15 if args.timeout is None else args.timeout
    if not 0 <= args.rpm <= 65535 or not 0 <= args.timeout <= 65535:
        parser.error('rpm/timeout must fit uint16; server further limits RPM to 40..180 and timeout to 1..60')
    rclpy.init(args=ros_args)
    node = Node('motor_action_client')
    client = None
    try:
        if args.cancel_all:
            service = node.create_client(CancelGoal, '/motor/move_wheel/_action/cancel_goal')
            if not service.wait_for_service(timeout_sec=5):
                raise RuntimeError('Action server not found.')
            response = wait(node, service.call_async(CancelGoal.Request()), 5)
            print(f'取消请求返回 code={response.return_code}，接受取消的目标数={len(response.goals_canceling)}。')
            print('这表示取消请求的答复；驱动是否关闭，请看原客户端的最终 Result。')
            return
        client = ActionClient(node, MoveWheel, '/motor/move_wheel')
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Action server not found; start motor_action_server.py first.')
        if args.configured:
            turns, args.rpm, args.timeout = configured_goal(node)
            print(f'读取 ROS 参数：target_turns={turns:g}, target_rpm={args.rpm}, run_seconds={args.timeout}',flush=True)
        else:
            turns = args.turns if args.turns is not None else args.degrees/360
        handle = wait(node, client.send_goal_async(
            MoveWheel.Goal(turns=turns, rpm=args.rpm, timeout_sec=args.timeout),
            feedback_callback=feedback), 5)
        if not handle.accepted:
            raise RuntimeError('目标被拒绝，请查看服务器窗口中的原因。')
        print('目标已接受。', flush=True)
        result_future = handle.get_result_async()
        accepted_at = time.monotonic()
        canceled = False
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=.1)
            if (args.cancel_after is not None and not canceled
                    and time.monotonic()-accepted_at >= args.cancel_after):
                canceled = True
                response = wait(node, handle.cancel_goal_async(), 5)
                print(f'已请求取消：code={response.return_code}；等待最终结果。', flush=True)
            if time.monotonic()-accepted_at > args.timeout+12:
                raise TimeoutError('等待最终结果超时；检查服务器。ESP32 仍受板上运行时限约束。')
        result = result_future.result()
        labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        row = result.result
        print(f'最终状态：{labels.get(result.status, result.status)}\n'
              f'成功={row.success}，驱动已关闭={row.drive_disabled}\n'
              f'估算位置={row.final_turns:+.5f}圈，误差={row.error_degrees:+.2f}°\n{row.message}')
    except KeyboardInterrupt:
        print('\n客户端退出。退出客户端不等于取消；需要取消时用 --cancel-all。')
    except (RuntimeError, TimeoutError) as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if client is not None:
            client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
