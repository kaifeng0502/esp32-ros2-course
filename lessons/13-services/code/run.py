#!/usr/bin/env python3
"""Request the stop service; this client never starts a motor."""
def main():
    import argparse
    argparse.ArgumentParser(description=__doc__).parse_args()
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('course_stop_client')
    try:
        client=node.create_client(Trigger,'/motor/stop')
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('No /motor/stop service; start the enabled bridge first')
        future=client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=4)
        if not future.done():raise RuntimeError('Stop confirmation timed out')
        response=future.result();print('success:',response.success,'message:',response.message)
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
