#!/usr/bin/env python3
"""Live Float64 motor telemetry plot; subscribes before publishers are ready.

No serial access, command publisher, Action introspection, or motion trigger.
"""
import argparse
from collections import deque
import math
import time


class PlotBuffer:
    def __init__(self, window=30.0):
        self.window = window
        self.samples = {name:deque() for name in ('target','rpm')}
        self.received = {}

    def add(self, name, now, value):
        if not math.isfinite(value):
            return
        self.samples[name].append((now,value))
        self.received[name] = now
        self.trim(now)

    def trim(self, now):
        for rows in self.samples.values():
            while rows and rows[0][0] < now-self.window:
                rows.popleft()

    def status(self, now):
        if any(name not in self.received for name in self.samples):
            missing = ', '.join(name for name in self.samples if name not in self.received)
            return f'Waiting for topics: {missing}', False
        if any(now-at > 1.5 for at in self.received.values()):
            return 'Telemetry stale - old samples are history, not current speed', False
        return 'Live - idle RPM=0 is normal; start a goal explicitly', True


def main():
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Float64
    import matplotlib
    matplotlib.use('QtAgg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window',type=float,default=30.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.window) or not 5 <= args.window <= 600:
        parser.error('--window must be 5..600 seconds')
    rclpy.init(args=ros_args)
    node = Node('motor_live_plot')
    buffer = PlotBuffer(args.window)
    started = time.monotonic()
    subscriptions = [
        node.create_subscription(Float64,'/motor/setpoint',
            lambda msg:buffer.add('target',time.monotonic()-started,msg.data),10),
        node.create_subscription(Float64,'/motor/rpm',
            lambda msg:buffer.add('rpm',time.monotonic()-started,msg.data),10),
    ]
    fig = None
    try:
        fig, ax = plt.subplots(figsize=(9,4.5))
        fig.canvas.manager.set_window_title('ESP32 motor - live RPM')
        target, = ax.plot([],[],label='Target RPM (signed)',color='tab:orange',linewidth=2)
        measured, = ax.plot([],[],label='Measured RPM (magnitude)',color='tab:blue',linewidth=1.5)
        label = ax.text(.01,.98,'Waiting for motor topics...',transform=ax.transAxes,
                        ha='left',va='top',fontsize=9)
        ax.set_xlabel('Time since plot start (s)')
        ax.set_ylabel('RPM')
        ax.set_xlim(0,args.window)
        ax.set_ylim(-5,140)
        ax.grid(True,alpha=.3)
        ax.legend(loc='lower right')
        fig.tight_layout()

        def update(_):
            if not rclpy.ok():
                plt.close(fig)
                return target, measured, label
            try:
                for _ in range(8):
                    rclpy.spin_once(node,timeout_sec=0)
            except (KeyboardInterrupt,ExternalShutdownException):
                plt.close(fig)
                return target, measured, label
            now = time.monotonic()-started
            buffer.trim(now)
            for line, name in [(target,'target'),(measured,'rpm')]:
                rows = buffer.samples[name]
                line.set_data([t for t,v in rows],[v for t,v in rows])
            ax.set_xlim(max(0,now-args.window),max(args.window,now))
            values = [v for rows in buffer.samples.values() for t,v in rows]
            ax.set_ylim(min(-5,min(values,default=0)-5),max(20,max(values,default=0)*1.15+5))
            text, live = buffer.status(now)
            label.set_text(text)
            label.set_color('darkgreen' if live else 'darkred')
            return target, measured, label

        animation = FuncAnimation(fig,update,interval=50,cache_frame_data=False)
        plt.show()
        # Keep subscriptions and animation alive for the entire GUI loop.
        del animation, subscriptions
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if fig is not None:
            plt.close(fig)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
