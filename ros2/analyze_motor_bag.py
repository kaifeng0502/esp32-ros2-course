#!/usr/bin/env python3
"""Export recorded motor telemetry to CSV and PNG. No ROS node, publishing or serial I/O."""
import argparse
import csv
import math
from pathlib import Path

TOPICS = {
    '/motor/requested_rpm':'std_msgs/msg/Float64',
    '/motor/setpoint':'std_msgs/msg/Float64',
    '/motor/rpm':'std_msgs/msg/Float64',
    '/motor/pwm':'std_msgs/msg/Float64',
    '/motor/rpm_valid':'std_msgs/msg/Bool',
    '/motor/state':'std_msgs/msg/String',
}


def collect(reader, decode):
    types = {topic.name:topic.type for topic in reader.get_all_topics_and_types()}
    for topic, expected in TOPICS.items():
        if topic in types and types[topic] != expected:
            raise ValueError(f'{topic}: expected {expected}, found {types[topic]}')
    required = {'/motor/setpoint','/motor/rpm','/motor/pwm'}
    if not required <= types.keys():
        raise ValueError('Missing topics: '+', '.join(sorted(required-types.keys())))
    rows = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in TOPICS:
            continue
        value = decode(data, types[topic]).data
        if isinstance(value,float) and not math.isfinite(value):
            raise ValueError(f'Non-finite recorded value on {topic} at {stamp}')
        rows.append((int(stamp),topic,value))
    if not rows or any(not any(r[1] == topic for r in rows) for topic in required):
        raise ValueError('Bag has no samples for one or more required motor topics')
    return sorted(rows,key=lambda r:r[0])


def write_csv(rows, path):
    origin = rows[0][0]
    with path.open('w',newline='',encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['time_s','timestamp_ns','topic','value'])
        for stamp,topic,value in rows:
            writer.writerow([f'{(stamp-origin)/1e9:.9f}',stamp,topic,value])


def series(rows, topic, max_gap=1.5):
    """Break lines across missing telemetry, rather than inventing a continuous trace."""
    x, y = [], []
    origin = rows[0][0]
    previous = None
    for stamp,name,value in rows:
        if name != topic:
            continue
        t = (stamp-origin)/1e9
        if previous is not None and t-previous > max_gap:
            x.append(t)
            y.append(float('nan'))
        x.append(t)
        y.append(float(value))
        previous = t
    return x,y


def draw(rows,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,1,sharex=True,figsize=(10,6))
    try:
        for topic,label,style in [('/motor/requested_rpm','Requested RPM','--'),
                                  ('/motor/setpoint','Board target RPM','-'),
                                  ('/motor/rpm','Measured RPM magnitude','-')]:
            x,y = series(rows,topic)
            if x:
                axes[0].plot(x,y,style,label=label,linewidth=1.6)
        axes[0].set_ylabel('RPM')
        axes[0].legend(loc='best')
        axes[0].set_title('Recorded motor experiment (bag receive timestamps)')
        x,y = series(rows,'/motor/pwm')
        axes[1].step(x,y,where='post',label='PWM',color='tab:purple')
        axes[1].set_ylabel('PWM (%)')
        axes[1].set_ylim(-5,105)
        axes[1].set_xlabel('Time since first recorded motor sample (s)')
        axes[1].legend(loc='best')
        for ax in axes:
            ax.grid(True,alpha=.3)
        fig.tight_layout()
        fig.savefig(path,dpi=160)
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path,help='Bag directory containing metadata.yaml')
    parser.add_argument('--out',type=Path,help='Output directory; default: sibling BAG_analysis')
    args = parser.parse_args()
    bag = args.bag.expanduser().resolve()
    if not (bag/'metadata.yaml').is_file():
        parser.error('Provide a bag directory containing metadata.yaml; stop recording with Ctrl+C first')
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        parser.error(f'{exc}; run in Ubuntu after source /opt/ros/jazzy/setup.bash')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id=''),
                rosbag2_py.ConverterOptions('',''))
    cache = {}
    def decode(data,type_name):
        if type_name not in cache:
            cache[type_name] = get_message(type_name)
        return deserialize_message(data,cache[type_name])
    rows = collect(reader,decode)
    output = args.out.expanduser().resolve() if args.out else bag.with_name(bag.name+'_analysis')
    output.mkdir(parents=True,exist_ok=True)
    write_csv(rows,output/'telemetry.csv')
    draw(rows,output/'curves.png')
    print(f'Recorded motor samples: {len(rows)}')
    print(f'CSV: {output / "telemetry.csv"}')
    print(f'Plot: {output / "curves.png"}')
    print('Measured RPM is magnitude; gaps are not interpolated. Times are bag receipt times.')


if __name__ == '__main__':
    main()
