#!/usr/bin/env python3
"""External observation only: retain source stamps and observer receipt times."""
import argparse
import json
from pathlib import Path
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

def stamp(s):return s.sec*1000000000+s.nanosec

class Observer(Node):
    def __init__(self,file):
        super().__init__('task_model_observer')
        self.file=file;self.clock=-1;self.sequence=0;self.last_flush=time.monotonic()
        self.create_subscription(Clock,'/clock',self.on_clock,qos_profile_sensor_data)
        for topic,type_ in [('/scan',LaserScan),('/odom',Odometry),('/cmd_vel_nav',Twist),('/cmd_vel_smoothed',Twist),('/cmd_vel',Twist),('/plan',NavPath)]:
            self.create_subscription(type_,topic,lambda m,t=topic:self.record(t,m),qos_profile_sensor_data)
        qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:self.record('/amcl_pose',m),qos)
    def on_clock(self,m):self.clock=stamp(m.clock);self.record('/clock',m)
    def record(self,topic,msg):
        self.sequence+=1
        d={'seq':self.sequence,'topic':topic,'receipt_mono_ns':time.monotonic_ns(),'receipt_epoch_ns':time.time_ns(),'latest_received_sim_ns':self.clock}
        if hasattr(msg,'header'):d['source_stamp_ns']=stamp(msg.header.stamp)
        if topic=='/clock':d['source_stamp_ns']=stamp(msg.clock)
        elif topic=='/scan':d.update(ranges=len(msg.ranges),intensities=len(msg.intensities),scan_time_s=msg.scan_time,time_increment_s=msg.time_increment)
        elif topic=='/plan':d['poses']=len(msg.poses)
        elif topic=='/odom':d.update(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,vx=msg.twist.twist.linear.x,wz=msg.twist.twist.angular.z)
        elif 'cmd_vel' in topic:d.update(vx=msg.linear.x,wz=msg.angular.z)
        elif topic=='/amcl_pose':d.update(x=msg.pose.pose.position.x,y=msg.pose.pose.position.y)
        self.file.write(json.dumps(d,separators=(',',':'))+'\n')
        if time.monotonic()-self.last_flush>1:self.file.flush();self.last_flush=time.monotonic()

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    rclpy.init();running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with args.output.open('w') as f:
        n=Observer(f)
        try:
            while running and rclpy.ok():rclpy.spin_once(n,timeout_sec=.1)
        finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
