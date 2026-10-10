#!/usr/bin/env python3
"""Record actual ROS odometry/commands with their simulation clock, without publishing."""
import argparse,csv,time,math,struct,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import GetState
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
import json

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--ready-file',type=Path);parser.add_argument('--vehicles',type=int,default=1);args=parser.parse_args()
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from tools.live_bridge.broker import runtime_paths
    clock_path=runtime_paths(args.output.parent)[1]
    timing=(args.output.parent/'native-timing.csv').open('w',newline='',buffering=1);tw=csv.writer(timing);tw.writerow(['host_mono_ns','broker_sim_ns','observer_clock_ns','topic','message_stamp_ns','frame_id','child_frame_id'])
    def native(topic,stamp,frame='',child=''):
        try:broker=struct.unpack('<Q',clock_path.read_bytes()[:8])[0]
        except (OSError,struct.error):broker=0
        tw.writerow([time.monotonic_ns(),broker,sim[0],topic,stamp,frame,child])
    rclpy.init();node=Node('live_bridge_observer');sim=[0];subs=[];previous={};distances={};last_write=[0.]
    with args.output.open('w',newline='',buffering=1) as output:
        writer=csv.writer(output);writer.writerow(['host_mono_ns','sim_ns','device','topic','x','y','vx','wz'])
        def clock(m):
            value=m.clock.sec*10**9+m.clock.nanosec;previous=sim[0];sim[0]=value
            if value//10000000!=previous//10000000 or value<previous:native('/clock',value)
        subs.append(node.create_subscription(Clock,'/clock',clock,10))
        def transforms(m):
            for t in m.transforms:native('/tf',t.header.stamp.sec*10**9+t.header.stamp.nanosec,t.header.frame_id,t.child_frame_id)
        subs.append(node.create_subscription(TFMessage,'/tf',transforms,100))
        subs.append(node.create_subscription(LaserScan,'/scan',lambda m:native('/scan',m.header.stamp.sec*10**9+m.header.stamp.nanosec,m.header.frame_id),qos_profile_sensor_data))
        for device,color in enumerate(('red','blue','white','green')[:args.vehicles]):
            ns='' if color=='red' else '/'+color
            def odom(m,d=device):
                stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec;p=(m.pose.pose.position.x,m.pose.pose.position.y)
                writer.writerow([time.monotonic_ns(),stamp,d,'odom',*p,m.twist.twist.linear.x,m.twist.twist.angular.z])
                if args.ready_file and (args.ready_file.parent/'armed').exists():
                    if d in previous:distances[d]=distances.get(d,0.)+math.dist(previous[d],p)
                    previous[d]=p
                    if time.monotonic()-last_write[0]>=.1:
                        target=args.output.parent/'distance-progress.json';tmp=target.with_suffix('.tmp');tmp.write_text(json.dumps({'sim_ns':stamp,'distance_m':distances})+'\n');tmp.replace(target);last_write[0]=time.monotonic()

            def command(m,d=device):writer.writerow([time.monotonic_ns(),sim[0],d,'cmd_vel','','',m.linear.x,m.angular.z])
            subs.append(node.create_subscription(Odometry,ns+'/odom',odom,qos_profile_sensor_data))
            for topic in ('cmd_vel_nav','cmd_vel_smoothed','cmd_vel'):
                def command(m,d=device,t=topic):writer.writerow([time.monotonic_ns(),sim[0],d,t,'','',m.linear.x,m.angular.z])
                subs.append(node.create_subscription(Twist,ns+'/'+topic,command,10))
        clients={};futures={};states={}
        if args.ready_file:
            for color in ('red','blue','white','green')[:args.vehicles]:
                ns='' if color=='red' else '/'+color
                for name in ('amcl','controller_server','planner_server','bt_navigator','velocity_smoother','collision_monitor'):
                    topic=ns+'/'+name+'/get_state';clients[topic]=node.create_client(GetState,topic)
            def check_ready():
                for topic,client in clients.items():
                    f=futures.get(topic)
                    if f and f.done():
                        try:states[topic]=f.result().current_state.label
                        except Exception:states[topic]='unavailable'
                        futures.pop(topic,None)
                    if topic not in futures and client.service_is_ready():futures[topic]=client.call_async(GetState.Request())
                args.ready_file.with_suffix('.json').write_text(json.dumps(states,indent=2)+'\n')
                if len(states)==len(clients) and all(s=='active' for s in states.values()):args.ready_file.touch()
            node.create_timer(.5,check_ready)
        try:rclpy.spin(node)
        except KeyboardInterrupt:pass
        finally:timing.close();node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
