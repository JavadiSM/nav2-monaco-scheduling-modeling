// Non-scheduling profiling shim for the existing ROS 2 / Nav2 binaries.
// Each span reports its executing thread's CPU time, not elapsed time as WCET.
#define _GNU_SOURCE
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <functional>
#include <memory>
#include <string>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <nav2_amcl/pf/pf.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>

namespace probe {
std::atomic<unsigned long long> sequence{0};
std::atomic<long long> sim_clock{-1};
thread_local unsigned long long parent_id=0;
long long now(clockid_t c) { timespec t{}; clock_gettime(c,&t); return t.tv_sec*1000000000LL+t.tv_nsec; }
int fd() {
  static int file=[] {
    const char* dir=std::getenv("NAV2_PROFILE_DIR"); if(!dir) return -1;
    char name[4096]; std::snprintf(name,sizeof(name),"%s/events-%d.csv",dir,getpid());
    int f=open(name,O_CREAT|O_WRONLY|O_APPEND|O_CLOEXEC,0644);
    const char* header="event,id,parent,pid,tid,object,kind,wall_start_ns,wall_end_ns,cpu_start_ns,cpu_end_ns,sim_start_ns,sim_end_ns,topic,stamp_ns,signature,aux\n";
    if(f>=0) { write(f,header,std::strlen(header)); }
    return f;
  }(); return file;
}
struct Buffer {
  char data[65536]; size_t used=0; long long last_flush=0;
  void flush() { int f=fd(); if(used && f>=0) { size_t done=0; while(done<used) { auto n=write(f,data+done,used-done); if(n<=0) break; done+=n; } } used=0; }
  void append(const char* line,size_t n) { if(used+n>sizeof(data)) flush(); if(n<=sizeof(data)) { std::memcpy(data+used,line,n); used+=n; } long long current=now(CLOCK_MONOTONIC);if(current-last_flush>=1000000000LL){flush();last_flush=current;} }
  ~Buffer() { flush(); }
};
thread_local Buffer buffer;
void record(const char* event,unsigned long long id,unsigned long long parent,const void* obj,const char* kind,
            long long ws,long long we,long long cs,long long ce,long long ss,long long se,
            const char* topic="",long long stamp=-1,unsigned long long signature=0,long long aux=0) {
  if(!std::getenv("NAV2_PROFILE_DIR")) return;
  char line[2048]; auto n=std::snprintf(line,sizeof(line),"%s,%llu,%llu,%d,%ld,%p,%s,%lld,%lld,%lld,%lld,%lld,%lld,%s,%lld,%llu,%lld\n",
    event,id,parent,getpid(),syscall(SYS_gettid),obj,kind,ws,we,cs,ce,ss,se,topic,stamp,signature,aux);
  if(n>0 && n<int(sizeof(line))) buffer.append(line,n);
}
struct Scope {
  const char* kind; const void* object; unsigned long long id,old; long long ws,cs,ss,aux=0,stamp_ns=-1; unsigned long long signature=0; bool success=false;
  Scope(const char* k,const void* o):kind(k),object(o),id(++sequence),old(parent_id) {
    fd(); parent_id=id; ss=sim_clock.load(std::memory_order_relaxed); ws=now(CLOCK_MONOTONIC); cs=now(CLOCK_THREAD_CPUTIME_ID);
  }
  ~Scope() {
    long long ce=now(CLOCK_THREAD_CPUTIME_ID),we=now(CLOCK_MONOTONIC),se=sim_clock.load(std::memory_order_relaxed);
    parent_id=old; record(success?"span":"span_exception",id,old,object,kind,ws,we,cs,ce,ss,se,"",stamp_ns,signature,aux);
  }
};
void* symbol(const char* lib,const char* name) {
  void* f=nullptr;
  if(lib) { std::string path=std::string("/opt/ros/jazzy/lib/")+lib; void* h=dlopen(path.c_str(),RTLD_NOW|RTLD_NOLOAD); if(h) f=dlsym(h,name); }
  else {
    f=dlsym(RTLD_NEXT,name);
    // rclpy / composable plugins may have loaded librcl with RTLD_LOCAL.
    if(!f) { void* h=dlopen("/opt/ros/jazzy/lib/librcl.so",RTLD_NOW|RTLD_NOLOAD);if(h) f=dlsym(h,name); }
  }
  if(!f) { std::fprintf(stderr,"NAV2 probe cannot resolve %s: %s\n",name,dlerror()); std::abort(); }
  return f;
}
template<class T> long long stamp(const T& t) { return (long long)t.sec*1000000000LL+t.nanosec; }
unsigned long long hash(const void* p,size_t n,unsigned long long h=1469598103934665603ULL) {
  const auto* b=(const unsigned char*)p; for(size_t i=0;i<n;i++) {h^=b[i];h*=1099511628211ULL;} return h;
}
bool tracked(const char* t) {
  if(!t) return false;
  return !std::strcmp(t,"/scan") || !std::strcmp(t,"/odom") || !std::strcmp(t,"/cmd_vel_nav") ||
    !std::strcmp(t,"/cmd_vel_smoothed") || !std::strcmp(t,"/cmd_vel") || !std::strcmp(t,"/plan") ||
    !std::strcmp(t,"/amcl_pose") || !std::strcmp(t,"/tf") || std::strstr(t,"/_action/");
}
void message(const char* event,const void* handle,const char* topic,const void* data) {
  if(!topic || !data) return;
  if(!std::strcmp(topic,"/clock")) { const auto* m=(const rosgraph_msgs::msg::Clock*)data; sim_clock.store(stamp(m->clock),std::memory_order_relaxed); return; }
  if(!tracked(topic)) return;
  long long st=-1,aux=0; unsigned long long h=0;
  if(!std::strcmp(topic,"/scan")) {auto*m=(const sensor_msgs::msg::LaserScan*)data;st=stamp(m->header.stamp);aux=m->ranges.size();h=hash(m->ranges.data(),m->ranges.size()*sizeof(float));}
  else if(!std::strcmp(topic,"/odom")) {auto*m=(const nav_msgs::msg::Odometry*)data;st=stamp(m->header.stamp);h=hash(&m->pose.pose.position.x,sizeof(double));h=hash(&m->pose.pose.position.y,sizeof(double),h);}
  else if(!std::strcmp(topic,"/plan")) {auto*m=(const nav_msgs::msg::Path*)data;st=stamp(m->header.stamp);aux=m->poses.size(); if(aux) h=hash(&m->poses.back().pose.position.x,sizeof(double));}
  else if(!std::strcmp(topic,"/tf")) {auto*m=(const tf2_msgs::msg::TFMessage*)data;aux=m->transforms.size(); if(aux) {st=stamp(m->transforms.back().header.stamp);h=hash(m->transforms.back().child_frame_id.data(),m->transforms.back().child_frame_id.size());}}
  else if(std::strstr(topic,"cmd_vel")) {auto*m=(const geometry_msgs::msg::Twist*)data;h=hash(&m->linear,sizeof(m->linear));h=hash(&m->angular,sizeof(m->angular),h);}
  long long w=now(CLOCK_MONOTONIC),c=now(CLOCK_THREAD_CPUTIME_ID),s=sim_clock.load(std::memory_order_relaxed);
  record(event,++sequence,parent_id,handle,"message",w,w,c,c,s,s,topic,st,h,aux);
}
}

#define VOID0(NAME,SYM,LIB,KIND) \
extern "C" void NAME(void* self) asm(SYM); \
extern "C" void NAME(void* self) { static auto fn=(void(*)(void*))probe::symbol(LIB,SYM); probe::Scope span(KIND,self); fn(self); span.success=true; }
VOID0(probe_control,"_ZN15nav2_controller16ControllerServer25computeAndPublishVelocityEv","libcontroller_server_core.so","control_iteration")
VOID0(probe_plan,"_ZN12nav2_planner13PlannerServer23computePlanThroughPosesEv","libplanner_server_core.so","planning_request")
extern "C" void probe_costmap(void*) asm("_ZN15nav2_costmap_2d12Costmap2DROS9updateMapEv");
extern "C" void probe_costmap(void* self) {
  static auto fn=(void(*)(void*))probe::symbol("libnav2_costmap_2d_core.so","_ZN15nav2_costmap_2d12Costmap2DROS9updateMapEv");
  const auto name=static_cast<nav2_costmap_2d::Costmap2DROS*>(self)->getName();
  const char* kind=name.find("local")!=std::string::npos?"local_costmap_update":name.find("global")!=std::string::npos?"global_costmap_update":"costmap_update_unknown";
  probe::Scope span(kind,self);fn(self);span.success=true;
}
VOID0(probe_smooth,"_ZN22nav2_velocity_smoother16VelocitySmoother13smootherTimerEv","libvelocity_smoother_core.so","velocity_smoothing_tick")
VOID0(probe_noise,"_ZN4mppi14NoiseGenerator22generateNoisedControlsEv","libmppi_controller.so","mppi_noise_generation")
VOID0(probe_optimize,"_ZN4mppi9Optimizer8optimizeEv","libmppi_controller.so","mppi_optimize")

#define VOID1(NAME,SYM,LIB,KIND) \
extern "C" void NAME(void* self,void* arg) asm(SYM); \
extern "C" void NAME(void* self,void* arg) {static auto fn=(void(*)(void*,void*))probe::symbol(LIB,SYM); probe::Scope span(KIND,self);fn(self,arg);span.success=true;}
extern "C" void probe_scan(void*,void*) asm("_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");
extern "C" void probe_scan(void* self,void* arg) {
  static auto fn=(void(*)(void*,void*))probe::symbol("libamcl_core.so","_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");
  auto& msg=*static_cast<std::shared_ptr<const sensor_msgs::msg::LaserScan>*>(arg);
  probe::Scope span("amcl_scan_callback",self);span.stamp_ns=probe::stamp(msg->header.stamp);span.aux=msg->ranges.size();fn(self,arg);span.success=true;
}
VOID1(probe_collision,"_ZN22nav2_collision_monitor16CollisionMonitor25cmdVelInCallbackUnstampedESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE","libcollision_monitor_core.so","collision_check")
VOID1(probe_collision_stamped,"_ZN22nav2_collision_monitor16CollisionMonitor23cmdVelInCallbackStampedESt10shared_ptrIN13geometry_msgs3msg13TwistStamped_ISaIvEEEE","libcollision_monitor_core.so","collision_check_stamped")

extern "C" bool probe_filter(void*,void*,void*,void*) asm("_ZN9nav2_amcl8AmclNode12updateFilterERKiRKSt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEERK11pf_vector_t");
extern "C" bool probe_filter(void* self,void* i,void* scan,void* pose) {
  static auto fn=(bool(*)(void*,void*,void*,void*))probe::symbol("libamcl_core.so","_ZN9nav2_amcl8AmclNode12updateFilterERKiRKSt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEERK11pf_vector_t");
  probe::Scope span("amcl_filter_update",self);bool v=fn(self,i,scan,pose);span.success=true;span.aux=v;return v;
}
extern "C" void probe_particles(void*,const pf_sample_set_t*) asm("_ZN9nav2_amcl8AmclNode20publishParticleCloudEPK16_pf_sample_set_t");
extern "C" void probe_particles(void* self,const pf_sample_set_t* set) {
  static auto fn=(void(*)(void*,const pf_sample_set_t*))probe::symbol("libamcl_core.so","_ZN9nav2_amcl8AmclNode20publishParticleCloudEPK16_pf_sample_set_t");
  probe::Scope span("amcl_particle_publication",self);span.aux=set->sample_count;fn(self,set);span.success=true;
}

extern "C" int probe_bt(void*) asm("_ZN2BT4Tree8tickOnceEv");
extern "C" int probe_bt(void* self) {static auto fn=(int(*)(void*))probe::symbol("libbehaviortree_cpp.so","_ZN2BT4Tree8tickOnceEv");probe::Scope span("bt_tick",self);int v=fn(self);span.success=true;span.aux=v;return v;}

using Path=nav_msgs::msg::Path; using Pose=geometry_msgs::msg::PoseStamped;
extern "C" Path probe_segment(void*,const Pose&,const Pose&,const std::string&,std::function<bool()>) asm("_ZN12nav2_planner13PlannerServer7getPlanERKN13geometry_msgs3msg12PoseStamped_ISaIvEEES7_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt8functionIFbvEE");
extern "C" Path probe_segment(void* self,const Pose& start,const Pose& goal,const std::string& planner,std::function<bool()> cancel) {
  static auto fn=(Path(*)(void*,const Pose&,const Pose&,const std::string&,std::function<bool()>))probe::symbol("libplanner_server_core.so","_ZN12nav2_planner13PlannerServer7getPlanERKN13geometry_msgs3msg12PoseStamped_ISaIvEEES7_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEESt8functionIFbvEE");
  probe::Scope span("planning_segment",self);auto v=fn(self,start,goal,planner,std::move(cancel));span.success=true;span.aux=v.poses.size();return v;
}

extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* pub,const void* message,rmw_publisher_allocation_t* allocation) {
  static auto fn=(decltype(&rcl_publish))probe::symbol(nullptr,"rcl_publish");
  static auto name=(decltype(&rcl_publisher_get_topic_name))probe::symbol(nullptr,"rcl_publisher_get_topic_name");
  auto ret=fn(pub,message,allocation); if(ret==RCL_RET_OK) probe::message("publish",pub,name(pub),message);return ret;
}
extern "C" rcl_ret_t rcl_take(const rcl_subscription_t* sub,void* message,rmw_message_info_t* info,rmw_subscription_allocation_t* allocation) {
  static auto fn=(decltype(&rcl_take))probe::symbol(nullptr,"rcl_take");
  static auto name=(decltype(&rcl_subscription_get_topic_name))probe::symbol(nullptr,"rcl_subscription_get_topic_name");
  auto ret=fn(sub,message,info,allocation);if(ret==RCL_RET_OK)probe::message("take",sub,name(sub),message);return ret;
}

// Used by the independent overhead harness; does not execute ROS or robot logic.
extern "C" void nav2_probe_noop() { probe::Scope span("probe_noop",nullptr);span.success=true; }

VOID1(probe_command,"_ZN22nav2_velocity_smoother16VelocitySmoother20inputCommandCallbackESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE","libvelocity_smoother_core.so","velocity_command_callback")
VOID1(probe_command_stamped,"_ZN22nav2_velocity_smoother16VelocitySmoother27inputCommandStampedCallbackESt10shared_ptrIN13geometry_msgs3msg13TwistStamped_ISaIvEEEE","libvelocity_smoother_core.so","velocity_command_stamped_callback")
extern "C" void probe_path(void*,const Path&) asm("_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");
extern "C" void probe_path(void* self,const Path& path) {
 static auto fn=(void(*)(void*,const Path&))probe::symbol("libcontroller_server_core.so","_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");
 probe::Scope span("controller_path_install",self);span.aux=path.poses.size();span.stamp_ns=probe::stamp(path.header.stamp);fn(self,path);span.success=true;
}
