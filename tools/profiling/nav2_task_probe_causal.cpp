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
#include <pthread.h>
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
    const char* header="event,id,parent,pid,tid,object,kind,wall_start_ns,wall_end_ns,cpu_start_ns,cpu_end_ns,sim_start_ns,sim_end_ns,topic,stamp_ns,signature,aux,message_address,rmw_received_epoch_ns,rmw_source_epoch_ns,publish_epoch_start_ns,publish_epoch_end_ns\n";
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
            const char* topic="",long long stamp=-1,unsigned long long signature=0,long long aux=0,const void* message_ptr=nullptr,long long received=-1,long long source=-1,long long pub_start=-1,long long pub_end=-1) {
  if(!std::getenv("NAV2_PROFILE_DIR")) return;
  char line[2048]; auto n=std::snprintf(line,sizeof(line),"%s,%llu,%llu,%d,%ld,%p,%s,%lld,%lld,%lld,%lld,%lld,%lld,%s,%lld,%llu,%lld,%p,%lld,%lld,%lld,%lld\n",
    event,id,parent,getpid(),syscall(SYS_gettid),obj,kind,ws,we,cs,ce,ss,se,topic,stamp,signature,aux,message_ptr,received,source,pub_start,pub_end);
  if(n>0 && n<int(sizeof(line))) buffer.append(line,n);
}
struct Scope;
thread_local Scope* active_scope=nullptr;
thread_local bool in_record=false;
struct StateSlot {
  std::atomic<uintptr_t> key{0};
  std::atomic<int> category{0}; // 1 local map, 2 global map, 3 unspecified map, 4 noise object
  std::atomic<unsigned long long> version{0},writer{0},trigger{0};
};
StateSlot states[128];
StateSlot* find_state(const void* ptr,int category=0) {
  auto key=reinterpret_cast<uintptr_t>(ptr);if(!key)return nullptr;
  for(auto& v:states) {if(v.key.load(std::memory_order_acquire)==key)return &v;}
  if(!category)return nullptr;
  for(auto& v:states) {uintptr_t empty=0;if(v.key.compare_exchange_strong(empty,key)){v.category.store(category,std::memory_order_release);return &v;}}
  return nullptr;
}
const char* state_kind(StateSlot* v) {int c=v?v->category.load():0;return c==1?"local_costmap":c==2?"global_costmap":c==4?"mppi_noise":"unclassified_shared_state";}
void state_event(const char* event,StateSlot* slot,const void* key,unsigned long long owner,long long w=0) {
  if(!slot)return;
  bool prior=in_record;in_record=true;
  long long wall=w?w:now(CLOCK_MONOTONIC),cpu=now(CLOCK_THREAD_CPUTIME_ID),sim=sim_clock.load();
  record(event,++sequence,owner,key,state_kind(slot),wall,wall,cpu,cpu,sim,sim,"",-1,slot->writer.load(),slot->version.load());
  in_record=prior;
}
struct Scope {
  const char* kind; const void* object; const void* input_ptr=nullptr;Scope* previous=nullptr; pthread_mutex_t* first_mutex=nullptr;unsigned int mutex_depth=0;int tag=0; unsigned long long id,old; long long ws,cs,ss,aux=0,stamp_ns=-1; unsigned long long signature=0; bool success=false;
  Scope(const char* k,const void* o):kind(k),object(o),id(++sequence),old(parent_id) {
    fd();previous=active_scope;active_scope=this;parent_id=id;
    if(!std::strcmp(k,"layered_costmap_update"))tag=1;
    else if(!std::strcmp(k,"mppi_velocity_commands"))tag=2;
    else if(!std::strcmp(k,"mppi_noise_copy"))tag=3;
    else if(!std::strcmp(k,"mppi_noise_reset"))tag=4;
    else if(!std::strcmp(k,"planning_segment"))tag=5;
    else if(!std::strcmp(k,"mppi_noise_trigger"))tag=6; ss=sim_clock.load(std::memory_order_relaxed); ws=now(CLOCK_MONOTONIC); cs=now(CLOCK_THREAD_CPUTIME_ID);
    if(!std::strcmp(k,"mppi_noise_generation")) {
      auto* v=find_state(object,4);if(v)record("trigger_read",++sequence,id,object,"mppi_noise_ready_flag",ws,ws,cs,cs,ss,ss,"",-1,v->trigger.load(),0);
    }
  }
  ~Scope() {
    long long ce=now(CLOCK_THREAD_CPUTIME_ID),we=now(CLOCK_MONOTONIC),se=sim_clock.load(std::memory_order_relaxed);
    if(success && !std::strcmp(kind,"mppi_noise_generation")) {
      auto* v=find_state(object,4);if(v){v->version.fetch_add(1);v->writer.store(id);state_event("state_commit",v,object,id);}
    }
    active_scope=previous;parent_id=old; record(success?"span":"span_exception",id,old,object,kind,ws,we,cs,ce,ss,se,"",stamp_ns,signature,aux,input_ptr);
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
void message(const char* event,const void* handle,const char* topic,const void* data,const rmw_message_info_t* info=nullptr,long long pub_start=-1,long long pub_end=-1) {
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
  record(event,++sequence,parent_id,handle,"message",w,w,c,c,s,s,topic,st,h,aux,data,info?info->received_timestamp:-1,info?info->source_timestamp:-1,pub_start,pub_end);
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

void input_metadata(probe::Scope& span,const geometry_msgs::msg::Twist* msg) {span.input_ptr=msg;span.signature=probe::hash(&msg->linear,sizeof(msg->linear));span.signature=probe::hash(&msg->angular,sizeof(msg->angular),span.signature);}
void input_metadata(probe::Scope& span,const geometry_msgs::msg::TwistStamped* msg) {input_metadata(span,&msg->twist);span.input_ptr=msg;span.stamp_ns=probe::stamp(msg->header.stamp);}
#define VOID1(NAME,SYM,LIB,KIND,TYPE) \
extern "C" void NAME(void* self,void* arg) asm(SYM); \
extern "C" void NAME(void* self,void* arg) {static auto fn=(void(*)(void*,void*))probe::symbol(LIB,SYM); probe::Scope span(KIND,self);input_metadata(span,static_cast<std::shared_ptr<TYPE>*>(arg)->get());fn(self,arg);span.success=true;}
extern "C" void probe_scan(void*,void*) asm("_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");
extern "C" void probe_scan(void* self,void* arg) {
  static auto fn=(void(*)(void*,void*))probe::symbol("libamcl_core.so","_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");
  auto& msg=*static_cast<std::shared_ptr<const sensor_msgs::msg::LaserScan>*>(arg);
  probe::Scope span("amcl_scan_callback",self);span.input_ptr=msg.get();span.stamp_ns=probe::stamp(msg->header.stamp);span.aux=msg->ranges.size();fn(self,arg);span.success=true;
}
VOID1(probe_collision,"_ZN22nav2_collision_monitor16CollisionMonitor25cmdVelInCallbackUnstampedESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE","libcollision_monitor_core.so","collision_check",geometry_msgs::msg::Twist)
VOID1(probe_collision_stamped,"_ZN22nav2_collision_monitor16CollisionMonitor23cmdVelInCallbackStampedESt10shared_ptrIN13geometry_msgs3msg13TwistStamped_ISaIvEEEE","libcollision_monitor_core.so","collision_check_stamped",geometry_msgs::msg::TwistStamped)

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
  long long before=probe::now(CLOCK_REALTIME);auto ret=fn(pub,message,allocation);long long after=probe::now(CLOCK_REALTIME); if(ret==RCL_RET_OK) probe::message("publish",pub,name(pub),message,nullptr,before,after);return ret;
}
extern "C" rcl_ret_t rcl_take(const rcl_subscription_t* sub,void* message,rmw_message_info_t* info,rmw_subscription_allocation_t* allocation) {
  static auto fn=(decltype(&rcl_take))probe::symbol(nullptr,"rcl_take");
  static auto name=(decltype(&rcl_subscription_get_topic_name))probe::symbol(nullptr,"rcl_subscription_get_topic_name");
  auto ret=fn(sub,message,info,allocation);if(ret==RCL_RET_OK)probe::message("take",sub,name(sub),message,info);return ret;
}

// Used by the independent overhead harness; does not execute ROS or robot logic.
extern "C" void nav2_probe_noop() { probe::Scope span("probe_noop",nullptr);span.success=true; }

VOID1(probe_command,"_ZN22nav2_velocity_smoother16VelocitySmoother20inputCommandCallbackESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE","libvelocity_smoother_core.so","velocity_command_callback",geometry_msgs::msg::Twist)
VOID1(probe_command_stamped,"_ZN22nav2_velocity_smoother16VelocitySmoother27inputCommandStampedCallbackESt10shared_ptrIN13geometry_msgs3msg13TwistStamped_ISaIvEEEE","libvelocity_smoother_core.so","velocity_command_stamped_callback",geometry_msgs::msg::TwistStamped)
extern "C" void probe_path(void*,const Path&) asm("_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");
extern "C" void probe_path(void* self,const Path& path) {
 static auto fn=(void(*)(void*,const Path&))probe::symbol("libcontroller_server_core.so","_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");
 probe::Scope span("controller_path_install",self);span.aux=path.poses.size();span.stamp_ns=probe::stamp(path.header.stamp);fn(self,path);span.success=true;
}


// Versioned aliases bypass this interposer and do not need a recursive dlsym bootstrap.
extern "C" int libc_mutex_lock(pthread_mutex_t*);
extern "C" int libc_mutex_unlock(pthread_mutex_t*);
asm(".symver libc_mutex_lock,__pthread_mutex_lock@GLIBC_2.2.5");
asm(".symver libc_mutex_unlock,__pthread_mutex_unlock@GLIBC_2.2.5");

extern "C" int pthread_mutex_lock(pthread_mutex_t* mutex) noexcept {
  auto* scope=probe::active_scope;
  if(!scope || probe::in_record)return libc_mutex_lock(mutex);
  probe::Scope* owner=nullptr;
  for(auto* p=scope;p;p=p->previous)if(p->first_mutex==mutex){owner=p;break;}
  bool selected=scope->tag || owner;
  if(!selected)return libc_mutex_lock(mutex);
  long long ws=probe::now(CLOCK_MONOTONIC),cs=probe::now(CLOCK_THREAD_CPUTIME_ID);
  int result=libc_mutex_lock(mutex);
  long long ce=probe::now(CLOCK_THREAD_CPUTIME_ID),we=probe::now(CLOCK_MONOTONIC);
  if(result)return result;
  if(owner){owner->mutex_depth++;return result;}
  probe::StateSlot* state=nullptr;
  if(scope->tag==1 && !scope->first_mutex) {
    int category=3;
    if(scope->previous && !std::strcmp(scope->previous->kind,"local_costmap_update"))category=1;
    if(scope->previous && !std::strcmp(scope->previous->kind,"global_costmap_update"))category=2;
    state=probe::find_state(mutex,category);
  } else if(scope->tag==3 || scope->tag==4 || scope->tag==6) {
    if(!scope->first_mutex)state=probe::find_state(scope->object,4);
  } else if(scope->tag==2 || scope->tag==5)state=probe::find_state(mutex);
  if(state && !scope->first_mutex) {
    scope->first_mutex=mutex;scope->mutex_depth=1;
    bool prior=probe::in_record;probe::in_record=true;
    long long sim=probe::sim_clock.load();
    probe::record("mutex_acquire",++probe::sequence,scope->id,mutex,probe::state_kind(state),ws,we,cs,ce,sim,sim,"",-1,state->writer.load(),state->version.load());
    if(scope->tag==2 || scope->tag==3 || scope->tag==5)
      probe::state_event("state_read",state,scope->tag==3?scope->object:static_cast<void*>(mutex),scope->id,we);
    probe::in_record=prior;
  }
  return result;
}
extern "C" int pthread_mutex_unlock(pthread_mutex_t* mutex) noexcept {
  auto* scope=probe::active_scope;
  if(!scope || probe::in_record)return libc_mutex_unlock(mutex);
  probe::Scope* owner=nullptr;
  for(auto* p=scope;p;p=p->previous)if(p->first_mutex==mutex){owner=p;break;}
  if(owner && owner->mutex_depth) {
    if(--owner->mutex_depth==0) {
      auto* key=(owner->tag==4 || owner->tag==6 || owner->tag==3)?owner->object:static_cast<void*>(mutex);
      auto* state=probe::find_state(key);
      if(state && (owner->tag==1 || owner->tag==4 || owner->tag==5)) {
        // A planner segment is conservatively a read/write grid access: NavFn clears the start cell.
        state->version.fetch_add(1);state->writer.store(owner->id);if(owner->tag==4)state->trigger.store(owner->id);
        probe::state_event("state_commit",state,key,owner->id);
      } else if(state && owner->tag==6) {
        state->trigger.store(owner->id);probe::state_event("trigger_commit",state,key,owner->id);
      }
      if(state) {
        bool prior=probe::in_record;probe::in_record=true;
        auto w=probe::now(CLOCK_MONOTONIC),c=probe::now(CLOCK_THREAD_CPUTIME_ID),t=probe::sim_clock.load();
        probe::record("mutex_release",++probe::sequence,owner->id,mutex,probe::state_kind(state),w,w,c,c,t,t);
        probe::in_record=prior;
      }
      owner->first_mutex=nullptr;
    }
  }
  return libc_mutex_unlock(mutex);
}

extern "C" void causal_layered(void*,double,double,double) asm("_ZN15nav2_costmap_2d14LayeredCostmap9updateMapEddd");
extern "C" void causal_layered(void* self,double x,double y,double yaw) {
 static auto fn=(void(*)(void*,double,double,double))probe::symbol("libnav2_costmap_2d_core.so","_ZN15nav2_costmap_2d14LayeredCostmap9updateMapEddd");
 probe::Scope span("layered_costmap_update",self);fn(self,x,y,yaw);span.success=true;
}
extern "C" void causal_noise_copy(void*,void*,void*) asm("_ZN4mppi14NoiseGenerator17setNoisedControlsERNS_6models5StateERKNS1_15ControlSequenceE");
extern "C" void causal_noise_copy(void* self,void* state,void* sequence) {
 static auto fn=(void(*)(void*,void*,void*))probe::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator17setNoisedControlsERNS_6models5StateERKNS1_15ControlSequenceE");
 probe::Scope span("mppi_noise_copy",self);fn(self,state,sequence);span.success=true;
}
VOID0(causal_noise_trigger,"_ZN4mppi14NoiseGenerator18generateNextNoisesEv","libmppi_controller.so","mppi_noise_trigger")
extern "C" void causal_noise_reset(void*,void*,bool) asm("_ZN4mppi14NoiseGenerator5resetERNS_6models17OptimizerSettingsEb");
extern "C" void causal_noise_reset(void* self,void* settings,bool holonomic) {
 static auto fn=(void(*)(void*,void*,bool))probe::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator5resetERNS_6models17OptimizerSettingsEb");
 probe::Scope span("mppi_noise_reset",self);fn(self,settings,holonomic);span.success=true;
}
extern "C" geometry_msgs::msg::TwistStamped causal_velocity(void*,const Pose&,const geometry_msgs::msg::Twist&,void*) asm("_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" geometry_msgs::msg::TwistStamped causal_velocity(void* self,const Pose& pose,const geometry_msgs::msg::Twist& speed,void* checker) {
 static auto fn=(geometry_msgs::msg::TwistStamped(*)(void*,const Pose&,const geometry_msgs::msg::Twist&,void*))probe::symbol("libmppi_controller.so","_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
 probe::Scope span("mppi_velocity_commands",self);auto out=fn(self,pose,speed,checker);span.success=true;return out;
}

extern "C" int nav2_probe_resource_check() {
 pthread_mutexattr_t attr;pthread_mutexattr_init(&attr);pthread_mutexattr_settype(&attr,PTHREAD_MUTEX_RECURSIVE);
 pthread_mutex_t map,noise;pthread_mutex_init(&map,&attr);pthread_mutex_init(&noise,&attr);pthread_mutexattr_destroy(&attr);
 int object=0;
 {probe::Scope outer("local_costmap_update",&map);{probe::Scope layer("layered_costmap_update",&map);pthread_mutex_lock(&map);pthread_mutex_lock(&map);pthread_mutex_unlock(&map);pthread_mutex_unlock(&map);layer.success=true;}outer.success=true;}
 {probe::Scope reader("mppi_velocity_commands",&map);pthread_mutex_lock(&map);pthread_mutex_unlock(&map);reader.success=true;}
 auto* m=probe::find_state(&map);if(!m || m->version.load()!=1)return 10;
 pthread_mutex_lock(&noise);{probe::Scope producer("mppi_noise_generation",&object);producer.success=true;}pthread_mutex_unlock(&noise);
 {probe::Scope reader("mppi_noise_copy",&object);pthread_mutex_lock(&noise);pthread_mutex_unlock(&noise);reader.success=true;}
 auto* n=probe::find_state(&object);if(!n || n->version.load()!=1)return 11;
 pthread_mutex_destroy(&map);pthread_mutex_destroy(&noise);return 0;
}

VOID0(causal_trajectories,"_ZN4mppi9Optimizer26generateNoisedTrajectoriesEv","libmppi_controller.so","mppi_noised_trajectories")
VOID0(causal_update_sequence,"_ZN4mppi9Optimizer21updateControlSequenceEv","libmppi_controller.so","mppi_control_sequence_update")
extern "C" void causal_critics(void*,void*) asm("_ZNK4mppi13CriticManager22evalTrajectoriesScoresERNS_10CriticDataE");
extern "C" void causal_critics(void* self,void* data) {
 static auto fn=(void(*)(void*,void*))probe::symbol("libmppi_controller.so","_ZNK4mppi13CriticManager22evalTrajectoriesScoresERNS_10CriticDataE");
 probe::Scope span("mppi_critic_scoring",self);fn(self,data);span.success=true;
}
