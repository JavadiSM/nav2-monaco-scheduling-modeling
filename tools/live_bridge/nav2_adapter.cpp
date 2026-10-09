// Live ROS output adapter. Algorithm implementations remain in installed Nav2 libraries.
#define _GNU_SOURCE
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <execinfo.h>
#include <signal.h>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <pthread.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <nav2_planner/planner_server.hpp>
#include <nav2_mppi_controller/tools/noise_generator.hpp>
#include <rcl_action/action_client.h>
#include <rcl_action/action_server.h>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <rclcpp/rate.hpp>
#include <rcl/timer.h>
#include <rcl/client.h>
#include <rcl/service.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rmw/rmw.h>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
namespace live {
using U=uint64_t;
enum {ENTER=1,DEP,STAGE,END,WAIT,RESUME,SLEEP,PUBLISH,LOOKUP,DEVICE_GATE,START_READY,START_ACK,TRIGGER,RESULT_OWNER,RESULT_GATE,PUBLISH_BEGIN};
struct Packet {U op,a,b,c,d,e,f;};
thread_local bool internal=false;thread_local int socket_fd=-1;
thread_local U job=0;thread_local int kind=-1;
thread_local U excluded_cpu_ns=0;thread_local unsigned pause_depth=0;thread_local bool flushing=false;
thread_local std::vector<std::function<void()>> pending_outputs;
thread_local std::vector<pthread_mutex_t*> held_commits;
thread_local std::vector<std::pair<void(*)(void*),void*>> deferred;
void defer_noise(void(*fn)(void*),void* obj){for(auto& item:deferred)if(item.second==obj)return;deferred.push_back({fn,obj});}
U* clock_words() {
 static U* p=[](){const char* name=getenv("NAV2_LIVE_CLOCK");if(!name)return (U*)nullptr;
 int f=open(name,O_RDWR|O_CLOEXEC);if(f<0){perror("live clock");abort();}void* v=mmap(nullptr,32,PROT_READ|PROT_WRITE,MAP_SHARED,f,0);close(f);if(v==MAP_FAILED)abort();return (U*)v;}();return p;
}
U now(){auto p=clock_words();return p?__atomic_load_n(p,__ATOMIC_ACQUIRE):0;}
bool armed(){auto p=clock_words();return p&&__atomic_load_n(p+1,__ATOMIC_ACQUIRE)&&!__atomic_load_n(p+2,__ATOMIC_ACQUIRE);}
U epoch(){timespec t{};clock_gettime(CLOCK_REALTIME,&t);return U(t.tv_sec)*1000000000+t.tv_nsec;}
U cpu(){timespec t{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t);return U(t.tv_sec)*1000000000+t.tv_nsec;}
struct MeterPause {
 U started=cpu();bool outer=(pause_depth++==0);
 ~MeterPause(){if(!--pause_depth&&outer)excluded_cpu_ns+=cpu()-started;}
};
[[noreturn]] void fail(const char* s){fprintf(stderr,"Live bridge: %s\n",s);_exit(90);}
U rpc(U op,U a=0,U b=0,U c=0,U d=0,U e=0,U f=0){
 MeterPause pause;bool old=internal;internal=true;
 if(socket_fd<0){socket_fd=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);sockaddr_un addr{};addr.sun_family=AF_UNIX;const char* name=getenv("NAV2_LIVE_SOCKET");if(!name||strlen(name)>=sizeof(addr.sun_path))fail("invalid broker socket");strcpy(addr.sun_path,name);if(connect(socket_fd,(sockaddr*)&addr,sizeof(addr)))fail("broker connection failed");}
 Packet p{op,a,b,c,d,e,f},r{};
 if(send(socket_fd,&p,sizeof(p),MSG_NOSIGNAL)!=sizeof(p)||recv(socket_fd,&r,sizeof(r),MSG_WAITALL)!=sizeof(r)){if(auto words=clock_words();words&&__atomic_load_n(words+2,__ATOMIC_ACQUIRE))_exit(0);fail("broker stopped or protocol failure");}
 internal=old;return r.a;
}
void* symbol(const char* lib,const char* name){
 bool old=internal;internal=true;void* f=dlsym(RTLD_NEXT,name);
 if(!f&&lib){std::string path=std::string("/opt/ros/jazzy/lib/")+lib;auto h=dlopen(path.c_str(),RTLD_NOW|RTLD_NOLOAD);if(h)f=dlsym(h,name);}
 internal=old;if(!f)fail(name);return f;
}
U hash(const void* p,size_t n,U h=1469598103934665603ULL){auto b=(const unsigned char*)p;for(size_t i=0;i<n;i++){h^=b[i];h*=1099511628211ULL;}return h;}
using Path=nav_msgs::msg::Path;using Pose=geometry_msgs::msg::PoseStamped;
U twist_key(const geometry_msgs::msg::Twist& m,const char* channel){U h=hash(channel,strlen(channel));h=hash(&m.linear,sizeof(m.linear),h);return hash(&m.angular,sizeof(m.angular),h);}
U path_key(const Path& p){U h=hash("path",4);h=hash(p.header.frame_id.data(),p.header.frame_id.size(),h);for(auto& x:p.poses){h=hash(&x.pose.position,sizeof(x.pose.position),h);h=hash(&x.pose.orientation,sizeof(x.pose.orientation),h);}return h;}
U message_key(const char* topic,const void* m){if(!topic||!m)return 0;auto end=strrchr(topic,'/');end=end?end+1:topic;
 if(!strcmp(end,"cmd_vel_nav")||!strcmp(end,"cmd_vel_smoothed")||!strcmp(end,"cmd_vel"))return twist_key(*(const geometry_msgs::msg::Twist*)m,end);
 if(!strcmp(end,"plan"))return path_key(*(const Path*)m);return 0;}
struct Input {U key,parent;};
std::mutex input_mutex;std::unordered_map<const void*,Input> inputs;
U input_parent(const void* ptr,U key){U parent=0;bool found=false;bool old=internal;internal=true;
 {std::lock_guard<std::mutex> unused(input_mutex);auto i=inputs.find(ptr);if(i!=inputs.end()&&i->second.key==key){parent=i->second.parent;found=true;}}
 internal=old;return found?parent:rpc(LOOKUP,key,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));}
struct Slot {std::atomic<uintptr_t> key{0};std::atomic<U> writer{0},trigger{0};};Slot slots[128];
Slot* slot(const void* ptr,bool create=false){uintptr_t k=(uintptr_t)ptr;if(!k)return nullptr;for(auto& s:slots)if(s.key.load()==k)return &s;if(create)for(auto& s:slots){uintptr_t zero=0;if(s.key.compare_exchange_strong(zero,k))return &s;}return nullptr;}
void dep(U parent){if(job&&parent&&parent!=job)rpc(DEP,job,parent);}
void hold_commit(pthread_mutex_t* m){for(auto p:held_commits)if(p==m)return;held_commits.push_back(m);}
bool held(pthread_mutex_t* m){for(auto p:held_commits)if(p==m)return true;return false;}
thread_local std::unordered_map<pthread_mutex_t*,unsigned> borrowed_holds;
std::mutex types_mutex;
std::unordered_map<const rcl_publisher_t*,const rosidl_message_type_support_t*> publisher_types;
std::unordered_map<const rcl_service_t*,const rosidl_message_type_support_t*> response_types;
struct Blob {
 rmw_serialized_message_t data=rmw_get_zero_initialized_serialized_message();
 Blob(const void* message,const rosidl_message_type_support_t* ts){
  auto alloc=rcutils_get_default_allocator();
  if(!ts||rmw_serialized_message_init(&data,0,&alloc)!=RMW_RET_OK||rmw_serialize(message,ts,&data)!=RMW_RET_OK)fail("cannot copy staged ROS output");
 }
 ~Blob(){if(rmw_serialized_message_fini(&data)!=RMW_RET_OK)fail("serialized output cleanup failed");}
};
struct ResponseCopy {
 const rosidl_typesupport_introspection_cpp::MessageMembers* members;void* message;
 ResponseCopy(const void* original,const rosidl_message_type_support_t* ts){
  auto introspection=ts?ts->func(ts,"rosidl_typesupport_introspection_cpp"):nullptr;
  if(!introspection)fail("missing service response introspection");
  members=static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers*>(introspection->data);
  message=::operator new(members->size_of_);members->init_function(message,rosidl_runtime_cpp::MessageInitialization::ALL);
  Blob bytes(original,ts);if(rmw_deserialize(&bytes.data,ts,message)!=RMW_RET_OK)fail("cannot clone service result");
 }
 ~ResponseCopy(){members->fini_function(message);::operator delete(message);}
};
thread_local pthread_mutex_t* planner_lock=nullptr;
struct Scope {
 U saved=job,id=0,start_cpu=0,start_excluded=0;int savedkind=kind;void* object;bool root=false;
 Scope(int task,void* obj,U parent=0):object(obj){
  if(armed()&&!job&&!internal){root=true;id=rpc(ENTER,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10),task,parent);job=id;kind=task;start_excluded=excluded_cpu_ns;start_cpu=cpu();}
 }
 ~Scope(){if(!root)return;
  U ended=cpu(),overhead=excluded_cpu_ns-start_excluded;
  U actual=ended-start_cpu>overhead?ended-start_cpu-overhead:0;
  // No computational result is visible until the complete host body is measured
  // and the selected virtual budget, dependencies and thermal guards permit it.
  rpc(STAGE,job,actual,1);
  flushing=true;
  auto outputs=std::move(pending_outputs);pending_outputs.clear();for(auto& output:outputs)output();
  flushing=false;
  if(kind==7)slot(object,true)->writer.store(job);
  extern int unlock_map(pthread_mutex_t*);
  for(auto m:held_commits){slot(m,true)->writer.store(job);unlock_map(m);}
  held_commits.clear();borrowed_holds.clear();
  if(planner_lock){slot(planner_lock,true)->writer.store(job);unlock_map(planner_lock);planner_lock=nullptr;}
  rpc(END,job,actual);job=saved;kind=savedkind;
  auto work=std::move(deferred);deferred.clear();for(auto& x:work)x.first(x.second);
 }
};
struct Helper {int tag;const void* obj;Helper* previous;pthread_mutex_t* first=nullptr;unsigned depth=0;};
thread_local Helper* helper=nullptr;
struct Help {Helper value;Help(int tag,const void* obj):value{tag,obj,helper}{helper=&value;}~Help(){helper=value.previous;}};
std::atomic<U> latest_command{0},latest_path{0};
}
extern "C" int libc_mutex_lock(pthread_mutex_t*);
extern "C" int libc_mutex_unlock(pthread_mutex_t*);
extern "C" int libc_mutex_trylock(pthread_mutex_t*);
asm(".symver libc_mutex_lock,__pthread_mutex_lock@GLIBC_2.2.5");
asm(".symver libc_mutex_unlock,__pthread_mutex_unlock@GLIBC_2.2.5");
asm(".symver libc_mutex_trylock,__pthread_mutex_trylock@GLIBC_2.2.5");
namespace live {int unlock_map(pthread_mutex_t* m){return libc_mutex_unlock(m);}}
extern "C" int pthread_mutex_lock(pthread_mutex_t* m) noexcept {
 if(live::internal||!live::job)return libc_mutex_lock(m);
 if(live::held(m)){live::borrowed_holds[m]++;return 0;}
 int result=libc_mutex_trylock(m);if(result==EBUSY){live::rpc(live::WAIT);result=libc_mutex_lock(m);live::rpc(live::RESUME);}
 if(!result&&live::helper){auto* h=live::helper;
  if(h->first==m){h->depth++;return result;}
  live::Slot* s=nullptr;
  if(!h->first){if(h->tag==1)s=live::slot(m,true);else if(h->tag==2)s=live::slot(m);else if(h->tag==3||h->tag==4||h->tag==5)s=live::slot(h->obj,true);}
  if(s&&!h->first){h->first=m;h->depth=1;if(h->tag==1||h->tag==2||h->tag==3)live::dep(s->writer.load());}
 }
 return result;
}
struct NoiseAccess:mppi::NoiseGenerator {
 static auto ready_member(){return &NoiseAccess::ready_;}
 static auto threaded_member(){return &NoiseAccess::regenerate_noises_;}
};
extern "C" int pthread_mutex_unlock(pthread_mutex_t* m) noexcept {
 if(live::job&&!live::internal&&live::held(m)){auto& n=live::borrowed_holds[m];if(!n)live::fail("unbalanced staged mutex");--n;return 0;}
 bool retain=false;
 if(live::job&&!live::internal)for(auto* h=live::helper;h;h=h->previous)if(h->first==m&&h->depth){
  if(!--h->depth){if(h->tag==1||h->tag==5){live::hold_commit(m);retain=true;}if(h->tag==4){auto noise=static_cast<mppi::NoiseGenerator*>(const_cast<void*>(h->obj));auto state=live::slot(h->obj,true);state->writer.store(0);state->trigger.store(0);
    if(live::kind==0&&(noise->*NoiseAccess::threaded_member())){noise->*NoiseAccess::ready_member()=false;state->trigger.store(live::job);static auto trigger=(void(*)(void*))live::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator18generateNextNoisesEv");live::defer_noise(trigger,noise);}}
  h->first=nullptr;}break;}
 if(retain)return 0;
 return libc_mutex_unlock(m);
}
extern "C" int pthread_cond_wait(pthread_cond_t* c,pthread_mutex_t* m){
 static auto fn=(int(*)(pthread_cond_t*,pthread_mutex_t*))live::symbol(nullptr,"pthread_cond_wait");bool tracked=live::job&&!live::internal;
 if(tracked)live::rpc(live::WAIT);int ret=fn(c,m);if(tracked)live::rpc(live::RESUME);return ret;
}
extern "C" int pthread_cond_timedwait(pthread_cond_t* c,pthread_mutex_t* m,const timespec* t){
 static auto fn=(int(*)(pthread_cond_t*,pthread_mutex_t*,const timespec*))live::symbol(nullptr,"pthread_cond_timedwait");bool tracked=live::job&&!live::internal;
 if(tracked)live::rpc(live::WAIT);int ret=fn(c,m,t);if(tracked)live::rpc(live::RESUME);return ret;
}
extern "C" int pthread_cond_clockwait(pthread_cond_t* c,pthread_mutex_t* m,clockid_t clock,const timespec* t){
 static auto fn=(int(*)(pthread_cond_t*,pthread_mutex_t*,clockid_t,const timespec*))live::symbol(nullptr,"pthread_cond_clockwait");bool tracked=live::job&&!live::internal;
 if(tracked)live::rpc(live::WAIT);int ret=fn(c,m,clock,t);if(tracked)live::rpc(live::RESUME);return ret;
}
extern "C" int nanosleep(const timespec* request,timespec* remainder){
 static auto fn=(int(*)(const timespec*,timespec*))live::symbol(nullptr,"nanosleep");
 if(live::job&&!live::internal&&live::armed()){live::rpc(live::SLEEP,live::now()+uint64_t(request->tv_sec)*1000000000+request->tv_nsec);if(remainder)*remainder={0,0};return 0;}return fn(request,remainder);
}
extern "C" int clock_nanosleep(clockid_t clock,int flags,const timespec* request,timespec* remainder){
 static auto fn=(int(*)(clockid_t,int,const timespec*,timespec*))live::symbol(nullptr,"clock_nanosleep");
 if(live::job&&!live::internal&&live::armed()){timespec t{};clock_gettime(clock,&t);long long duration=(long long)request->tv_sec*1000000000+request->tv_nsec;if(flags&TIMER_ABSTIME)duration-=(long long)t.tv_sec*1000000000+t.tv_nsec;if(duration>0)live::rpc(live::SLEEP,live::now()+duration);if(remainder)*remainder={0,0};return 0;}return fn(clock,flags,request,remainder);
}
#define VOID0(NAME,SYM,LIB,TASK) \
 extern "C" void NAME(void*) asm(SYM);extern "C" void NAME(void* self){static auto fn=(void(*)(void*))live::symbol(LIB,SYM);live::Scope scope(TASK,self);fn(self);}
VOID0(live_control,"_ZN15nav2_controller16ControllerServer25computeAndPublishVelocityEv","libcontroller_server_core.so",0)
struct PlanAccess:nav2_planner::PlannerServer {static auto server_member(){return &PlanAccess::action_server_poses_;}};
extern "C" void live_plan(void*) asm("_ZN12nav2_planner13PlannerServer23computePlanThroughPosesEv");
extern "C" void live_plan(void* self){static auto fn=(void(*)(void*))live::symbol("libplanner_server_core.so","_ZN12nav2_planner13PlannerServer23computePlanThroughPosesEv");live::U parent=0;
 if(live::armed()){auto& server=static_cast<nav2_planner::PlannerServer*>(self)->*PlanAccess::server_member();if(server){auto uuid=server->get_current_goal_id();auto key=live::hash(uuid.data(),uuid.size(),live::hash("goal",4));parent=live::rpc(live::LOOKUP,key,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));}}
 live::Scope s(5,self,parent);
 if(s.root){auto& server=static_cast<nav2_planner::PlannerServer*>(self)->*PlanAccess::server_member();auto uuid=server->get_current_goal_id();auto key=live::hash(uuid.data(),uuid.size(),live::hash("result-goal",11));live::rpc(live::RESULT_OWNER,s.id,key);}
 fn(self);}

extern "C" void live_costmap(void*) asm("_ZN15nav2_costmap_2d12Costmap2DROS9updateMapEv");
extern "C" void live_costmap(void* self){static auto fn=(void(*)(void*))live::symbol("libnav2_costmap_2d_core.so","_ZN15nav2_costmap_2d12Costmap2DROS9updateMapEv");auto n=((nav2_costmap_2d::Costmap2DROS*)self)->getName();live::Scope scope(n.find("local")!=std::string::npos?1:2,self);fn(self);}
extern "C" void live_smooth(void*) asm("_ZN22nav2_velocity_smoother16VelocitySmoother13smootherTimerEv");
extern "C" void live_smooth(void* self){static auto fn=(void(*)(void*))live::symbol("libvelocity_smoother_core.so","_ZN22nav2_velocity_smoother16VelocitySmoother13smootherTimerEv");live::Scope s(3,self,live::latest_command.load());fn(self);}
extern "C" void live_noise(void*) asm("_ZN4mppi14NoiseGenerator22generateNoisedControlsEv");
extern "C" void live_noise(void* self){static auto fn=(void(*)(void*))live::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator22generateNoisedControlsEv");live::Scope s(7,self,live::slot(self,true)->trigger.load());live::Help h(5,self);fn(self);}
extern "C" int live_bt(void*) asm("_ZN2BT4Tree8tickOnceEv");
extern "C" int live_bt(void* self){static auto fn=(int(*)(void*))live::symbol("libbehaviortree_cpp.so","_ZN2BT4Tree8tickOnceEv");live::Scope s(4,self);return fn(self);}
extern "C" void live_scan(void*,void*) asm("_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");
extern "C" void live_scan(void* self,void* p){static auto fn=(void(*)(void*,void*))live::symbol("libamcl_core.so","_ZN9nav2_amcl8AmclNode13laserReceivedESt10shared_ptrIKN11sensor_msgs3msg10LaserScan_ISaIvEEEE");live::Scope s(6,self);fn(self,p);}
#define COMMAND(NAME,SYM,TASK,CHANNEL,TYPE) \
 extern "C" void NAME(void*,void*) asm(SYM);extern "C" void NAME(void* self,void* arg){static auto fn=(void(*)(void*,void*))live::symbol(TASK==8?"libvelocity_smoother_core.so":"libcollision_monitor_core.so",SYM);auto p=((std::shared_ptr<TYPE>*)arg)->get();live::U parent=live::armed()?live::input_parent(p,live::twist_key(*p,CHANNEL)):0;live::Scope s(TASK,self,parent);fn(self,arg);if(TASK==8&&s.root)live::latest_command.store(s.id);}
COMMAND(live_command,"_ZN22nav2_velocity_smoother16VelocitySmoother20inputCommandCallbackESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE",8,"cmd_vel_nav",geometry_msgs::msg::Twist)
COMMAND(live_collision,"_ZN22nav2_collision_monitor16CollisionMonitor25cmdVelInCallbackUnstampedESt10shared_ptrIN13geometry_msgs3msg6Twist_ISaIvEEEE",9,"cmd_vel_smoothed",geometry_msgs::msg::Twist)
extern "C" void live_path(void*,const live::Path&) asm("_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");
extern "C" void live_path(void* self,const live::Path& path){static auto fn=(void(*)(void*,const live::Path&))live::symbol("libcontroller_server_core.so","_ZN15nav2_controller16ControllerServer14setPlannerPathERKN8nav_msgs3msg5Path_ISaIvEEE");auto parent=live::armed()?live::rpc(live::LOOKUP,live::path_key(path),strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10)):0;live::Scope s(10,self,parent);fn(self,path);if(s.root)live::latest_path.store(s.id);}
extern "C" void live_layered(void*,double,double,double) asm("_ZN15nav2_costmap_2d14LayeredCostmap9updateMapEddd");
extern "C" void live_layered(void* self,double x,double y,double yaw){static auto fn=(void(*)(void*,double,double,double))live::symbol("libnav2_costmap_2d_core.so","_ZN15nav2_costmap_2d14LayeredCostmap9updateMapEddd");live::Help h(1,self);fn(self,x,y,yaw);}
extern "C" void live_copy(void*,void*,void*) asm("_ZN4mppi14NoiseGenerator17setNoisedControlsERNS_6models5StateERKNS1_15ControlSequenceE");
extern "C" void live_copy(void* self,void* state,void* seq){static auto fn=(void(*)(void*,void*,void*))live::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator17setNoisedControlsERNS_6models5StateERKNS1_15ControlSequenceE");live::Help h(3,self);fn(self,state,seq);}
extern "C" void live_trigger(void*) asm("_ZN4mppi14NoiseGenerator18generateNextNoisesEv");
extern "C" void live_trigger(void* self){static auto fn=(void(*)(void*))live::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator18generateNextNoisesEv");if(live::job&&!live::internal){live::slot(self,true)->trigger.store(live::job);live::defer_noise(fn,self);}else fn(self);}
extern "C" geometry_msgs::msg::TwistStamped live_velocity(void*,const live::Pose&,const geometry_msgs::msg::Twist&,void*) asm("_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");
extern "C" geometry_msgs::msg::TwistStamped live_velocity(void* self,const live::Pose& p,const geometry_msgs::msg::Twist& v,void* g){static auto fn=(geometry_msgs::msg::TwistStamped(*)(void*,const live::Pose&,const geometry_msgs::msg::Twist&,void*))live::symbol("libmppi_controller.so","_ZN20nav2_mppi_controller14MPPIController23computeVelocityCommandsERKN13geometry_msgs3msg12PoseStamped_ISaIvEEERKNS2_6Twist_IS4_EEPN9nav2_core11GoalCheckerE");live::dep(live::latest_path.load());live::Help h(2,self);return fn(self,p,v,g);}
// Expose the already configured map pointer; no planner or map algorithm is reimplemented.
struct PlannerAccess:nav2_navfn_planner::NavfnPlanner {static auto map_member(){return &PlannerAccess::costmap_;}};
extern "C" live::Path live_navfn(void*,const live::Pose&,const live::Pose&,std::function<bool()>) asm("_ZN18nav2_navfn_planner12NavfnPlanner10createPlanERKN13geometry_msgs3msg12PoseStamped_ISaIvEEES7_St8functionIFbvEE");
extern "C" live::Path live_navfn(void* self,const live::Pose& a,const live::Pose& b,std::function<bool()> cancel){static auto fn=(live::Path(*)(void*,const live::Pose&,const live::Pose&,std::function<bool()>))live::symbol("libnav2_navfn_planner.so","_ZN18nav2_navfn_planner12NavfnPlanner10createPlanERKN13geometry_msgs3msg12PoseStamped_ISaIvEEES7_St8functionIFbvEE");
 if(live::job&&live::kind==5&&!live::planner_lock){auto m=(pthread_mutex_t*)(static_cast<nav2_navfn_planner::NavfnPlanner*>(self)->*PlannerAccess::map_member())->getMutex();pthread_mutex_lock(m);live::planner_lock=m;if(auto s=live::slot(m))live::dep(s->writer.load());}
 return fn(self,a,b,std::move(cancel));}
extern "C" rcl_ret_t rcl_publisher_init(rcl_publisher_t* publisher,const rcl_node_t* node,const rosidl_message_type_support_t* ts,const char* topic,const rcl_publisher_options_t* options){
 static auto fn=(decltype(&rcl_publisher_init))live::symbol("librcl.so","rcl_publisher_init");auto ret=fn(publisher,node,ts,topic,options);
 if(ret==RCL_RET_OK){bool old=live::internal;live::internal=true;{std::lock_guard<std::mutex> g(live::types_mutex);live::publisher_types[publisher]=ts;}live::internal=old;}return ret;
}
extern "C" rcl_ret_t rcl_service_init(rcl_service_t* service,const rcl_node_t* node,const rosidl_service_type_support_t* ts,const char* name,const rcl_service_options_t* options){
 static auto fn=(decltype(&rcl_service_init))live::symbol("librcl.so","rcl_service_init");auto ret=fn(service,node,ts,name,options);
 if(ret==RCL_RET_OK){bool old=live::internal;live::internal=true;{std::lock_guard<std::mutex> g(live::types_mutex);live::response_types[service]=ts->response_typesupport;}live::internal=old;}return ret;
}
extern "C" rcl_ret_t rcl_publish(const rcl_publisher_t* p,const void* m,rmw_publisher_allocation_t* allocation){
 static auto fn=(decltype(&rcl_publish))live::symbol("librcl.so","rcl_publish");static auto name=(decltype(&rcl_publisher_get_topic_name))live::symbol("librcl.so","rcl_publisher_get_topic_name");
 auto topic=name(p);bool final_command=topic&&!strcmp(strrchr(topic,'/')?strrchr(topic,'/')+1:topic,"cmd_vel");
 if(live::job&&!live::flushing){
  live::MeterPause pause;bool old=live::internal;live::internal=true;
  const rosidl_message_type_support_t* ts=nullptr;{std::lock_guard<std::mutex> g(live::types_mutex);auto i=live::publisher_types.find(p);if(i!=live::publisher_types.end())ts=i->second;}
  auto bytes=std::make_shared<live::Blob>(m,ts);auto key=live::message_key(topic,m);auto jid=live::job;
  live::pending_outputs.push_back([p,bytes,key,jid,final_command]{
   static auto publish=(decltype(&rcl_publish_serialized_message))live::symbol("librcl.so","rcl_publish_serialized_message");
   auto begin=live::epoch();if(key)live::rpc(live::PUBLISH_BEGIN,jid,key,begin);auto ret=publish(p,&bytes->data,nullptr);auto end=live::epoch();
   if(ret!=RCL_RET_OK)live::fail("buffered publisher failed");
   live::rpc(live::PUBLISH,jid,key,begin,end,final_command,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));
  });live::internal=old;return RCL_RET_OK;
 }
 if(final_command&&live::armed())live::rpc(live::DEVICE_GATE,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));
 auto begin=live::epoch();auto signature=live::message_key(topic,m);if(signature&&live::armed())live::rpc(live::PUBLISH_BEGIN,live::job,signature,begin,0,0,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));auto ret=fn(p,m,allocation);auto end=live::epoch();
 if(ret==RCL_RET_OK&&live::armed()&&(live::job||final_command)){auto key=live::message_key(topic,m);live::rpc(live::PUBLISH,live::job,key,begin,end,final_command,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));}return ret;
}
extern "C" rcl_ret_t rcl_send_request(const rcl_client_t* c,const void* m,int64_t* seq){
 static auto fn=(decltype(&rcl_send_request))live::symbol("librcl.so","rcl_send_request");return fn(c,m,seq);
}
extern "C" rcl_ret_t rcl_send_response(const rcl_service_t* service,rmw_request_id_t* id,void* m){
 static auto fn=(decltype(&rcl_send_response))live::symbol("librcl.so","rcl_send_response");
 if(live::job&&!live::flushing){
  live::MeterPause pause;bool old=live::internal;live::internal=true;
  const rosidl_message_type_support_t* ts=nullptr;{std::lock_guard<std::mutex> g(live::types_mutex);auto i=live::response_types.find(service);if(i!=live::response_types.end())ts=i->second;}
  auto copy=std::make_shared<live::ResponseCopy>(m,ts);auto request=*id;auto jid=live::job;
  live::pending_outputs.push_back([service,request,copy,jid]()mutable{if(fn(service,&request,copy->message)!=RCL_RET_OK)live::fail("buffered service response failed");live::rpc(live::PUBLISH,jid);});
  live::internal=old;return RCL_RET_OK;
 }
 return fn(service,id,m);
}
// WallRate cadence is translated from the actual Rate object's configured duration.
extern "C" bool live_rate(void*) asm("_ZN6rclcpp4Rate5sleepEv");
extern "C" bool live_rate(void* self){static auto fn=(bool(*)(void*))live::symbol("librclcpp.so","_ZN6rclcpp4Rate5sleepEv");if(!live::armed())return fn(self);thread_local std::unordered_map<void*,live::U> next;
 auto period=((rclcpp::Rate*)self)->period().count();auto n=live::now();auto& deadline=next[self];if(!deadline)deadline=n;deadline+=period;if(deadline<=n){deadline=n;return false;}live::rpc(live::SLEEP,deadline);return true;}
namespace live {
pthread_mutex_t timer_mutex=PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;
rcl_clock_t* timer_clock(){static rcl_clock_t* c=[](){auto p=new rcl_clock_t{};auto alloc=rcl_get_default_allocator();if(rcl_clock_init(RCL_ROS_TIME,p,&alloc)!=RCL_RET_OK||rcl_enable_ros_time_override(p)!=RCL_RET_OK)fail("timer clock initialization");
 std::thread([p]{U prev=~U(0);while(true){auto words=clock_words();if(words&&__atomic_load_n(words+2,__ATOMIC_ACQUIRE))return;auto n=now();if(n!=prev){libc_mutex_lock(&timer_mutex);auto ret=rcl_set_ros_time_override(p,n);libc_mutex_unlock(&timer_mutex);if(ret!=RCL_RET_OK)fail("timer clock update");prev=n;}timespec t{0,200000};nanosleep(&t,nullptr);}}).detach();return p;}();return c;}
}
extern "C" rcl_ret_t rcl_timer_init2(rcl_timer_t* timer,rcl_clock_t* clock,rcl_context_t* context,int64_t period,rcl_timer_callback_t callback,rcl_allocator_t allocator,bool autostart){static auto fn=(decltype(&rcl_timer_init2))live::symbol("librcl.so","rcl_timer_init2");bool smoothing=false;void* frames[16];int count=backtrace(frames,16);for(int i=0;i<count;i++){Dl_info info{};if(dladdr(frames[i],&info)&&info.dli_fname){if(strstr(info.dli_fname,"libbondcpp"))break;if(strstr(info.dli_fname,"libvelocity_smoother_core")){smoothing=true;break;}}}if(smoothing&&live::clock_words()&&clock&&clock->type==RCL_STEADY_TIME){clock=live::timer_clock();libc_mutex_lock(&live::timer_mutex);auto ret=fn(timer,clock,context,period,callback,allocator,autostart);libc_mutex_unlock(&live::timer_mutex);return ret;}return fn(timer,clock,context,period,callback,allocator,autostart);}

extern "C" rcl_ret_t rcl_take(const rcl_subscription_t* sub,void* message,rmw_message_info_t* info,rmw_subscription_allocation_t* allocation){
 static auto fn=(decltype(&rcl_take))live::symbol("librcl.so","rcl_take");static auto name=(decltype(&rcl_subscription_get_topic_name))live::symbol("librcl.so","rcl_subscription_get_topic_name");auto ret=fn(sub,message,info,allocation);
 if(ret==RCL_RET_OK&&live::armed()){auto key=live::message_key(name(sub),message);if(key){auto parent=live::rpc(live::LOOKUP,key,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10),info?info->source_timestamp:0);bool old=live::internal;live::internal=true;{std::lock_guard<std::mutex> guard(live::input_mutex);live::inputs[message]={key,parent};}live::internal=old;}}return ret;
}
extern "C" rcl_ret_t rcl_action_send_goal_request(const rcl_action_client_t* client,const void* request,int64_t* sequence){
 static auto fn=(decltype(&rcl_action_send_goal_request))live::symbol("librcl_action.so","rcl_action_send_goal_request");static auto name=(decltype(&rcl_action_client_get_action_name))live::symbol("librcl_action.so","rcl_action_client_get_action_name");
 if(live::job){auto action=name(client);if(action&&(strstr(action,"compute_path_through_poses")||strstr(action,"follow_path"))){auto& uuid=*(const unique_identifier_msgs::msg::UUID*)request;auto key=live::hash(uuid.uuid.data(),uuid.uuid.size(),live::hash("goal",4));live::rpc(live::TRIGGER,live::job,key);}}
 return fn(client,request,sequence);
}

extern "C" void live_reset(void*,void*,bool) asm("_ZN4mppi14NoiseGenerator5resetERNS_6models17OptimizerSettingsEb");
extern "C" void live_reset(void* self,void* settings,bool holonomic){static auto fn=(void(*)(void*,void*,bool))live::symbol("libmppi_controller.so","_ZN4mppi14NoiseGenerator5resetERNS_6models17OptimizerSettingsEb");live::Help h(4,self);fn(self,settings,holonomic);}

extern "C" void live_path_result(void*,const live::Path&) asm("_ZN12nav2_planner13PlannerServer11publishPlanERKN8nav_msgs3msg5Path_ISaIvEEE");
extern "C" void live_path_result(void* self,const live::Path& path){static auto fn=(void(*)(void*,const live::Path&))live::symbol("libplanner_server_core.so","_ZN12nav2_planner13PlannerServer11publishPlanERKN8nav_msgs3msg5Path_ISaIvEEE");
 if(live::job){live::MeterPause pause;auto key=live::path_key(path),jid=live::job;live::pending_outputs.push_back([key,jid]{live::rpc(live::PUBLISH,jid,key);});}fn(self,path);
}

__attribute__((constructor)) static void live_debug_signals(){if(getenv("NAV2_LIVE_DEBUG")){struct sigaction action{};action.sa_handler=[](int sig){const char* text="Live adapter native abort stack:\n";write(2,text,strlen(text));void* frames[48];int n=backtrace(frames,48);backtrace_symbols_fd(frames,n,2);_exit(128+sig);};sigemptyset(&action.sa_mask);sigaction(SIGABRT,&action,nullptr);}}

extern "C" rcl_ret_t rcl_timer_fini(rcl_timer_t* timer){static auto fn=(decltype(&rcl_timer_fini))live::symbol("librcl.so","rcl_timer_fini");libc_mutex_lock(&live::timer_mutex);auto ret=fn(timer);libc_mutex_unlock(&live::timer_mutex);return ret;}

// Result-cache replies may run on a different ROS executor thread from planning.
// Bind them to the exact accepted goal, then hold transmission for that job.
namespace live {
std::mutex result_mutex;std::unordered_map<U,U> result_requests;
U request_key(const rmw_request_id_t& id){return hash(&id.sequence_number,sizeof(id.sequence_number),hash(id.writer_guid,sizeof(id.writer_guid)));}
}
extern "C" rcl_ret_t rcl_action_take_result_request(const rcl_action_server_t* server,rmw_request_id_t* header,void* request){
 static auto fn=(decltype(&rcl_action_take_result_request))live::symbol("librcl_action.so","rcl_action_take_result_request");
 auto ret=fn(server,header,request);
 if(ret==RCL_RET_OK&&live::armed()){
  static auto name=(decltype(&rcl_action_server_get_action_name))live::symbol("librcl_action.so","rcl_action_server_get_action_name");
  auto action=name(server);if(action&&strstr(action,"compute_path_through_poses")){
   auto& uuid=*(const unique_identifier_msgs::msg::UUID*)request;auto key=live::hash(uuid.uuid.data(),uuid.uuid.size(),live::hash("result-goal",11));
   bool old=live::internal;live::internal=true;{std::lock_guard<std::mutex> lock(live::result_mutex);live::result_requests[live::request_key(*header)]=key;}live::internal=old;
  }
 }return ret;
}
extern "C" rcl_ret_t rcl_action_send_result_response(const rcl_action_server_t* server,rmw_request_id_t* header,void* response){
 static auto fn=(decltype(&rcl_action_send_result_response))live::symbol("librcl_action.so","rcl_action_send_result_response");
 live::U key=0;
 if(live::armed()){
  bool old=live::internal;live::internal=true;{std::lock_guard<std::mutex> lock(live::result_mutex);auto i=live::result_requests.find(live::request_key(*header));if(i!=live::result_requests.end())key=i->second;}live::internal=old;
 }
 if(key&&!live::job){
  auto owner=live::rpc(live::LOOKUP,key,strtoull(getenv("NAV2_LIVE_DEVICE"),nullptr,10));
  if(!owner)live::fail("planner result has no actual producing job");
  live::rpc(live::RESULT_GATE,owner);auto ret=fn(server,header,response);
  if(ret!=RCL_RET_OK)live::fail("gated action result failed");live::rpc(live::PUBLISH,owner);return ret;
 }
 return fn(server,header,response);
}
