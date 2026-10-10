// Optional observation of TF receipt and lookup timing; never supplies transforms.
#include <tf2/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <dlfcn.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cstdio>
#include <string>
namespace live {uint64_t now();void* symbol(const char*,const char*);}
namespace {
int audit_fd(){static int fd=[](){auto path=getenv("NAV2_LIVE_TF_AUDIT");return path?open(path,O_WRONLY|O_APPEND|O_CREAT,0600):-1;}();return fd;}
void record(const char* kind,const void* buffer,const std::string& source,const std::string& target,int64_t stamp,int64_t other,int ok){
 int fd=audit_fd();if(fd>=0)dprintf(fd,"%s,%llu,%p,%s,%s,%lld,%lld,%d\n",kind,(unsigned long long)live::now(),buffer,source.c_str(),target.c_str(),(long long)stamp,(long long)other,ok);
}
}
using Transform=geometry_msgs::msg::TransformStamped;
extern "C" bool audit_set(void*,const Transform&,const std::string&,bool) asm("_ZN3tf210BufferCore12setTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb");
extern "C" bool audit_set(void* self,const Transform& msg,const std::string& authority,bool is_static){
 static auto fn=(bool(*)(void*,const Transform&,const std::string&,bool))live::symbol("libtf2.so","_ZN3tf210BufferCore12setTransformERKN13geometry_msgs3msg17TransformStamped_ISaIvEEERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEEb");
 bool ok=fn(self,msg,authority,is_static);if(audit_fd()>=0)record("set",self,msg.header.frame_id,msg.child_frame_id,int64_t(msg.header.stamp.sec)*1000000000+msg.header.stamp.nanosec,0,ok);return ok;
}
extern "C" Transform audit_lookup(const void*,const std::string&,const tf2::TimePoint&,const std::string&,const tf2::TimePoint&,const std::string&,const tf2::Duration) asm("_ZNK7tf2_ros6Buffer15lookupTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEEEEES8_SJ_S8_SG_");
extern "C" Transform audit_lookup(const void* self,const std::string& target,const tf2::TimePoint& target_time,const std::string& source,const tf2::TimePoint& source_time,const std::string& fixed,const tf2::Duration timeout){
 static auto fn=(Transform(*)(const void*,const std::string&,const tf2::TimePoint&,const std::string&,const tf2::TimePoint&,const std::string&,tf2::Duration))live::symbol("libtf2_ros.so","_ZNK7tf2_ros6Buffer15lookupTransformERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNSt6chrono10time_pointINS9_3_V212system_clockENS9_8durationIlSt5ratioILl1ELl1000000000EEEEEES8_SJ_S8_SG_");
 if(audit_fd()>=0)record("lookup_begin",self,source,target,target_time.time_since_epoch().count(),source_time.time_since_epoch().count(),0);
 try{auto value=fn(self,target,target_time,source,source_time,fixed,timeout);if(audit_fd()>=0)record("lookup_end",self,source,target,target_time.time_since_epoch().count(),source_time.time_since_epoch().count(),1);return value;}
 catch(...){if(audit_fd()>=0)record("lookup_end",self,source,target,target_time.time_since_epoch().count(),source_time.time_since_epoch().count(),0);throw;}
}
