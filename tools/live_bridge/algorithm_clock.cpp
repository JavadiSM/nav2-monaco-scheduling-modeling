// Use the physics clock for the installed path-replanning throttle only.
// DDS timestamps, host timeouts and actual thread CPU measurements retain host clocks.
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
namespace live {extern thread_local bool internal;void* symbol(const char*,const char*);}
namespace {thread_local bool rate_tick=false;}
namespace {
uint64_t* words() {
 static auto p=[]() -> uint64_t* {
  const char* path=getenv("NAV2_LIVE_CLOCK");if(!path)return nullptr;
  int fd=open(path,O_RDONLY|O_CLOEXEC);if(fd<0)return nullptr;
  void* view=mmap(nullptr,32,PROT_READ,MAP_SHARED,fd,0);close(fd);
  return view==MAP_FAILED?nullptr:static_cast<uint64_t*>(view);
 }();return p;
}
bool algorithm_caller(void* address) {
 Dl_info info{};if(!dladdr(address,&info)||!info.dli_fname)return false;
 const char* base=strrchr(info.dli_fname,'/');base=base?base+1:info.dli_fname;
 return !strcmp(base,"libnav2_rate_controller_bt_node.so");
}
}
namespace std { namespace chrono {
system_clock::time_point system_clock::now() noexcept {
 auto p=words();
 bool semantic=false;
 if(rate_tick&&!getenv("NAV2_LIVE_HOST_REPLAN_DIAGNOSTIC")&&p&&__atomic_load_n(p+1,__ATOMIC_ACQUIRE)) {
  bool previous=live::internal;live::internal=true;
  semantic=algorithm_caller(__builtin_return_address(0));
  live::internal=previous;
 }
 if(semantic) {
  auto ns=__atomic_load_n(p,__ATOMIC_ACQUIRE);
  return time_point(duration_cast<duration>(nanoseconds(ns)));
 }
 using Function=time_point(*)() noexcept;
 static auto original=reinterpret_cast<Function>(dlsym(RTLD_NEXT,"_ZNSt6chrono3_V212system_clock3nowEv"));
 if(!original)abort();return original();
}
}}

extern "C" int live_rate_tick(void*) asm("_ZN18nav2_behavior_tree14RateController4tickEv");
extern "C" int live_rate_tick(void* self){
 static auto original=(int(*)(void*))live::symbol("libnav2_rate_controller_bt_node.so","_ZN18nav2_behavior_tree14RateController4tickEv");
 struct Context {bool previous=rate_tick;Context(){rate_tick=true;}~Context(){rate_tick=previous;}} context;
 return original(self);
}
