#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <iostream>
int main(){
 auto file=getenv("NAV2_LIVE_CLOCK");int fd=open(file,O_RDWR);auto clock=(uint64_t*)mmap(nullptr,32,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
 clock[0]=6000000000ULL;clock[1]=1;
 BT::BehaviorTreeFactory factory;factory.registerFromPlugin("/opt/ros/jazzy/lib/libnav2_rate_controller_bt_node.so");int count=0;
 factory.registerSimpleAction("Counter",[&](BT::TreeNode&){++count;return BT::NodeStatus::SUCCESS;});
 auto tree=factory.createTreeFromText("<root BTCPP_format=\"4\"><BehaviorTree ID=\"Main\"><RateController hz=\"1.0\"><Counter/></RateController></BehaviorTree></root>");
 tree.rootNode()->executeTick();int first=count;auto host=std::chrono::system_clock::now();std::this_thread::sleep_for(std::chrono::milliseconds(1150));
 tree.rootNode()->executeTick();int paused=count;
 clock[0]+=999000000ULL;tree.rootNode()->executeTick();int before=count;
 clock[0]+=1000000ULL;tree.rootNode()->executeTick();int after=count;
 bool ok=first==1&&paused==1&&before==1&&after==2&&std::chrono::system_clock::now()-host>=std::chrono::seconds(1);
 std::cout<<"{\"first\":"<<first<<",\"host_pause\":"<<paused<<",\"sim_999ms\":"<<before<<",\"sim_1000ms\":"<<after<<",\"passed\":"<<(ok?"true":"false")<<"}\n";return ok?0:1;
}
