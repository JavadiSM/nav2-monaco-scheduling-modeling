// Persistent Gazebo transport: one acknowledged physics iteration per request.
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <sstream>
#include <array>
#include <cstring>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <chrono>
int main() {
  gz::transport::Node node; std::mutex mutex; std::condition_variable cv;
  long long now=0; // A freshly launched paused world starts at zero; ignore a delayed initial clock.
  node.Subscribe<gz::msgs::Clock>("/world/monaco/clock",[&](const gz::msgs::Clock& c){
    std::lock_guard<std::mutex> guard(mutex);now=c.sim().sec()*1000000000LL+c.sim().nsec();cv.notify_all();
  });
  std::array<uint64_t,4> commands{};
  auto hash=[](const void* p,size_t n,uint64_t h){auto b=(const unsigned char*)p;for(size_t i=0;i<n;i++){h^=b[i];h*=1099511628211ULL;}return h;};
  std::array<std::string,4> topics{"/cmd_vel","/blue/cmd_vel","/white/cmd_vel","/green/cmd_vel"};
  for(size_t i=0;i<4;i++)node.Subscribe<gz::msgs::Twist>(topics[i],[&,i](const gz::msgs::Twist& m){double v[6]{m.linear().x(),m.linear().y(),m.linear().z(),m.angular().x(),m.angular().y(),m.angular().z()};auto h=hash(v,sizeof(v),hash("cmd_vel",7,1469598103934665603ULL));std::lock_guard<std::mutex> guard(mutex);commands[i]=h;cv.notify_all();});
  std::string command;
  while(std::getline(std::cin,command)) {
    if(command=="quit")break;
    std::istringstream line(command);std::string verb;std::array<uint64_t,4> expected{};line>>verb;for(auto& e:expected)line>>e;
    {std::unique_lock<std::mutex> lock(mutex);if(!cv.wait_for(lock,std::chrono::seconds(3),[&]{for(size_t i=0;i<4;i++)if(expected[i]&&expected[i]!=commands[i])return false;return true;})){std::cerr<<"Modeled command was not acknowledged on Gazebo transport before the next step\n";return 4;}}
    long long before;{std::lock_guard<std::mutex> guard(mutex);before=now;}
    gz::msgs::WorldControl request;request.set_pause(true);request.set_multi_step(1);
    gz::msgs::Boolean response;bool result=false;
    if(!node.Request("/world/monaco/control",request,3000,response,result)||!result||!response.data()) {
      std::cerr<<"World control did not acknowledge one step\n";return 2;
    }
    std::unique_lock<std::mutex> lock(mutex);
    if(!cv.wait_for(lock,std::chrono::seconds(3),[&]{return now>before;})) {
      std::cerr<<"No increasing simulation clock after step\n";return 3;
    }
    std::cout<<now<<std::endl;
  }
}
