// Test-only subscriber: one compact evidence line per complete Airy frame.
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
void OnScan(ConstLaserScanStampedPtr &msg) {
  const auto &s = msg->scan();
  double minimum = std::numeric_limits<double>::infinity();
  int finite = 0;
  for (int i = 0; i < s.ranges_size(); ++i) {
    if (std::isfinite(s.ranges(i))) { ++finite; minimum = std::min(minimum, s.ranges(i)); }
  }
  std::cout << msg->time().sec() + msg->time().nsec()*1e-9 << ","
            << s.count() << "," << s.vertical_count() << ","
            << s.ranges_size() << "," << finite << "," << minimum << std::endl;
}
int main(int argc, char **argv) {
  gazebo::client::setup(argc, argv);
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init("default");
  auto sub = node->Subscribe("~/rematrice/base_link/airy/scan", OnScan);
  while (true) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
