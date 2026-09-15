// Test-only external wrench fixture. Never installed in the production model.
// Commands are in world axes, N and Nm; held until explicitly replaced.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <mutex>

namespace gazebo {
class RematriceDisturbanceFixture : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr) override {
    link_ = model->GetLink("base_link");
    node_.reset(new transport::Node());
    node_->Init(model->GetWorld()->Name());
    sub_ = node_->Subscribe("~/rematrice_test/wrench", &RematriceDisturbanceFixture::Command, this);
    pub_ = node_->Advertise<msgs::Wrench>("~/rematrice_test/applied_wrench", 10);
    update_ = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo &) {
      std::lock_guard<std::mutex> lock(mutex_);
      link_->AddForce(force_);
      link_->AddTorque(torque_);
      if (++ticks_ % 25 == 0) {
        msgs::Wrench actual;
        msgs::Set(actual.mutable_force(), force_);
        msgs::Set(actual.mutable_torque(), torque_);
        pub_->Publish(actual);
      }
    });
  }
 private:
  void Command(ConstWrenchPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    force_ = msgs::ConvertIgn(msg->force());
    torque_ = msgs::ConvertIgn(msg->torque());
  }
  physics::LinkPtr link_;
  transport::NodePtr node_;
  transport::SubscriberPtr sub_;
  transport::PublisherPtr pub_;
  event::ConnectionPtr update_;
  ignition::math::Vector3d force_{0,0,0}, torque_{0,0,0};
  std::mutex mutex_;
  unsigned ticks_{0};
};
GZ_REGISTER_MODEL_PLUGIN(RematriceDisturbanceFixture)
}
