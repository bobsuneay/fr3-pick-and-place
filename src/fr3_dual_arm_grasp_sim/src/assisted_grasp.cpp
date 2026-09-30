#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo_ros/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include "contact_evidence.hpp"

namespace fr3_dual_arm_grasp_sim {
class AssistedGrasp : public gazebo::WorldPlugin {
  using SetBool = std_srvs::srv::SetBool;
  struct Pending {
    std::string side, message;
    double deadline=0.0;
    bool close=false, done=false, success=false, expired=false;
  };
  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  gazebo::event::ConnectionPtr update_, contact_update_;
  std::array<ContactEvidence, 2> contacts_;
  gazebo::physics::JointPtr grasp_;
  std::array<rclcpp::Service<SetBool>::SharedPtr, 2> services_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_;
  std::mutex mutex_; std::condition_variable cv_; std::shared_ptr<Pending> pending_;
  std::string robot_, object_, owner_;
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_=world; world_->Physics()->GetContactManager()->SetNeverDropContacts(true);
    node_=gazebo_ros::Node::Get(sdf); robot_=sdf->Get<std::string>("robot_model"); object_=sdf->Get<std::string>("object_model");
    for (size_t i=0;i<2;++i) {
      const std::string side=i==0?"left":"right";
      services_[i]=node_->create_service<SetBool>(side+"_grasp",[this,side](
          const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res){
        std::unique_lock<std::mutex> lock(mutex_);
        if(pending_){res->message="Another operation is pending";return;}
        auto p=std::make_shared<Pending>();p->side=side;p->close=req->data;
        p->deadline=world_->SimTime().Double()+2.0;pending_=p;
        if(!cv_.wait_for(lock,std::chrono::seconds(2),[&p]{return p->done;})){
          p->expired=true;pending_.reset();res->message="Simulation did not advance; request expired";return;}
        res->success=p->success;res->message=p->message;
      });
    }
    status_=node_->create_service<std_srvs::srv::Trigger>("owner",[this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res){
      std::lock_guard<std::mutex> lock(mutex_);res->success=true;res->message=owner_;
    });
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(
      [this](const gazebo::common::UpdateInfo &){Update();});
    contact_update_=gazebo::event::Events::ConnectWorldUpdateEnd([this](){SampleContacts();});
    RCLCPP_WARN(node_->get_logger(),"ASSISTED GAZEBO GRASP: bilateral contact + fixed joint; not friction validation");
  }
 private:
  void SampleContacts(){
    std::lock_guard<std::mutex> lock(mutex_);double now=world_->SimTime().Double();
    auto robot=world_->ModelByName(robot_),object=world_->ModelByName(object_);if(!robot||!object)return;
    auto body=object->GetLink("body");std::array<std::array<double,2>,2> depths{{{{-1,-1}},{{-1,-1}}}};
    auto manager=world_->Physics()->GetContactManager();
    for(unsigned int i=0;i<manager->GetContactCount();++i){
      auto c=manager->GetContact(i);if(!c||!c->collision1||!c->collision2||!c->count)continue;
      auto a=c->collision1->GetLink(),b=c->collision2->GetLink();auto other=a==body?b:(b==body?a:gazebo::physics::LinkPtr());if(!other)continue;
      double depth=0;for(unsigned int k=0;k<c->count;++k)depth=std::max(depth,c->depths[k]);
      for(size_t s=0;s<2;++s)for(size_t f=0;f<2;++f){
        std::string name=std::string(s==0?"left":"right")+(f==0?"_left_finger":"_right_finger");
        if(other==robot->GetLink(name))depths[s][f]=std::max(depths[s][f],depth);
      }
    }
    for(size_t s=0;s<2;++s)contacts_[s].sample(now,depths[s]);
  }
  bool FingerFeedbackReady(const std::string &side,gazebo::physics::ModelPtr robot){
    auto l=robot->GetJoint(side+"_left_finger_joint"),r=robot->GetJoint(side+"_right_finger_joint");if(!l||!r)return false;
    double a=l->Position(0),b=r->Position(0);return std::isfinite(a)&&std::isfinite(b)&&a>=0&&b>=0&&std::abs(a-b)<=.001;
  }
  void Update(){
    std::lock_guard<std::mutex> lock(mutex_);if(!pending_||pending_->expired)return;auto p=pending_;
    try{
      auto robot=world_->ModelByName(robot_),object=world_->ModelByName(object_);if(!robot||!object)throw std::runtime_error("Robot/workpiece model missing");
      auto palm=robot->GetLink(p->side+"_gripper_palm"),body=object->GetLink("body");if(!palm||!body)throw std::runtime_error("Required physical link missing");
      if(p->close){size_t i=p->side=="left"?0:1;double now=world_->SimTime().Double();
        // Contact samples arrive at WorldUpdateEnd, while this callback runs
        // at WorldUpdateBegin.  Keep the request pending until the bilateral
        // evidence is stable instead of failing on the first transient frame.
        if(!contacts_[i].ready(now)){
          if(now < p->deadline)return;
          throw std::runtime_error("Require >=100 ms sustained contact on BOTH fingers");
        }
        if(!FingerFeedbackReady(p->side,robot)){
          if(now < p->deadline)return;
          throw std::runtime_error("Invalid or unsynchronized physical finger feedback");
        }
        if(owner_!=p->side){auto next=world_->Physics()->CreateJoint("fixed",robot);if(!next)throw std::runtime_error("Cannot create assisted grasp joint");
          next->SetName("fr3_assisted_grasp_"+p->side);next->Attach(palm,body);next->Load(palm,body,ignition::math::Pose3d::Zero);next->SetModel(object);next->Init();
          if(grasp_){grasp_->Detach();grasp_->Fini();}grasp_=next;owner_=p->side;}
      }else if(owner_==p->side){if(grasp_){grasp_->Detach();grasp_->Fini();grasp_.reset();}owner_.clear();}
      p->success=true;p->message=owner_;
    }catch(const std::exception &e){p->message=e.what();}
    p->done=true;pending_.reset();cv_.notify_all();
  }
};
GZ_REGISTER_WORLD_PLUGIN(AssistedGrasp)
}
