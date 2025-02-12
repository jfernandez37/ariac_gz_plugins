#ifndef VACUUM_GRIPPER_PLUGIN_HPP
#define VACUUM_GRIPPER_PLUGIN_HPP

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/contacts.pb.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace gz;
using namespace sim;
using namespace systems;

namespace ariac_sensors{
    class VacuumGripperPlugin
        : public System,
          public ISystemConfigure,
          public ISystemPreUpdate
    {
        public:
            void Configure (const gz::sim::Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                gz::sim::EntityComponentManager &_ecm,
                gz::sim::EventManager &_event_mgr) override;
            
            void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) final;

            std::shared_ptr<gz::transport::Node> gz_node_;
            
            rclcpp::Node::SharedPtr ros_node_;
            rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
            std::thread thread_executor_spin_;
            
            std::string gz_contact_topic;
            std::string gripper_link_name;
            
            gz::sim::Entity detatchable_joint_entity;
            gz::sim::Entity gripper_entity;

            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_joint_srv;

            rclcpp::Context::SharedPtr vacuum_gripper_context;

            bool is_attatched = false;
            bool attatch_requested = false;
            bool detatch_requested = false;

            void toggle_joint_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response);

            void NewContactMessageRecieved(const gz::msgs::Contacts &_gz_contacts_msg);
    };
}

#endif