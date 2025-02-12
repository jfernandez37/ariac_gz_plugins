#include "ariac_gz_plugins/vacuum_gripper_plugin.hpp"

using namespace gz;
using namespace sim;
using namespace systems;

namespace ariac_sensors{
    void VacuumGripperPlugin::Configure(const Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        EntityComponentManager &_ecm,
                                        EventManager &)
    {
        std::vector<std::string> arguments = {"--ros-args"};
        arguments.push_back(RCL_PARAM_FILE_FLAG);
        arguments.push_back(ament_index_cpp::get_package_share_directory("aprs_description")+"/config/robot_controllers.yaml");
        std::vector<const char *> argv;
        for (const auto & arg : arguments) {
            argv.push_back(reinterpret_cast<const char *>(arg.data()));
        }
        
        if (!rclcpp::ok(vacuum_gripper_context)){
            rclcpp::init(static_cast<int>(argv.size()), argv.data());
        }
        
        ros_node_ = rclcpp::Node::make_shared("vacuum_gripper_plugin_node");

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(ros_node_);

        auto spin = [this](){
            while(rclcpp::ok(vacuum_gripper_context)){
            executor_->spin_once();
            }
        };

        thread_executor_spin_ = std::thread(spin);

        gripper_link_name = _sdf->Get<std::string>("gripper_link");

        // gz_contact_topic = _sdf->Get<std::string>("gz_contact_topic");

        auto model = Model(_entity);
        
        gripper_entity = model.LinkByName(_ecm, gripper_link_name);

        gz_node_ = std::make_shared<gz::transport::Node>();
        // gz_node_->Subscribe(gz_contact_topic, &VacuumGripperPlugin::NewContactMessageRecieved, this);

        toggle_joint_srv = ros_node_->create_service<std_srvs::srv::Trigger>(
            "/toggle_joint", 
            std::bind(
              &VacuumGripperPlugin::toggle_joint_cb, this,
              std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(ros_node_->get_logger(), "Finished configure");
    }

    void VacuumGripperPlugin::toggle_joint_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if(is_attatched){
            detatch_requested = true;
        } else {
            attatch_requested = true;
        }
    }

    void VacuumGripperPlugin::PreUpdate(const gz::sim::UpdateInfo &,
        gz::sim::EntityComponentManager &_ecm)
    {
        auto child_entity = _ecm.EntityByName("box_2_link");

        if (!child_entity.has_value()){
            return;
        }

        if (attatch_requested && !is_attatched){
            detatchable_joint_entity = _ecm.CreateEntity();
            
            _ecm.CreateComponent(
                detatchable_joint_entity,
                components::DetachableJoint({gripper_entity, child_entity.value(), "fixed"})
            );
            
            attatch_requested = false;
            is_attatched = true;

            RCLCPP_INFO(ros_node_->get_logger(), "Joint attatched.");
        }
        else if (detatch_requested && is_attatched)
        {
            _ecm.RequestRemoveEntity(detatchable_joint_entity);

            detatchable_joint_entity = kNullEntity;

            detatch_requested = false;
            is_attatched = false;

            RCLCPP_INFO(ros_node_->get_logger(), "Removed joint");
        }
        

    }

    void VacuumGripperPlugin::NewContactMessageRecieved(const gz::msgs::Contacts &_gz_contacts_msg){
        std::string model_in_contact;
        for (int i = 0; i < _gz_contacts_msg.contact_size(); ++i){
            RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Contact 1: " << _gz_contacts_msg.contact(i).collision1().name());
            RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Contact 2: " << _gz_contacts_msg.contact(i).collision2().name());
        }
        // auto data = _gz_contacts_msg.contact().data();
        // RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Number of contacts: " << );
    }
}

GZ_ADD_PLUGIN(
    ariac_sensors::VacuumGripperPlugin,
    gz::sim::System,
    ariac_sensors::VacuumGripperPlugin::ISystemConfigure,
    ariac_sensors::VacuumGripperPlugin::ISystemPreUpdate
  )

