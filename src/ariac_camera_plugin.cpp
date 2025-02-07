#include <rclcpp/rclcpp.hpp>
#include <gz/sim/Sensor.hh>

#include <gz/transport/Node.hh>

#include <ros_gz_bridge/convert.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <ariac_msgs/msg/sensors.hpp>

#include <gz/plugin/Register.hh>

#include "ariac_gz_plugins/ariac_camera_plugin.hpp"

namespace ariac_sensors{

  class AriacCameraPluginPrivate {
    public:
      void OnNewImageFrame(const gz::msgs::Image & _gz_msg);
      void OnNewDepthFrame(const gz::msgs::Image & _gz_msg);

      void SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg);
    
      std::string camera_type_;
      std::shared_ptr<gz::transport::Node> gz_node_;
      rclcpp::Node::SharedPtr ros_node_;
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
      std::thread thread_executor_spin_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub_;
      rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
      sensor_msgs::msg::CameraInfo camera_info_msg_;

      std::string gz_topic_;
      std::string gz_topic_depth_;
      std::string cam_info_gz_topic_;

      bool publish_sensor_data_;
      sensor_msgs::msg::Image ros_msg;
      rclcpp::Subscription<ariac_msgs::msg::Sensors>::SharedPtr sensor_health_sub_;

      void FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg);
  };

  AriacCameraPlugin::AriacCameraPlugin() : impl_(std::make_unique<AriacCameraPluginPrivate>()) {}

  void AriacCameraPlugin::Configure (const gz::sim::Entity &_entity,
                      const std::shared_ptr<const sdf::Element> &_sdf,
                      gz::sim::EntityComponentManager &_ecm,
                      gz::sim::EventManager &_event_mgr) 
  {
    // Set up ros publisher
        std::vector<std::string> arguments = {"--ros-args"};
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(ament_index_cpp::get_package_share_directory("aprs_description")+"/config/robot_controllers.yaml");
    std::vector<const char *> argv;
    for (const auto & arg : arguments) {
      argv.push_back(reinterpret_cast<const char *>(arg.data()));
    }
    
    if (!rclcpp::ok()){
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    impl_->ros_node_ = rclcpp::Node::make_shared(_sdf->Get<std::string>("camera_name")+"_node");
    impl_->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    impl_->executor_->add_node(impl_->ros_node_);
    auto spin = [this]()
      {
        while (rclcpp::ok()) {
          impl_->executor_->spin_once();
        }
      };
    impl_->thread_executor_spin_ = std::thread(spin);

    impl_->publish_sensor_data_ = true;
    impl_->image_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Image>(
      _sdf->Get<std::string>("rgb_img_ros_topic"), 10);

    impl_->camera_type_ = _sdf->Get<std::string>("camera_type");
    if (impl_->camera_type_ == "rgb") {
      impl_->gz_topic_ = _sdf->Get<std::string>("gz_topic");
      impl_->cam_info_gz_topic_ = _sdf->Get<std::string>("cam_info_gz_topic");
    } else if (impl_->camera_type_ == "rgbd") {
      impl_->gz_topic_ = _sdf->Get<std::string>("gz_topic") + "/image";
      impl_->gz_topic_depth_ = _sdf->Get<std::string>("gz_topic") + "/depth_image";
      impl_->cam_info_gz_topic_ = _sdf->Get<std::string>("gz_topic") + "/camera_info";
      impl_->depth_image_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::Image>(
        _sdf->Get<std::string>("depth_img_ros_topic"), 10);
    }

    // // Set up gz subscriber
    impl_->gz_node_ = std::make_shared<gz::transport::Node>();
    impl_->gz_node_->Subscribe(impl_->gz_topic_, &AriacCameraPluginPrivate::OnNewImageFrame, impl_.get());
    impl_->gz_node_->Subscribe(impl_->cam_info_gz_topic_, &AriacCameraPluginPrivate::FillCameraInfoMsg, impl_.get());
    if (impl_->camera_type_ == "rgbd") {
      impl_->gz_node_->Subscribe(impl_->gz_topic_depth_, &AriacCameraPluginPrivate::OnNewDepthFrame, impl_.get());
    }

    // Subscribe to sensor health topic
    impl_->sensor_health_sub_ = impl_->ros_node_->create_subscription<ariac_msgs::msg::Sensors>("/ariac/sensor_health", 
      10, std::bind(&AriacCameraPluginPrivate::SensorHealthCallback, impl_.get(), std::placeholders::_1));
    
    impl_->camera_info_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      _sdf->Get<std::string>("cam_info_ros_topic"), 10);
  }


  void AriacCameraPluginPrivate::SensorHealthCallback(const ariac_msgs::msg::Sensors::SharedPtr msg) {
    publish_sensor_data_ = msg->camera;
  }


  void AriacCameraPluginPrivate::OnNewImageFrame(const gz::msgs::Image & _gz_msg)
  {
    if (publish_sensor_data_) {
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      image_pub_->publish(ros_msg);

      camera_info_msg_.header.stamp.sec = _gz_msg.header().stamp().sec();
      camera_info_msg_.header.stamp.nanosec = _gz_msg.header().stamp().nsec();      
      camera_info_pub_->publish(camera_info_msg_);
    }
  }

  void AriacCameraPluginPrivate::OnNewDepthFrame(const gz::msgs::Image & _gz_msg) {
    if (publish_sensor_data_) {
      ros_gz_bridge::convert_gz_to_ros(_gz_msg, ros_msg);
      depth_image_pub_->publish(ros_msg);
    }
  }

  void AriacCameraPluginPrivate::FillCameraInfoMsg(const gz::msgs::CameraInfo &_info_msg) {
    // Assuming a particular camera's CameraInfo (except timestamp) doesn't change once it is spawned, 
    // fill the ROS msg only once at startup (but publish it along with every new image frame).
    
    camera_info_msg_.header.frame_id = _info_msg.header().data(0).value(0);
    // camera_info_msg_.header's time is updated while publishing

    camera_info_msg_.height = _info_msg.height();
    camera_info_msg_.width = _info_msg.width();

    if (_info_msg.distortion().model() == 0) {
      camera_info_msg_.distortion_model = "plumb_bob";
    } else if (_info_msg.distortion().model() == 1) {
      camera_info_msg_.distortion_model = "rational_polynomial";
    } else if (_info_msg.distortion().model() == 2) {
      camera_info_msg_.distortion_model = "equidistant";
    }
    camera_info_msg_.d.resize(5);
    camera_info_msg_.d[0] = _info_msg.distortion().k(0);
    camera_info_msg_.d[1] = _info_msg.distortion().k(1);
    camera_info_msg_.d[2] = _info_msg.distortion().k(2);
    camera_info_msg_.d[3] = _info_msg.distortion().k(3);
    camera_info_msg_.d[4] = _info_msg.distortion().k(4);

    camera_info_msg_.k[0] = _info_msg.intrinsics().k(0);
    camera_info_msg_.k[1] = _info_msg.intrinsics().k(1);
    camera_info_msg_.k[2] = _info_msg.intrinsics().k(2);
    camera_info_msg_.k[3] = _info_msg.intrinsics().k(3);
    camera_info_msg_.k[4] = _info_msg.intrinsics().k(4);
    camera_info_msg_.k[5] = _info_msg.intrinsics().k(5);
    camera_info_msg_.k[6] = _info_msg.intrinsics().k(6);
    camera_info_msg_.k[7] = _info_msg.intrinsics().k(7);
    camera_info_msg_.k[8] = _info_msg.intrinsics().k(8);

    camera_info_msg_.r[0] = _info_msg.rectification_matrix(0);
    camera_info_msg_.r[1] = _info_msg.rectification_matrix(1);
    camera_info_msg_.r[2] = _info_msg.rectification_matrix(2);
    camera_info_msg_.r[3] = _info_msg.rectification_matrix(3);
    camera_info_msg_.r[4] = _info_msg.rectification_matrix(4);
    camera_info_msg_.r[5] = _info_msg.rectification_matrix(5);
    camera_info_msg_.r[6] = _info_msg.rectification_matrix(6);
    camera_info_msg_.r[7] = _info_msg.rectification_matrix(7);
    camera_info_msg_.r[8] = _info_msg.rectification_matrix(8);

    camera_info_msg_.p[0] = _info_msg.projection().p(0);
    camera_info_msg_.p[1] = _info_msg.projection().p(1);
    camera_info_msg_.p[2] = _info_msg.projection().p(2);
    camera_info_msg_.p[3] = _info_msg.projection().p(3);
    camera_info_msg_.p[4] = _info_msg.projection().p(4);
    camera_info_msg_.p[5] = _info_msg.projection().p(5);
    camera_info_msg_.p[6] = _info_msg.projection().p(6);
    camera_info_msg_.p[7] = _info_msg.projection().p(7);
    camera_info_msg_.p[8] = _info_msg.projection().p(8);
    camera_info_msg_.p[9] = _info_msg.projection().p(9);
    camera_info_msg_.p[10] = _info_msg.projection().p(10);
    camera_info_msg_.p[11] = _info_msg.projection().p(11);

    // After CameraIfo message got filled once, unsubscribe the callback to prevent future callbacks
    // As an attempt to make the plugin lighter.
    gz_node_->Unsubscribe(cam_info_gz_topic_);
  }
}

GZ_ADD_PLUGIN(
  ariac_sensors::AriacCameraPlugin,
  gz::sim::System,
  ariac_sensors::AriacCameraPlugin::ISystemConfigure
)
