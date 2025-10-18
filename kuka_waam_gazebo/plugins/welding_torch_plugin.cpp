#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class WeldingTorchPlugin : public ModelPlugin
  {
    public:
      WeldingTorchPlugin() : ModelPlugin()
      {
      }

      void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
      {
        // Store model pointer
        this->model = _model;
        this->world = _model->GetWorld();
        
        // Initialize ROS node
        this->ros_node_ = gazebo_ros::Node::Get(_sdf);
        
        // Get parameters from SDF
        if (_sdf->HasElement("arc_topic"))
        {
          this->arc_topic = _sdf->Get<std::string>("arc_topic");
        }
        else
        {
          this->arc_topic = "/welding_state";
        }
        
        if (_sdf->HasElement("tcp_link"))
        {
          std::string tcp_link_name = _sdf->Get<std::string>("tcp_link");
          this->tcp_link = this->model->GetLink(tcp_link_name);
          if (!this->tcp_link)
          {
            RCLCPP_ERROR(this->ros_node_->get_logger(), 
                        "TCP link [%s] not found!", tcp_link_name.c_str());
            return;
          }
        }
        
        if (_sdf->HasElement("heat_radius"))
        {
          this->heat_radius = _sdf->Get<double>("heat_radius");
        }
        else
        {
          this->heat_radius = 0.01; // 10mm
        }
        
        if (_sdf->HasElement("heat_intensity"))
        {
          this->heat_intensity = _sdf->Get<double>("heat_intensity");
        }
        else
        {
          this->heat_intensity = 1000.0;
        }
        
        // Subscribe to welding state
        this->arc_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
          this->arc_topic,
          10,
          std::bind(&WeldingTorchPlugin::OnArcMsg, this, std::placeholders::_1)
        );
        
        // Publisher for heat point (visualization)
        this->heat_pub_ = this->ros_node_->create_publisher<geometry_msgs::msg::Point>(
          "/welding_heat_point", 10
        );
        
        // Connect to world update event
        this->update_connection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WeldingTorchPlugin::OnUpdate, this)
        );
        
        RCLCPP_INFO(this->ros_node_->get_logger(), 
                   "Welding Torch Plugin loaded successfully");
        
        this->welding_active = false;
      }

      void OnArcMsg(const std_msgs::msg::Bool::SharedPtr msg)
      {
        this->welding_active = msg->data;
        
        if (this->welding_active)
        {
          RCLCPP_INFO(this->ros_node_->get_logger(), "Welding arc ON");
        }
        else
        {
          RCLCPP_INFO(this->ros_node_->get_logger(), "Welding arc OFF");
        }
      }

      void OnUpdate()
      {
        if (!this->tcp_link || !this->welding_active)
        {
          return;
        }
        
        // Get TCP position in world frame
        ignition::math::Pose3d tcp_pose = this->tcp_link->WorldPose();
        ignition::math::Vector3d tcp_position = tcp_pose.Pos();
        
        // Publish heat point for visualization
        geometry_msgs::msg::Point heat_msg;
        heat_msg.x = tcp_position.X();
        heat_msg.y = tcp_position.Y();
        heat_msg.z = tcp_position.Z();
        this->heat_pub_->publish(heat_msg);
        
        // Apply heat to nearby objects (simplified - in real impl would affect material properties)
        // This is a placeholder for future thermal simulation
        
        // Visual effect: Create light at weld point
        this->CreateWeldLight(tcp_position);
      }
      
      void CreateWeldLight(const ignition::math::Vector3d& position)
      {
        
        // This is visual only - actual welding physics would be more complex

      }

    private:
      // Gazebo pointers
      physics::ModelPtr model;
      physics::WorldPtr world;
      physics::LinkPtr tcp_link;
      event::ConnectionPtr update_connection;
      
      // ROS interface
      gazebo_ros::Node::SharedPtr ros_node_;
      rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arc_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr heat_pub_;
      
      // Parameters
      std::string arc_topic;
      double heat_radius;
      double heat_intensity;
      bool welding_active;
  };

  GZ_REGISTER_MODEL_PLUGIN(WeldingTorchPlugin)
}