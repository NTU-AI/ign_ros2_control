#include <rclcpp/rclcpp.hpp>
#include "ign_ros2_control/ign_system.hpp"
#include "Robot.cpp"

namespace ign_ros2_control {
  //////////////////////////////////////////////////
  class IgnitionROS2ControlPluginPrivate{
    public:
        /// \brief Get the URDF XML from the parameter server
        //std::string getURDF(std::string) const;

        /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
        /// joint names are specified in the plugin configuration, all valid 1-axis
        /// joints are returned
        /// \param[in] _entity Entity of the model that the plugin is being
        /// configured for
        /// \param[in] _ecm Ignition Entity Component Manager
        /// \return List of entities containing all enabled joints
        //std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(const ignition::gazebo::Entity & _entity, ignition::gazebo::EntityComponentManager & _ecm) const;

        /// \brief Entity ID for sensor within Gazebo.
        //ignition::gazebo::Entity entity_;

        /// \brief Node Handles
        rclcpp::Node::SharedPtr ign_ros2_control_node{nullptr};

        /// \brief Thread where the executor will spin
        std::thread thread_executor_spin_;

        /// \brief Flag to stop the executor thread when this plugin is exiting
        bool stop_{false};

        /// \brief Executor to spin the controller
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

        /// \brief Timing
        rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

        /// \brief Interface loader
        std::shared_ptr<pluginlib::ClassLoader<ign_ros2_control::IgnitionSystemInterface>> robot_hw_sim_loader_{nullptr};

        /// \brief Last time the update method was called
        rclcpp::Time last_update_sim_time_ros_ = rclcpp::Time((int64_t)0, RCL_ROS_TIME);

        /// \brief ECM pointer
        ignition::gazebo::EntityComponentManager * ecm{nullptr};

        /// \brief controller update rate
        int update_rate;

        /// \brief Vector of all System Interfaces
        std::vector<ign_ros2_control::IgnitionSystemInterface*> ignSystemInstances;

        /// \brief Vector of all Resource Managers
        std::vector<hardware_interface::ResourceManager*> resource_managers_;

        /// \brief Flag to setup robot_hw_sim_loader_
        bool uniqueLoop = true;

        /// \brief Robot param
        std::vector<Robot*> robots;
  };
}