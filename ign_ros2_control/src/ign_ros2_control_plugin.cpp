// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/plugin/Register.hh>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "ign_ros2_control/ign_ros2_control_plugin.hpp"
#include "ign_ros2_control/ign_system.hpp"

namespace ign_ros2_control {
  //////////////////////////////////////////////////
  class IgnitionROS2ControlPluginPrivate{
    public:
      /// \brief Get the URDF XML from the parameter server
      std::string getURDF(std::string) const;

      /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
      /// joint names are specified in the plugin configuration, all valid 1-axis
      /// joints are returned
      /// \param[in] _entity Entity of the model that the plugin is being
      /// configured for
      /// \param[in] _ecm Ignition Entity Component Manager
      /// \return List of entities containing all enabled joints
      std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(const ignition::gazebo::Entity & _entity, ignition::gazebo::EntityComponentManager & _ecm) const;

      /// \brief Entity ID for sensor within Gazebo.
      ignition::gazebo::Entity entity_;

      /// \brief Node Handles
      std::shared_ptr<rclcpp::Node> node_{nullptr};

      /// \brief Thread where the executor will spin
      std::thread thread_executor_spin_;

      /// \brief Flag to stop the executor thread when this plugin is exiting
      bool stop_{false};

      /// \brief Executor to spin the controller
      rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

      /// \brief Timing
      rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

      /// \brief Interface loader
      std::shared_ptr<pluginlib::ClassLoader<
          ign_ros2_control::IgnitionSystemInterface>>
      robot_hw_sim_loader_{nullptr};

      /// \brief Controller manager
      std::shared_ptr<controller_manager::ControllerManager>
      controller_manager_{nullptr};

      /// \brief String with the robot description param_name
      // TODO(ahcorde): Add param in plugin tag
      std::string robot_description_ = "robot_description";

      /// \brief String with the name of the node that contains the robot_description
      // TODO(ahcorde): Add param in plugin tag
      //std::string robot_description_node_ = "robot_state_publisher";

      /// \brief Last time the update method was called
      rclcpp::Time last_update_sim_time_ros_ =
        rclcpp::Time((int64_t)0, RCL_ROS_TIME);

      /// \brief ECM pointer
      ignition::gazebo::EntityComponentManager * ecm{nullptr};

      /// \brief controller update rate
      int update_rate;
  };

  //////////////////////////////////////////////////
  std::map<std::string, ignition::gazebo::Entity>
  IgnitionROS2ControlPluginPrivate::GetEnabledJoints(const ignition::gazebo::Entity & _entity,
                                                     ignition::gazebo::EntityComponentManager & _ecm) const
  {
    std::map<std::string, ignition::gazebo::Entity> output;

    std::vector<std::string> enabledJoints;

    // Get all available joints
    auto jointEntities = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Joint());

    // Iterate over all joints and verify whether they can be enabled or not
    for (const auto & jointEntity : jointEntities) {
      const auto jointName = _ecm.Component<ignition::gazebo::components::Name>(
        jointEntity)->Data();

      // Make sure the joint type is supported, i.e. it has exactly one
      // actuated axis
      const auto * jointType = _ecm.Component<ignition::gazebo::components::JointType>(jointEntity);
      switch (jointType->Data()) {
        case sdf::JointType::PRISMATIC:
        case sdf::JointType::REVOLUTE:
        case sdf::JointType::CONTINUOUS:
        case sdf::JointType::GEARBOX:{
          // Supported joint type
          break;
        }
        case sdf::JointType::FIXED:{
          RCLCPP_INFO(node_->get_logger(), "[ign_ros2_control] Fixed joint [%s] (Entity=%d)] is skipped", jointName.c_str(), jointEntity);
          continue;
        }
        case sdf::JointType::REVOLUTE2:
        case sdf::JointType::SCREW:
        case sdf::JointType::BALL:
        case sdf::JointType::UNIVERSAL:{
          RCLCPP_WARN(node_->get_logger(), "[ign_ros2_control] Joint [%s] (Entity=%d)] is of unsupported type.\n Only joints with a single axis are supported.",jointName.c_str(), jointEntity);
          continue;
        }
        default:{
          RCLCPP_WARN(node_->get_logger(),"[ign_ros2_control] Joint [%s] (Entity=%d)] is of unknown type",jointName.c_str(), jointEntity);
          continue;
        }
      }
      output[jointName] = jointEntity;
    }

    return output;
  }

  //////////////////////////////////////////////////
  std::string IgnitionROS2ControlPluginPrivate::getURDF(std::string robot_description_node_) const
  {
    std::string urdf_string;

    //robot_description_node_= rsp_node_name;

    using namespace std::chrono_literals;

    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_description_node_);

    while (!parameters_client->wait_for_service(0.5s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for %s service. Exiting.", robot_description_node_.c_str());
        return 0;
      }
      RCLCPP_ERROR( node_->get_logger(), "%s service not available, waiting again...", robot_description_node_.c_str());
    }

    RCLCPP_INFO( node_->get_logger(), "connected to service!! %s asking for %s", robot_description_node_.c_str(), this->robot_description_.c_str());

    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
      RCLCPP_DEBUG( node_->get_logger(), "param_name %s", this->robot_description_.c_str());

      try {
        auto f = parameters_client->get_parameters({this->robot_description_});
        f.wait();
        std::vector<rclcpp::Parameter> values = f.get();
        urdf_string = values[0].as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
      }

      if (!urdf_string.empty()) {
        break;
      } 
      else {
        RCLCPP_ERROR( node_->get_logger(), "ign_ros2_control plugin is waiting for model URDF in parameter [%s] on the ROS param server.", this->robot_description_.c_str());
      }
      usleep(100000);
    }

    RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

    return urdf_string;
  }

  //////////////////////////////////////////////////
  IgnitionROS2ControlPlugin::IgnitionROS2ControlPlugin() : dataPtr(std::make_unique<IgnitionROS2ControlPluginPrivate>()){}

  //////////////////////////////////////////////////
  IgnitionROS2ControlPlugin::~IgnitionROS2ControlPlugin()
  {
    // Stop controller manager thread
    this->dataPtr->stop_ = true;
    this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);
    this->dataPtr->executor_->cancel();
    this->dataPtr->thread_executor_spin_.join();
  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::Configure(const ignition::gazebo::Entity & _entity,
                                            const std::shared_ptr<const sdf::Element> & _sdf,
                                            ignition::gazebo::EntityComponentManager & _ecm,
                                            ignition::gazebo::EventManager &)
  {

    // Make sure the controller is attached to a valid model
    const auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR( this->dataPtr->node_->get_logger(),
                    "[Ignition ROS 2 Control] Failed to initialize because [%s] (Entity=%u)] is not a model."
                    "Please make sure that Ignition ROS 2 Control is attached to a valid model.",
                    model.Name(_ecm).c_str(), _entity
      );
      return;
    }

    // Get params from SDF
    std::string paramFileName = _sdf->Get<std::string>("parameters");

    if (paramFileName.empty()) {
      RCLCPP_ERROR(this->dataPtr->node_->get_logger(),"Ignition ros2 control found an empty parameters file. Failed to initialize.");
      return;
    }

    // Get robot_state_publisher from SDF
    std::string ns = _sdf->Get<std::string>("robot_namespace");
    std::string rsp_nodename{"robot_state_publisher"};

    if (ns.empty()) 
      ns = "/";
  

    std::cout << "Selected Namespace: " + ns << std::endl;

    // Get robot_name from SDF
    std::string robot_name = _sdf->Get<std::string>("robot_name");

    if (robot_name.empty()) {
      robot_name = ""; 
    }

    // Get controller manager node name
    std::string controllerManagerNodeName{"controller_manager"};

    // Creating arguments to run the ignition_ros2_control
    std::vector<std::string> arguments = {}; // {"--ros-args","--remap", "__ns:="+ns};
    std::vector<const char *> argv;
    for (const auto & arg : arguments) {
      argv.push_back(reinterpret_cast<const char *>(arg.data()));
    }

    // Creating node to publish logs
    std::string node_name = "ignition_ros2_control";

    if (!rclcpp::ok()) {
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns);

    this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->dataPtr->executor_->add_node(this->dataPtr->node_);

    this->dataPtr->stop_ = false;
    auto spin = [this]()
    {
      while (rclcpp::ok() && !this->dataPtr->stop_) {
        this->dataPtr->executor_->spin_once();
      }
    };

    this->dataPtr->thread_executor_spin_ = std::thread(spin);

    RCLCPP_DEBUG_STREAM(this->dataPtr->node_->get_logger(), "[Ignition ROS 2 Control] Setting up controller for [" << model.Name(_ecm) << "] (Entity=" << _entity << ")].");

    // Get list of enabled joints
    auto enabledJoints = this->dataPtr->GetEnabledJoints(_entity, _ecm);

    if (enabledJoints.size() == 0) {
      RCLCPP_DEBUG_STREAM( this->dataPtr->node_->get_logger(), "[Ignition ROS 2 Control] There are no available Joints.");
      return;
    }

    // Read urdf from ros parameter server then
    // setup actuators and mechanism control node.
    // This call will block if ROS is not properly initialized.
    std::string urdf_string;
    std::vector<hardware_interface::HardwareInfo> control_hardware;

    try {
      urdf_string = this->dataPtr->getURDF(rsp_nodename);
      control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    } 
    catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_STREAM(this->dataPtr->node_->get_logger(), "Error parsing URDF in ign_ros2_control plugin, plugin not active : " << ex.what());
      return;
    }

    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ = std::make_unique<hardware_interface::ResourceManager>();

    try {
      this->dataPtr->robot_hw_sim_loader_.reset(
        new pluginlib::ClassLoader<ign_ros2_control::IgnitionSystemInterface>("ign_ros2_control","ign_ros2_control::IgnitionSystemInterface")
      );
    } 
    catch (pluginlib::LibraryLoadException & ex) {
      RCLCPP_ERROR(this->dataPtr->node_->get_logger(), "Failed to create robot simulation interface loader: %s ", ex.what());
      return;
    }

    for (unsigned int i = 0; i < control_hardware.size(); ++i) {
      std::string robot_hw_sim_type_str_ = control_hardware[i].hardware_class_type;
      auto ignitionSystem = std::unique_ptr<ign_ros2_control::IgnitionSystemInterface>( this->dataPtr->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));

      if (!ignitionSystem->initSim(this->dataPtr->node_, enabledJoints, control_hardware[i], _ecm, this->dataPtr->update_rate))
      {
        RCLCPP_FATAL(this->dataPtr->node_->get_logger(), "Could not initialize robot simulation interface");
        return;
      }

      resource_manager_->import_component(std::move(ignitionSystem));
    }

    // Create the controller manager
    RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");

    auto *cm = new controller_manager::ControllerManager(std::move(resource_manager_), this->dataPtr->executor_, controllerManagerNodeName, ns);

    this->dataPtr->controller_manager_.reset(cm);

    YAML::Node config = YAML::LoadFile(paramFileName);

    const YAML::Node &node = config["controller_manager"]["ros__parameters"];
    for(const auto& it : node ){
      if(it.first.as<std::string>().compare("update_rate") == 0)
        this->dataPtr->controller_manager_->declare_parameter(it.first.as<std::string>() , it.second.as<int>());
      else{
        std::string prefix;

        if(!robot_name.empty())
          prefix = robot_name + "_";

        this->dataPtr->controller_manager_->declare_parameter(prefix + it.first.as<std::string>()+".type" , it.second["type"].as<std::string>() );
      }
    }

    //this->dataPtr->controller_manager_->declare_parameter("prefix", "/"+robot_name);

    this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

    if (!this->dataPtr->controller_manager_->has_parameter("update_rate")) {
      RCLCPP_ERROR_STREAM(this->dataPtr->node_->get_logger(), "controller manager doesn't have an update_rate parameter");
      return;
    }

    this->dataPtr->update_rate = this->dataPtr->controller_manager_->get_parameter("update_rate").as_int();

    this->dataPtr->control_period_ = rclcpp::Duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / static_cast<double>(this->dataPtr->update_rate)
        )
      )
    );

    this->dataPtr->entity_ = _entity;

  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & _info, ignition::gazebo::EntityComponentManager & /*_ecm*/)
  {
    static bool warned{false};

    if (!warned) {
      rclcpp::Duration gazebo_period(_info.dt);

      //Check the period against the simulation period
      if (this->dataPtr->control_period_ < _info.dt) {
        RCLCPP_ERROR_STREAM( this->dataPtr->node_->get_logger(), "Desired controller update period (" << 
                                                                  this->dataPtr->control_period_.seconds() << 
                                                                  " s) is faster than the gazebo simulation period (" <<
                                                                  gazebo_period.seconds() << " s)."
        );
      } 
      else if (this->dataPtr->control_period_ > gazebo_period) {
        RCLCPP_WARN_STREAM( this->dataPtr->node_->get_logger(), " Desired controller update period (" << 
                                                                this->dataPtr->control_period_.seconds() <<
                                                                " s) is slower than the gazebo simulation period (" <<
                                                                gazebo_period.seconds() << " s)."
        );
      }

      warned = true;

    }

    // Always set commands on joints, otherwise at low control frequencies the joints tremble
    // as they are updated at a fraction of gazebo sim time
    this->dataPtr->controller_manager_->write();
  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::PostUpdate(const ignition::gazebo::UpdateInfo & _info, const ignition::gazebo::EntityComponentManager & /*_ecm*/) 
  {
    // Get the simulation time and period
    rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count(), RCL_ROS_TIME);

    rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

    if (sim_period >= this->dataPtr->control_period_) {
      this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
      auto ign_controller_manager = std::dynamic_pointer_cast<ign_ros2_control::IgnitionSystemInterface>(this->dataPtr->controller_manager_);
      this->dataPtr->controller_manager_->read();
      this->dataPtr->controller_manager_->update();
    }
  }
}  // namespace ign_ros2_control

IGNITION_ADD_PLUGIN(
  ign_ros2_control::IgnitionROS2ControlPlugin,
  ignition::gazebo::System,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemConfigure,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemPreUpdate,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemPostUpdate
)
