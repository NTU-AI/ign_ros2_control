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
  IgnitionROS2ControlPlugin::IgnitionROS2ControlPlugin(){
    std::cout << "Ignition ROS 2 Control Plugin"<< std::endl;
  }

  //////////////////////////////////////////////////
  IgnitionROS2ControlPlugin::~IgnitionROS2ControlPlugin()
  {
    // Stop controller manager thread
    this->ignRos2ControlPtr->stop_ = true;

    if(this->ignRos2ControlPtr->robots.size()!=0){
      for(auto * robot: this->ignRos2ControlPtr->robots){
        this->ignRos2ControlPtr->executor_->remove_node(robot->controller_manager_);
      }
    }
    //this->ignRos2ControlPtr->executor_->remove_node(this->ignRos2ControlPtr->controller_manager_);
    this->ignRos2ControlPtr->executor_->cancel();
    this->ignRos2ControlPtr->thread_executor_spin_.join();
  }

  void IgnitionROS2ControlPlugin::initialize(){
    
    this->ignRos2ControlPtr = std::make_unique<IgnitionROS2ControlPluginPrivate>();

    // Creating node to publish logs
    std::string node_name = "ignition_ros2_control";

    if (!rclcpp::ok()) {
      // Creating arguments to run the ignition_ros2_control
      std::vector<std::string> arguments = {}; // {"--ros-args","--remap", "__ns:="+ns};
      std::vector<const char *> argv;
      for (const auto & arg : arguments) {
        argv.push_back(reinterpret_cast<const char *>(arg.data()));
      }

      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }

    this->ignRos2ControlPtr->ign_ros2_control_node = rclcpp::Node::make_shared(node_name);

    this->ignRos2ControlPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    this->ignRos2ControlPtr->executor_->add_node(this->ignRos2ControlPtr->ign_ros2_control_node);

    this->ignRos2ControlPtr->stop_ = false;
    auto spin = [this]()
    {
      while (rclcpp::ok() && !this->ignRos2ControlPtr->stop_) {
        this->ignRos2ControlPtr->executor_->spin_once();
      }
    };

    this->ignRos2ControlPtr->thread_executor_spin_ = std::thread(spin);
  }

  void IgnitionROS2ControlPlugin::importRobotDescription(const ignition::gazebo::Entity & _entity,const std::shared_ptr<const sdf::Element> & _sdf, ignition::gazebo::EntityComponentManager & _ecm){

    /// Application configs

    // Make sure the controller is attached to a valid model
    const auto model = ignition::gazebo::Model(_entity);
    if (!model.Valid(_ecm)) {
      RCLCPP_ERROR( this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(),
                    "[Ignition ROS 2 Control] Failed to initialize because [%s] (Entity=%u)] is not a model."
                    "Please make sure that Ignition ROS 2 Control is attached to a valid model.",
                    model.Name(_ecm).c_str(), _entity
      );
      return;
    }

    // Get robot_name from SDF
    std::string robot_name = _sdf->Get<std::string>("robot_name");

    if (robot_name.empty()) {
      RCLCPP_ERROR(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(),"Ignition ros2 control found an robot_name empty. Please, specify a valid name.");
      return;
    }

     // Get robot_type from SDF
    std::string robot_type = _sdf->Get<std::string>("robot_type");

    if (robot_type.empty()) {
      RCLCPP_ERROR(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(),"Ignition ros2 control found an robot_type empty. Please, specify a valid type.");
      return;
    }

    // Get params from SDF
    std::string paramFileName = _sdf->Get<std::string>("parameters");

    if (paramFileName.empty()) {
      RCLCPP_ERROR(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(),"Ignition ros2 control found an empty parameters file. Failed to initialize.");
      return;
    }

    // Creating Robot entity
    Robot *robot = new Robot(robot_type, robot_name, paramFileName, this->ignRos2ControlPtr->ign_ros2_control_node);

    this->ignRos2ControlPtr->robots.push_back(robot);

    RCLCPP_DEBUG_STREAM(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "[Ignition ROS 2 Control] Setting up controller for [" << model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  }

  void IgnitionROS2ControlPlugin::configHardwarePlugin(std::string urdf_string, std::map<std::string, ignition::gazebo::Entity> enabledJoints, Robot *currentRobot, ignition::gazebo::EntityComponentManager & _ecm){

    // Read urdf from ros parameter server then
    // setup actuators and mechanism control node.
    // This call will block if ROS is not properly initialized.
    std::vector<hardware_interface::HardwareInfo> control_hardware;

    try {
      //urdf_string = this->ignRos2ControlPtr->getURDF(rsp_nodename);
      control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    } 
    catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_STREAM(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "Error parsing URDF in ign_ros2_control plugin, plugin not active : " << ex.what());
      return;
    }

    if(this->ignRos2ControlPtr->uniqueLoop){
      try {
        this->ignRos2ControlPtr->robot_hw_sim_loader_.reset(
          new pluginlib::ClassLoader<ign_ros2_control::IgnitionSystemInterface>("ign_ros2_control","ign_ros2_control::IgnitionSystemInterface")
        );
      } 
      catch (pluginlib::LibraryLoadException & ex) {
        RCLCPP_ERROR(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "Failed to create robot simulation interface loader: %s ", ex.what());
        return;
      }
    }

    auto resource_manager_ = new hardware_interface::ResourceManager();
    std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ptr = std::unique_ptr<hardware_interface::ResourceManager>(resource_manager_);

    for (unsigned int i = 0; i < control_hardware.size(); ++i) {

      std::string robot_hw_sim_type_str_ = control_hardware[i].hardware_class_type;
      auto ignSystemInstance = this->ignRos2ControlPtr->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_);
      auto ignitionSystem = std::unique_ptr<ign_ros2_control::IgnitionSystemInterface>( ignSystemInstance );

      ignitionSystem->configSim(this->ignRos2ControlPtr->ign_ros2_control_node, _ecm, this->ignRos2ControlPtr->update_rate);

      bool initializeInterface = ignitionSystem->initSim(enabledJoints, control_hardware[i] );

      if (!initializeInterface)
      {
        RCLCPP_FATAL(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "Could not initialize robot simulation interface");
        return;
      }
      
      //resource_manager_ptr->stop_components();
      resource_manager_ptr->import_component(std::move(ignitionSystem));
      //resource_manager_ptr->start_components();

      this->ignRos2ControlPtr->ignSystemInstances.push_back(ignSystemInstance);
    }

    this->ignRos2ControlPtr->resource_managers_.push_back(resource_manager_);

    this->ignRos2ControlPtr->uniqueLoop = false;

    

    // Create the controller manager
    RCLCPP_INFO(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "Loading controller_manager");

    std::string controllerManagerNodeName = currentRobot->getCM_Nodename();
    std::string ns = currentRobot->getNamespace();

    auto *cm = new controller_manager::ControllerManager(std::move(resource_manager_ptr), this->ignRos2ControlPtr->executor_, controllerManagerNodeName, ns);

    currentRobot->controller_manager_.reset(cm);

    this->ignRos2ControlPtr->executor_->add_node(currentRobot->controller_manager_);
  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::Configure(const ignition::gazebo::Entity & _entity,
                                            const std::shared_ptr<const sdf::Element> & _sdf,
                                            ignition::gazebo::EntityComponentManager & _ecm,
                                            ignition::gazebo::EventManager &)
  {

    // Initializing ROS NODE
    this->initialize();

    // Importing robot
    this->importRobotDescription(_entity, _sdf, _ecm);
    
    //Application Configs
    auto currentRobot = this->ignRos2ControlPtr->robots.back();

    std::string urdf_string = currentRobot->getURDF();

    // Get list of enabled joints
    auto enabledJoints = currentRobot->GetEnabledJoints(_entity, _ecm);

    if (enabledJoints.size() == 0) {
      RCLCPP_DEBUG_STREAM( this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "[Ignition ROS 2 Control] There are no available Joints.");
      return;
    }

    std::string paramFileName = currentRobot->getParamFilePath();
    std::string robot_name = currentRobot->getName();

    this->configHardwarePlugin(urdf_string, enabledJoints, currentRobot, _ecm);

    YAML::Node config = YAML::LoadFile(paramFileName);

    const YAML::Node &node = config["controller_manager"]["ros__parameters"];
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    for(const auto& it : node ){

      //std::cout << "CONTROLLER ENTREI: " + it.first.as<std::string>() << std::endl;

      if(it.first.as<std::string>().compare("update_rate") == 0)
        currentRobot->controller_manager_->declare_parameter(it.first.as<std::string>() , it.second.as<int>());
      else{
        std::string prefix;

        if(!robot_name.empty())
          prefix = robot_name + "_";

        std::string controller_name = prefix + it.first.as<std::string>();
        std::string controller_type = it.second["type"].as<std::string>();

        currentRobot->controller_manager_->declare_parameter(controller_name+".type" , controller_type);

        //std::cout << "CONTROLLER: " + prefix + it.first.as<std::string>()+".type" << std::endl;
        //std::cout << it.second["type"].as<std::string>() << std::endl;
        currentRobot->controller_manager_->load_controller(controller_name , controller_type);

        std::string ntu_gazebo_share_directory = ament_index_cpp::get_package_share_directory("ntu_gazebo");

        std::string cmd = "ros2 param load " + controller_name + " "+ ntu_gazebo_share_directory + it.second["filepath"].as<std::string>();
        std::system(cmd.c_str());

        currentRobot->controller_manager_->configure_controller(controller_name);
        
        start_controllers.push_back(controller_name);
      }
    }

    //std::cout << "STARTING " << start_controllers.size() << " CONTROLLERS" << std::endl;
    std::string cmd = "ros2 control switch_controllers -c " + currentRobot->getNamespace()+"/"+currentRobot->getCM_Nodename() + " --start";
    for(auto controller : start_controllers){
      cmd = cmd + " " + controller;
    }
    cmd = cmd + " &";
    std::system(cmd.c_str());
    //rclcpp::Duration timeout = rclcpp::Duration(5,0);
    //currentRobot->controller_manager_->switch_controller(start_controllers, stop_controllers, controller_manager_msgs::srv::SwitchController::Request::STRICT,true, timeout);
    //std::cout << "CONTROLLERS STARTED" << std::endl;

    if (!currentRobot->controller_manager_->has_parameter("update_rate")) {
      RCLCPP_ERROR_STREAM(this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "controller manager doesn't have an update_rate parameter");
      return;
    }

    this->ignRos2ControlPtr->update_rate = currentRobot->controller_manager_->get_parameter("update_rate").as_int();

    this->ignRos2ControlPtr->control_period_ = rclcpp::Duration(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(
          1.0 / static_cast<double>(this->ignRos2ControlPtr->update_rate)
        )
      )
    );

    currentRobot->setObjectHandle(_entity);
    //this->ignRos2ControlPtr->entity_ = _entity;

    //currentRobot->launchControllers();

  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::PreUpdate(const ignition::gazebo::UpdateInfo & _info, ignition::gazebo::EntityComponentManager & /*_ecm*/)
  {
    static bool warned{false};

    if (!warned) {
      rclcpp::Duration gazebo_period(_info.dt);

      //Check the period against the simulation period
      if (this->ignRos2ControlPtr->control_period_ < _info.dt) {
        RCLCPP_ERROR_STREAM( this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), "Desired controller update period (" << 
                                                                  this->ignRos2ControlPtr->control_period_.seconds() << 
                                                                  " s) is faster than the gazebo simulation period (" <<
                                                                  gazebo_period.seconds() << " s)."
        );
      } 
      else if (this->ignRos2ControlPtr->control_period_ > gazebo_period) {
        RCLCPP_WARN_STREAM( this->ignRos2ControlPtr->ign_ros2_control_node->get_logger(), " Desired controller update period (" << 
                                                                this->ignRos2ControlPtr->control_period_.seconds() <<
                                                                " s) is slower than the gazebo simulation period (" <<
                                                                gazebo_period.seconds() << " s)."
        );
      }

      warned = true;

    }

    // Always set commands on joints, otherwise at low control frequencies the joints tremble
    // as they are updated at a fraction of gazebo sim time

    if(this->ignRos2ControlPtr->robots.size() !=0){
      for(auto *robot : this->ignRos2ControlPtr->robots){
        if (robot->controller_manager_ != nullptr) {
          robot->controller_manager_->write();
        }
      }
    }
  }

  //////////////////////////////////////////////////
  void IgnitionROS2ControlPlugin::PostUpdate(const ignition::gazebo::UpdateInfo & _info, const ignition::gazebo::EntityComponentManager & /*_ecm*/) 
  {
    // Get the simulation time and period
    rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count(), RCL_ROS_TIME);

    rclcpp::Duration sim_period = sim_time_ros - this->ignRos2ControlPtr->last_update_sim_time_ros_;

    if (sim_period >= this->ignRos2ControlPtr->control_period_) {
      this->ignRos2ControlPtr->last_update_sim_time_ros_ = sim_time_ros;

      if(this->ignRos2ControlPtr->robots.size() !=0){
        for(auto *robot : this->ignRos2ControlPtr->robots){
          if (robot->controller_manager_ != nullptr) {
            auto ign_controller_manager = std::dynamic_pointer_cast<ign_ros2_control::IgnitionSystemInterface>(robot->controller_manager_);
            robot->controller_manager_->read();
            robot->controller_manager_->update();
          }
        }
      }

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
