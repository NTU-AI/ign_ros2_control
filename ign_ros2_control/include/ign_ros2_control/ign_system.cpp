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

#include "ign_system.hpp"

namespace ign_ros2_control
{

  void IgnitionSystem::configSim(rclcpp::Node::SharedPtr &model_nh, ignition::gazebo::EntityComponentManager &_ecm, int &update_rate)
  {
    this->dataPtr = std::make_unique<IgnitionSystemPrivate>();

    this->nh_ = model_nh;

    this->dataPtr->ecm = &_ecm;

    this->dataPtr->update_rate = &update_rate;
  }

  bool IgnitionSystem::initSim(std::map<std::string, ignition::gazebo::Entity> &enableJoints, const hardware_interface::HardwareInfo &hardware_info)
  {
    this->control = false;

    size_t initSize = this->dataPtr->n_dof_;
    // std::cout << "Init Config: " << this->dataPtr->n_dof_ << std::endl;

    this->dataPtr->n_dof_ = this->dataPtr->n_dof_ + hardware_info.joints.size();
    // std::cout << "Total # of joints: " << this->dataPtr->n_dof_ << std::endl;

    RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

    if (this->dataPtr->n_dof_ == 0)
    {
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is not joint available ");
      return false;
    }

    this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

    for (unsigned int j = initSize, k = 0; j < this->dataPtr->n_dof_; j++, k++)
    {

      std::string joint_name = this->dataPtr->joints_[j].name = hardware_info.joints[k].name;

      ignition::gazebo::Entity simjoint = enableJoints[joint_name];
      this->dataPtr->joints_[j].sim_joint = simjoint;

      // Create joint position component if one doesn't exist
      if (!(*this->dataPtr->ecm).EntityHasComponentType(simjoint, ignition::gazebo::components::JointPosition().TypeId()))
      {
        (*this->dataPtr->ecm).CreateComponent(simjoint, ignition::gazebo::components::JointPosition());
      }

      // Create joint velocity component if one doesn't exist
      if (!(*this->dataPtr->ecm).EntityHasComponentType(simjoint, ignition::gazebo::components::JointVelocity().TypeId()))
      {
        (*this->dataPtr->ecm).CreateComponent(simjoint, ignition::gazebo::components::JointVelocity());
      }

      // Create joint force component if one doesn't exist
      if (!(*this->dataPtr->ecm).EntityHasComponentType(simjoint, ignition::gazebo::components::JointForce().TypeId()))
      {
        (*this->dataPtr->ecm).CreateComponent(simjoint, ignition::gazebo::components::JointForce());
      }

      // Accept this joint and continue configuration
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

      // register the command handles
      for (unsigned int i = 0; i < hardware_info.joints[k].command_interfaces.size(); ++i)
      {
        if (hardware_info.joints[k].command_interfaces[i].name == "position")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");

          this->dataPtr->joints_[j].joint_control_method |= POSITION;

          this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joints_[j].joint_position_cmd);
        }
        else if (hardware_info.joints[k].command_interfaces[i].name == "velocity")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");

          this->dataPtr->joints_[j].joint_control_method |= VELOCITY;

          this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joints_[j].joint_velocity_cmd);
        }
        else if (hardware_info.joints[k].command_interfaces[i].name == "effort")
        {
          this->dataPtr->joints_[j].joint_control_method |= EFFORT;

          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");

          this->dataPtr->command_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joints_[j].joint_effort_cmd);
        }
      }

      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
      // register the state handles
      for (unsigned int i = 0; i < hardware_info.joints[k].state_interfaces.size(); ++i)
      {
        if (hardware_info.joints[k].state_interfaces[i].name == "position")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");

          this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joints_[j].joint_position);
        }

        if (hardware_info.joints[k].state_interfaces[i].name == "velocity")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");

          this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joints_[j].joint_velocity);
        }

        if (hardware_info.joints[k].state_interfaces[i].name == "effort")
        {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");

          this->dataPtr->state_interfaces_.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joints_[j].joint_effort);
        }
      }

      // this->dataPtr->joints_.push_back(this->dataPtr->joints_[j]);
    }

    std::cout << "# of Joints: " << this->dataPtr->joints_.size() << std::endl;
    std::cout << "Sensors size: " << hardware_info.sensors.size() << std::endl;
    std::cout << "Joints size: " << hardware_info.joints.size() << std::endl;
    std::cout << "GPIOs size: " << hardware_info.gpios.size() << std::endl;
    std::cout << "Transmissions size: " << hardware_info.transmissions.size() << std::endl;

    registerSensors(hardware_info);

    this->control = true;

    return true;
  }

  template <typename T>
  void IgnitionSystem::configureSensor(std::shared_ptr<T> sensorData, std::vector<hardware_interface::ComponentInfo> sensor_components_, std::string _name)
  {

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name);

    auto sensorTopicComp = this->dataPtr->ecm->Component<ignition::gazebo::components::SensorTopic>(sensorData->sim_sensors_);

    if (sensorTopicComp)
    {
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    hardware_interface::ComponentInfo component;

    for (auto &comp : sensor_components_)
    {
      if (comp.name == _name)
      {
        component = comp;
      }
    }

    for (const auto &state_interface : component.state_interfaces)
    {
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

      size_t data_index = sensorData->interface_name_map.at(state_interface.name);

      if (sensorData->type == "imu")
      {
        this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, &sensorData->sensor_data_[data_index]);
      }

      else if(sensorData->type == "lidar"){
        if(state_interface.name != "ranges" && state_interface.name != "intensities")
          this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, &sensorData->sensor_data_[data_index]);
        else{
          if(state_interface.name == "ranges")
            this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, &sensorData->sensor_array_data_[0][0], &sensorData->sensor_array_data_[0]);

          else if(state_interface.name == "intensities")
            this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, &sensorData->sensor_array_data_[1][0], &sensorData->sensor_array_data_[1]);
          
        }
      }
      else if (sensorData->type == "camera")
      {
        if (state_interface.name != "data"){
          this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, &sensorData->sensor_data_[data_index]);
        }
        else{
          this->dataPtr->state_interfaces_.emplace_back(sensorData->name, state_interface.name, "teste", &sensorData->sensor_str_data_[0]);
        }
      }
    }
  }

  void IgnitionSystem::registerSensors(const hardware_interface::HardwareInfo &hardware_info)
  {
    // Collect gazebo sensor handles
    size_t n_sensors = hardware_info.sensors.size();
    std::vector<hardware_interface::ComponentInfo> sensor_components_;

    for (unsigned int j = 0; j < n_sensors; j++)
    {
      hardware_interface::ComponentInfo component = hardware_info.sensors[j];
      std::cout << "Sensor: " << hardware_info.sensors[j].name << std::endl;
      sensor_components_.push_back(component);
    }

    // This is split in two steps: Count the number and type of sensor and associate the interfaces
    // So we have resize only once the structures where the data will be stored, and we can safely
    // use pointers to the structures

    this->dataPtr->ecm->Each<ignition::gazebo::components::Sensor, ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Sensor *_sensor,
            const ignition::gazebo::components::Name *_name) -> bool
        {
          std::string sensorName = _name->Data().c_str();

          if (sensorName.find("imu") != std::string::npos)
          {
            auto sensorData = std::make_shared<ImuData>();
            sensorData->name = _name->Data();
            sensorData->type = "imu";
            sensorData->sim_sensors_ = _entity;
            this->configureSensor<ImuData>(sensorData, sensor_components_, sensorName);
            this->dataPtr->imus_.push_back(sensorData);
          }
          else if (sensorName.find("lidar") != std::string::npos)
          {
            auto sensorData = std::make_shared<LidarData>();
            sensorData->name = _name->Data();
            sensorData->type = "lidar";
            sensorData->sim_sensors_ = _entity;
            this->configureSensor<LidarData>(sensorData, sensor_components_, sensorName);
            this->dataPtr->lidars_.push_back(sensorData);
          }
          else if (sensorName.find("camera") != std::string::npos)
          {
            auto sensorData = std::make_shared<CameraData>();
            sensorData->name = _name->Data();
            sensorData->type = "camera";
            sensorData->sim_sensors_ = _entity;
            this->configureSensor<CameraData>(sensorData, sensor_components_, sensorName);
            this->dataPtr->cameras_.push_back(sensorData);
          }
          return true;
        });
  }

  hardware_interface::return_type IgnitionSystem::configure(const hardware_interface::HardwareInfo &actuator_info)
  {
    if (configure_default(actuator_info) != hardware_interface::return_type::OK)
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> IgnitionSystem::export_state_interfaces()
  {
    return std::move(this->dataPtr->state_interfaces_);
  }

  std::vector<hardware_interface::CommandInterface> IgnitionSystem::export_command_interfaces()
  {
    return std::move(this->dataPtr->command_interfaces_);
  }

  hardware_interface::return_type IgnitionSystem::start()
  {
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type IgnitionSystem::stop()
  {
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type IgnitionSystem::read()
  {
    for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
    {
      // Get the joint velocity
      const auto *jointVelocity =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocity>(
              this->dataPtr->joints_[i].sim_joint);

      // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
      // Get the joint force
      // const auto * jointForce =
      //   _ecm.Component<ignition::gazebo::components::JointForce>(
      //   this->dataPtr->sim_joints_[j]);

      // Get the joint position
      const auto *jointPositions =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointPosition>(
              this->dataPtr->joints_[i].sim_joint);

      this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
      this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
      // this->dataPtr->joint_effort_[j] = jointForce->Data()[0];
    }

    for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i)
    {
      if (this->dataPtr->imus_[i]->topicName.empty())
      {
        //auto sensorTopicComp = this->dataPtr->ecm->Component<
        //    ignition::gazebo::components::SensorTopic>(this->dataPtr->imus_[i]->sim_sensors_);

        // std::cout << "Handle ID: "<< this->dataPtr->imus_[i]->sim_sensors_ << std::endl;
        if (this->loopOnce){ 

          this->dataPtr->imus_[i]->topicName = this->dataPtr->imus_[i]->name +  "/imu";
          RCLCPP_INFO_STREAM(
              this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name << " has a topic name: " << this->dataPtr->imus_[i]->topicName);

          this->dataPtr->node.Subscribe(
              this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
              this->dataPtr->imus_[i].get());
        }
      }
    }

    for (unsigned int i = 0; i < this->dataPtr->lidars_.size(); ++i)
    {
      if (this->dataPtr->lidars_[i]->topicName.empty())
      {
        // ignition::gazebo::components::SensorTopic* sensorTopicComp = this->dataPtr->ecm->Component<ignition::gazebo::components::SensorTopic>(this->dataPtr->lidars_[i]->sim_sensors_);

        if (this->loopOnce){ 
          this->dataPtr->lidars_[i]->topicName = this->dataPtr->lidars_[i]->name +  "/lidar"; //sensorTopicComp->Data();
          //std::cout << "Sensor Topic: "<< this->dataPtr->lidars_[i]->topicName << std::endl;
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "LIDAR " << this->dataPtr->lidars_[i]->name << " has a topic name: " << this->dataPtr->lidars_[i]->topicName);

          this->dataPtr->node.Subscribe(this->dataPtr->lidars_[i]->topicName, &LidarData::OnLIDAR, this->dataPtr->lidars_[i].get());
        }
      }
    }

    for (unsigned int i = 0; i < this->dataPtr->cameras_.size(); ++i)
    {
      if (this->dataPtr->cameras_[i]->topicName.empty())
      {
        // ignition::gazebo::components::SensorTopic* sensorTopicComp = this->dataPtr->ecm->Component<ignition::gazebo::components::SensorTopic>(this->dataPtr->lidars_[i]->sim_sensors_);

        if (this->loopOnce){ 
          this->dataPtr->cameras_[i]->topicName = this->dataPtr->cameras_[i]->name +  "/cam/image"; //sensorTopicComp->Data();
          //std::cout << "Sensor Topic: "<< this->dataPtr->lidars_[i]->topicName << std::endl;
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "CAMERA " << this->dataPtr->cameras_[i]->name << " has a topic name: " << this->dataPtr->cameras_[i]->topicName);

          this->dataPtr->node.Subscribe(this->dataPtr->cameras_[i]->topicName, &CameraData::OnCAMERA, this->dataPtr->cameras_[i].get());
        }
      }
    }
  

    this->loopOnce = false;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type IgnitionSystem::write()
  {
    for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i)
    {
      if (this->dataPtr->joints_[i].joint_control_method & VELOCITY)
      {
        std::string handle = "Handle Joint: " + std::to_string(this->dataPtr->joints_[i].sim_joint);

        std::string vel = "Velocity: " + std::to_string(this->dataPtr->joints_[i].joint_velocity_cmd);

        // std::cout << handle <<std::endl;
        // std::cout << vel <<std::endl;

        if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
                this->dataPtr->joints_[i].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointVelocityCmd({0}));
        }
        else
        {
          const auto jointVelCmd =
              this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
                  this->dataPtr->joints_[i].sim_joint);
          *jointVelCmd = ignition::gazebo::components::JointVelocityCmd(
              {this->dataPtr->joints_[i].joint_velocity_cmd});
        }
      }

      if (this->dataPtr->joints_[i].joint_control_method & POSITION)
      {
        // Get error in position
        double error;
        error = (this->dataPtr->joints_[i].joint_position -
                 this->dataPtr->joints_[i].joint_position_cmd) *
                *this->dataPtr->update_rate;

        // Calculate target velcity
        double targetVel = -error;

        auto vel =
            this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
                this->dataPtr->joints_[i].sim_joint);

        if (vel == nullptr)
        {
          this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointVelocityCmd({targetVel}));
        }
        else if (!vel->Data().empty())
        {
          vel->Data()[0] = targetVel;
        }
      }

      if (this->dataPtr->joints_[i].joint_control_method & EFFORT)
      {
        if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
                this->dataPtr->joints_[i].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[i].sim_joint,
              ignition::gazebo::components::JointForceCmd({0}));
        }
        else
        {
          const auto jointEffortCmd =
              this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
                  this->dataPtr->joints_[i].sim_joint);
          *jointEffortCmd = ignition::gazebo::components::JointForceCmd(
              {this->dataPtr->joints_[i].joint_effort_cmd});
        }
      }
    }

    return hardware_interface::return_type::OK;
  }
} // namespace ign_ros2_control

#include "pluginlib/class_list_macros.hpp" // NOLINT
PLUGINLIB_EXPORT_CLASS(ign_ros2_control::IgnitionSystem, ign_ros2_control::IgnitionSystemInterface)
