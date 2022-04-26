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

#ifndef IGN_ROS2_CONTROL__IGN_SYSTEM_HPP_
#define IGN_ROS2_CONTROL__IGN_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/laserscan.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/Lidar.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>

#include <ignition/transport/Node.hh>

#include "ign_ros2_control/ign_system_interface.hpp"

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd = 0;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd = 0;

  /// \brief Current cmd joint effort
  double joint_effort_cmd = 0;

  /// \brief handles to the joints from within Gazebo
  ignition::gazebo::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  ign_ros2_control::IgnitionSystemInterface::ControlMethod joint_control_method;
};

class SensorData{
  public:
    /// \brief sensors's name.
    std::string name{};

    /// \brief sensors's type.
    std::string type{};

    /// \brief sensors's topic name.
    std::string topicName{};

    /// \brief handles to the sensors from within Gazebo
    ignition::gazebo::Entity sim_sensors_ = ignition::gazebo::kNullEntity;

    /// \brief 1 dimensional array that constains all unique properties for sensors
    std::vector<double> sensor_data_;

    /// \brief multidimensional array that constains all array properties for sensors
    std::vector<std::vector<double>> sensor_array_data_;
};

class ImuData : public SensorData
{
  public:
    /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
    const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
    };

    ImuData(){
      this->sensor_data_.resize(10);
    }

    /// \brief callback to get the IMU topic values
    void OnIMU(const ignition::msgs::IMU &_msg);
};

void ImuData::OnIMU(const ignition::msgs::IMU &_msg)
{
  this->sensor_data_[0] = _msg.orientation().x();
  this->sensor_data_[1] = _msg.orientation().y();
  this->sensor_data_[2] = _msg.orientation().z();
  this->sensor_data_[3] = _msg.orientation().w();
  this->sensor_data_[4] = _msg.angular_velocity().x();
  this->sensor_data_[5] = _msg.angular_velocity().y();
  this->sensor_data_[6] = _msg.angular_velocity().z();
  this->sensor_data_[7] = _msg.linear_acceleration().x();
  this->sensor_data_[8] = _msg.linear_acceleration().y();
  this->sensor_data_[9] = _msg.linear_acceleration().z();
}

class LidarData : public SensorData
{
public:
  /// \brief An array per LIDAR with angle_min, angle_max, angle_increment, time_increment, scan_time, range_min, range_max, ranges_size, intensities_size, ranges, intensities
  const std::map<std::string, size_t> interface_name_map = {
      {"angle_min", 0},
      {"angle_max", 1},
      {"angle_increment", 2},
      {"time_increment", 3},
      {"scan_time", 4},
      {"range_min", 5},
      {"range_max", 6},
      {"ranges_size", 7},
      {"intensities_size", 8},
      {"ranges", 9},
      {"intensities", 10},
  };

  LidarData(){
    this->sensor_data_.resize(9);
    this->sensor_array_data_.resize(2);
    this->sensor_array_data_[0].resize(1);
    this->sensor_array_data_[1].resize(1);
  }

  /// \brief callback to get the IMU topic values
  void OnLIDAR(const ignition::msgs::LaserScan &_msg);
};

void LidarData::OnLIDAR(const ignition::msgs::LaserScan &_msg)
{
  this->sensor_data_[0] = _msg.angle_min();
  this->sensor_data_[1] = _msg.angle_max();
  this->sensor_data_[2] = _msg.angle_step();
  this->sensor_data_[3] = 5.0; // valores default, arrumar para time_increment
  this->sensor_data_[4] = 6.0; // valores default, arrumar para scan_time
  this->sensor_data_[5] = _msg.range_min();
  this->sensor_data_[6] = _msg.range_max();

  this->sensor_data_[7] = _msg.ranges_size();
  this->sensor_array_data_[0].resize(_msg.ranges_size());

  this->sensor_data_[8] = _msg.intensities_size();
  this->sensor_array_data_[1].resize(_msg.ranges_size());
  
  for(int i=0; i<_msg.ranges_size(); i++)
    this->sensor_array_data_[0][i] = _msg.ranges()[i];

  for(int i=0; i<_msg.intensities_size(); i++)
    this->sensor_array_data_[1][i] = _msg.intensities()[i];  

}

// class CameraData : public SensorData
// {
// public:
//   /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
//   std::array<double, 6> sensor_data_;

//   std::array<std::array<double,25>,1> array_data_;

//   const std::map<std::string, size_t> interface_name_map = {
//       {"height", 0},
//       {"width", 1},
//       {"encoding", 2},
//       {"is_bigendian", 3},
//       {"step", 4},
//       {"data", 5},
//       {"data_size", 6},
//   };

//   /// \brief callback to get the IMU topic values
//   void OnCAMERA(const ignition::msgs::Image &_msg);
// };

// void CameraData::OnCAMERA(const ignition::msgs::Image &_msg)
// {
//   this->sensor_data_[0] = (double) _msg.height(); 
//   this->sensor_data_[1] = (double) _msg.width();
//   this->sensor_data_[2] = _msg.pixel_format_type();
//   this->sensor_data_[3] = true;
//   this->sensor_data_[4] = _msg.step();
//   //this->sensor_data_[5] = _msg.data();
//   this->sensor_data_[6] = _msg.data().length();

  //std::cout << std::to_string(_msg.pixel_format_type()); << std::endl;


  for(int i=0; i< _msg.data().length(); i++)
    this->array_data_[0][i] = _msg.data()[i];
}

namespace ign_ros2_control
{

  class IgnitionSystemPrivate
  {
  public:
    IgnitionSystemPrivate() = default;

    ~IgnitionSystemPrivate() = default;
    /// \brief Degrees od freedom.
    size_t n_dof_;

    /// \brief vector with the joint's names.
    std::vector<struct jointData> joints_;

    /// \brief vector with the imus .
    std::vector<std::shared_ptr<ImuData>> imus_;

    /// \brief vector with the lidar .
    std::vector<std::shared_ptr<LidarData>> lidars_;

    /// \brief vector with the lidar .
    std::vector<std::shared_ptr<CameraData>> cameras_;

    /// \brief state interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::StateInterface> state_interfaces_;

    /// \brief command interfaces that will be exported to the Resource Manager
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    /// \brief Entity component manager, ECM shouldn't be accessed outside those
    /// methods, otherwise the app will crash
    ignition::gazebo::EntityComponentManager *ecm;

    /// \brief controller update rate
    int *update_rate;

    /// \brief Ignition communication node.
    ignition::transport::Node node;
  };

  // These class must inherit `ign_ros2_control::IgnitionSystemInterface` which implements a
  // simulated `ros2_control` `hardware_interface::SystemInterface`.

  class IgnitionSystem : public IgnitionSystemInterface
  {
  public:
    // Documentation Inherited
    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &system_info) override;

    // Documentation Inherited
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // Documentation Inherited
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Documentation Inherited
    hardware_interface::return_type start() override;

    // Documentation Inherited
    hardware_interface::return_type stop() override;

    // Documentation Inherited
    hardware_interface::return_type read() override;

    // Documentation Inherited
    hardware_interface::return_type write() override;

    // Documentation Inherited
    void configSim(rclcpp::Node::SharedPtr &model_nh, ignition::gazebo::EntityComponentManager &_ecm, int &update_rate) override;

    bool initSim(
        std::map<std::string, ignition::gazebo::Entity> &joints,
        const hardware_interface::HardwareInfo &hardware_info) override;

  private:
    // Register a sensor (for now just IMUs)
    // \param[in] hardware_info hardware information where the data of
    // the sensors is extract.
    void registerSensors(const hardware_interface::HardwareInfo &hardware_info);

    template<typename T>
    void configureSensor(std::shared_ptr<T>,std::vector<hardware_interface::ComponentInfo>, std::string);

    /// \brief Private data class
    std::unique_ptr<IgnitionSystemPrivate> dataPtr;

    /// \brief stop write, read updates while configuring joints array
    bool control;
    bool loopOnce = true;
  };

} // namespace ign_ros2_control

#endif // IGN_ROS2_CONTROL__IGN_SYSTEM_HPP_
