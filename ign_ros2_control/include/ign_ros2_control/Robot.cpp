#include "Robot.hpp"

namespace ign_ros2_control{
    
    // Constructors

    Robot::Robot(std::string type, std::string name, std::string paramFileName, rclcpp::Node::SharedPtr ign_ros2_control_node){
        this->__init__(type, name, paramFileName, ign_ros2_control_node);
    }

    void Robot::__init__(std::string type, std::string name, std::string paramFileName, rclcpp::Node::SharedPtr ign_ros2_control_node){
        this->setModelType(type);
        this->setName(name);

        std::string ns;
        if (this->getName().empty()) 
            ns = "/";
        else   
            ns = "/model/"+this->getName();

        this->setNamespace(ns);
        RCLCPP_INFO(ign_ros2_control_node->get_logger(),"Selected Namespace: " + this->getNamespace() );

        this->setParamFilePath(paramFileName);
        //this->initRSP_Node();

        this->loadURDF(ign_ros2_control_node);
    }

    // Setters

    void Robot::setName(std::string name){
        this->name = name;
    }

    void Robot::setModelType(std::string model_type){
        this->model_type = model_type;
    }

    void Robot::setNamespace(std::string ns){
        this->ns = ns;
    }

    void Robot::setURDF(std::string urdf_txt){
        this->urdf = urdf_txt;
    }

    void Robot::setMasterLink(std::string link){
        this->master_link = link;
    }

    void Robot::setParamFilePath(std::string path){
        this->paramFilePath = path;
    }

    void Robot::setObjectHandle(ignition::gazebo::Entity handle){
        this->objectHandle = handle;
    }

    // Getters

    std::string Robot::getName(){
        return this->name;
    }

    std::string Robot::getModelType(){
        return this->model_type;
    }

    std::string Robot::getNamespace(){
        return this->ns;
    }

    std::string Robot::getURDF(){
        return this->urdf;
    }

    std::string Robot::getRSP_Nodename(){
        return this->rsp_nodename;
    }

    std::string Robot::getCM_Nodename(){
        return this->cm_nodename;
    }

    std::string Robot::getMasterLink(){
        return this->master_link;
    }

    std::string Robot::getParamFilePath(){
        return this->paramFilePath;
    }

    ignition::gazebo::Entity Robot::getObjectHandle(){
        return this->objectHandle;
    }

    // Methods

    // void Robot::initRSP_Node(){
    //     // launching robot_state_publisher
    //     std::string cmd = "ros2 launch ntu_gazebo models_rsp.launch.py model:='" + this->getModelType() +  "' model_name:='" + this->getName() + "' > /dev/null & ";
    //     std::system(cmd.c_str());
    // }

    //void Robot::launchControllers(){
    //    std::string cmd = "ros2 launch ntu_gazebo models_sim.launch.py model:='" + this->getModelType() +  "' model_name:='" + this->getName() + "' &";
    //    std::system(cmd.c_str());
    //}

    // void Robot::stopRSP_Node(){
    //     std::string cmd = "pkill -f robot_state_publisher";
    //     std::system(cmd.c_str());
    // }

    std::map<std::string, ignition::gazebo::Entity> Robot::GetEnabledJoints(const ignition::gazebo::Entity & _entity, ignition::gazebo::EntityComponentManager & _ecm) const
    {
        std::map<std::string, ignition::gazebo::Entity> output;

        std::vector<std::string> enabledJoints;

        // Get all available joints
        auto jointEntities = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Joint());

        // Iterate over all joints and verify whether they can be enabled or not
        for (const auto & jointEntity : jointEntities) {
            const auto jointName = _ecm.Component<ignition::gazebo::components::Name>(jointEntity)->Data();

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
                    //RCLCPP_INFO(node_->get_logger(), "[ign_ros2_control] Fixed joint [%s] (Entity=%d)] is skipped", jointName.c_str(), jointEntity);
                    continue;
                }
                case sdf::JointType::REVOLUTE2:
                case sdf::JointType::SCREW:
                case sdf::JointType::BALL:
                case sdf::JointType::UNIVERSAL:{
                    //RCLCPP_WARN(node_->get_logger(), "[ign_ros2_control] Joint [%s] (Entity=%d)] is of unsupported type.\n Only joints with a single axis are supported.",jointName.c_str(), jointEntity);
                    continue;
                }
                default:{
                    //RCLCPP_WARN(node_->get_logger(),"[ign_ros2_control] Joint [%s] (Entity=%d)] is of unknown type",jointName.c_str(), jointEntity);
                    continue;
                }
            }
            output[jointName] = jointEntity;
        }

        return output;
    }


    void Robot::loadURDF(rclcpp::Node::SharedPtr node_){
        // Read urdf from ros parameter server
        std::string urdf_string;

        std::string robot_description_node_ = this->getNamespace()+"/"+this->getRSP_Nodename();
        std::string robot_description_param_name = "robot_description";

        //std::cout << robot_description_node_ << std::endl;

        RCLCPP_INFO(node_->get_logger(), "Loading URDF model");

        try {
            using namespace std::chrono_literals;

            auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_, robot_description_node_);

            while (!parameters_client->wait_for_service(0.5s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for %s service. Exiting.", robot_description_node_.c_str());
                    return;
                }
                RCLCPP_ERROR( node_->get_logger(), "%s service not available, waiting again...", robot_description_node_.c_str());
            }

            RCLCPP_INFO( node_->get_logger(), "connected to service!! %s asking for %s", robot_description_node_.c_str(), robot_description_param_name.c_str());

            // search and wait for robot_description on param server
            while (urdf_string.empty()) {
                RCLCPP_DEBUG( node_->get_logger(), "param_name %s", robot_description_param_name.c_str());

                try {
                    auto f = parameters_client->get_parameters({robot_description_param_name});
                    f.wait();
                    std::vector<rclcpp::Parameter> values = f.get();
                    urdf_string = values[0].as_string();
                } 
                catch (const std::exception & e) {
                    RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
                }

                if (!urdf_string.empty()) {
                    break;
                } 
                else {
                    RCLCPP_ERROR( node_->get_logger(), "ign_ros2_control plugin is waiting for model URDF in parameter [%s] on the ROS param server.", robot_description_param_name.c_str());
                }
                usleep(100000);
            }

            RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");
            
            this->setURDF(urdf_string);
        }
        catch (const std::runtime_error & ex) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Error parsing URDF in ign_ros2_control plugin, plugin not active : " << ex.what());
            return;
        }

        // Parse and get informations about urdf
        urdf::Model model;

        model.initString(urdf_string);

        this->setMasterLink(model.links_.begin()->first);
    }


}