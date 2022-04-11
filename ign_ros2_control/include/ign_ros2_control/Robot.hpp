
#include <urdf/model.h>
#include <string>


namespace ign_ros2_control
{
    
    class Robot{
        private:
            // Parameters
            std::string name;
            std::string model_type;
            std::string ns;
            std::string urdf;
            std::string rsp_nodename = "robot_state_publisher";
            std::string cm_nodename = "controller_manager";
            std::string master_link;
            std::string paramFilePath;
            ignition::gazebo::Entity objectHandle;

            // Setters
            void setNamespace(std::string);
            void setName(std::string);
            void setModelType(std::string);
            void setMasterLink(std::string);
            void setParamFilePath(std::string);

            // Methods

            void __init__(std::string,std::string, std::string, rclcpp::Node::SharedPtr);
            //void initRSP_Node();
            void loadURDF(rclcpp::Node::SharedPtr);
            

        public:

            // Parameters
            /// \brief Controller manager
            std::shared_ptr<controller_manager::ControllerManager> controller_manager_{nullptr};

            // Constructors
            Robot(std::string, std::string, std::string, rclcpp::Node::SharedPtr);

            // Setters
            void setURDF(std::string);
            void setObjectHandle(ignition::gazebo::Entity);

            // Getters 
            std::string getNamespace();
            std::string getName();
            std::string getModelType();
            std::string getMasterLink();
            std::string getURDF();
            std::string getRSP_Nodename();
            std::string getCM_Nodename();
            std::string getParamFilePath();
            ignition::gazebo::Entity getObjectHandle();

            // Methods
            //void launchControllers();
            std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(const ignition::gazebo::Entity &, ignition::gazebo::EntityComponentManager &) const;
            //void stopRSP_Node();
    };
}