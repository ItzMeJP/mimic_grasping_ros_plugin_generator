//
// Created by joaopedro on 16/07/21.
//

#define MSG_PREFIX "<PhoxiCamObjLocalizationROSPlugin> "

#ifndef NDEBUG
#define DEBUG_MSG(str) do { std::cout << "\033[;33m" <<"[WARN] "<< MSG_PREFIX << str << "\033[0m"<< std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif

#ifndef MIMIC_GRASPING_ROS_PLUGIN_GENERATOR_PHOXI_CAM_OBJ_LOCALIZATION_ROS_H
#define MIMIC_GRASPING_ROS_PLUGIN_GENERATOR_PHOXI_CAM_OBJ_LOCALIZATION_ROS_H

#include <fcntl.h>
#include <unistd.h> //read function
#include <boost/thread.hpp>
#include <fstream>
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <object_recognition_skill_msgs/ObjectRecognitionSkillAction.h>
#include <photoneo_skill_msgs/PhotoneoSkillAction.h>

#include <mimic_grasping_api/localization_base.h>

class PhoxiCamObjLocalizationROS : public LocalizationBase {
public:
    PhoxiCamObjLocalizationROS();

    ~PhoxiCamObjLocalizationROS();

    bool setAppConfigPath(std::string _file_with_path);

    bool setAppExec(std::string _file_with_path_or_command);

    bool setAppTermination(std::string _file_with_path_or_command);

    bool setTargetName(std::string _name);

    bool loadAppConfiguration();

    bool runApp();

    bool stopApp();

    bool requestData(Pose &_result);

    int getStatus();

    void spin(int _usec);

    std::string getOutputString();

protected:

    bool first_obj_localization_communication_ = true,
            err_flag_pipe_corrupted_ = false;

    std::string current_pipe_output_str_;

    std::shared_ptr<boost::thread> obj_localization_thread_reader_;
    std::shared_ptr<FILE> pipe_to_obj_localization_;

    std::shared_ptr<ros::NodeHandle> node_handle_, private_node_handle_;

    std::shared_ptr<ros::AsyncSpinner> spinner_;
    geometry_msgs::PoseStamped current_received_msg_;
    std::shared_ptr<actionlib::SimpleActionClient<object_recognition_skill_msgs::ObjectRecognitionSkillAction>> recognition_action_server_;
    std::shared_ptr<actionlib::SimpleActionClient<photoneo_skill_msgs::PhotoneoSkillAction>> camera_action_server_;

    Json::Value config_data_;

    void publisherResultCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg);

    void execCallback(int _descriptor);

    void exec(int _descriptor);

    bool initRosNode();

    std::string getNameFromPath(std::string _s);

private:

    int wait_for_recognition_server_timeout_in_seconds_,
            wait_for_recognition_result_timeout_in_seconds_,
            wait_for_camera_server_timeout_in_seconds_,
            wait_for_camera_result_server_timeout_in_seconds;

    std::string ros_recognition_namespace_,
            recognition_action_name_,
            ros_camera_namespace_,
            camera_action_name_;

    void freeMem();

};

/** ################## Factory Function - Plugin EntryPoint  ##################  **/
/** This part should be added in all plugins, therefore the plugin management can identified it. **/

/** Define the Plugin name **/
std::string plugin_name = "PhoxiCamObjLocalizationROSPlugin";

PLUGIN_EXPORT_C
auto GetPluginFactory() -> IPluginFactory * {

    static PluginFactory pinfo = [] {
        /** Properly set the plugin name and version **/
        auto p = PluginFactory(plugin_name.c_str(), "01_09_2021");
        /** Register all classes defined inside the plugin **/
        p.registerClass<PhoxiCamObjLocalizationROS>("PhoxiCamObjLocalizationROS");
        return p;
    }();
    return &pinfo;
}

struct _DLLInit {
    _DLLInit() {
        std::cout << " [TRACE] Shared library " << plugin_name << " loaded OK.\n";
    }

    ~_DLLInit() {
        std::cout << " [TRACE] Shared library " << plugin_name << " unloaded OK.\n";
    }
} dll_init;


#endif //MIMIC_GRASPING_ROS_PLUGIN_GENERATOR_PHOXI_CAM_OBJ_LOCALIZATION_ROS_H
