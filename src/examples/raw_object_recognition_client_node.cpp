//
// Created by joaopedro on 12/07/21.
//

#include <plugin_system_management/plugin_system_management.h>
#include <mimic_grasping_api/localization_base.h>
#include <memory>

int main(int argc, char **argv) {

    PluginSystemManagement ph_;

    std::string path = (std::filesystem::current_path().parent_path().string() + "/plugins/");
    if(!ph_.loadDynamicPlugins(path,true)){
        std::cout << ph_.getPluginManagementOutputMsg() << std::endl;
        return false;
    }

    std::cout<< "Number of plugins loaded: " << ph_.GetNumberOfPluginsLoaded() << std::endl;

    std::shared_ptr<LocalizationBase> instance = ph_.CreateInstanceAs<LocalizationBase>(ph_.GetPluginFactoryInfo(0)->Name(),ph_.GetPluginFactoryInfo(0)->GetClassName(0));
    assert(instance != nullptr);






    char const* env_root_folder_path;

    env_root_folder_path =  getenv("MIMIC_GRASPING_SERVER_ROOT");

    if(env_root_folder_path == NULL) {
        ROS_ERROR_STREAM("The environment variable $MIMIC_GRASPING_SERVER_ROOT is not defined.");
        return false;
    }

    std::string root_folder_path_ = std::string(env_root_folder_path);

    ObjLocalizationROS o;

    o.setAppExec(root_folder_path_ + "/scripts/obj_localization_ros_init.sh");
    o.setAppTermination( root_folder_path_ + "/scripts/obj_localization_ros_close.sh");
    o.setAppConfigPath(root_folder_path_ + "/configs/plugin_object_localization_ros.json");
    o.setTargetName(root_folder_path_ + "/models/single_side_bracket.ply");

    if (!o.loadAppConfiguration() || !o.runApp()) {
        std::cout << o.getOutputString() << std::endl;
        return false;
    }

    Pose p;
    int it = 1;
    while (ros::ok()) {
        ROS_INFO_STREAM("#" << it << " localization recognition iteration...");
        o.spin(250);
        if(o.requestData(p)){
        ROS_INFO_STREAM("\n Child frame name: " << p.getName() << "\n Parent frame name: " << p.getParentName()
                                                << "\n Position [x,y,z]: [" <<
                                                p.getPosition().x() << ", " << p.getPosition().y() << ", "
                                                << p.getPosition().z() << "]\n Orientation [x,y,z,w]: [" <<
                                                p.getQuaternionOrientation().x() << ", "
                                                << p.getQuaternionOrientation().y() << ", "
                                                << p.getQuaternionOrientation().z() << ", "
                                                << p.getQuaternionOrientation().w() << "]");
        break;
        }
        else{
            ROS_ERROR_STREAM("ERROR in localization estimation");
        }
        it++;
    }

    o.spin(250);
    o.stopApp();

    std::cout << "Finished..." << std::endl;
    return 0;
}