//
// Created by joaopedro on 12/07/21.
//

#include <object_recognition_client/obj_localization_ros.h>

int main(int argc, char **argv) {

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

    while (ros::ok()) {
        o.spin(250);
        o.requestData(p);
        ROS_INFO_STREAM("\n Child frame name: " << p.getName() << "\n Parent frame name: " << p.getParentName()
                                                << "\n Position [x,y,z]: [" <<
                                                p.getPosition().x() << ", " << p.getPosition().y() << ", "
                                                << p.getPosition().z() << "]\n Orientation [x,y,z,w]: [" <<
                                                p.getQuaternionOrientation().x() << ", "
                                                << p.getQuaternionOrientation().y() << ", "
                                                << p.getQuaternionOrientation().z() << ", "
                                                << p.getQuaternionOrientation().w() << "]");

    }

    o.spin(250);
    o.stopApp();
    return 0;
}