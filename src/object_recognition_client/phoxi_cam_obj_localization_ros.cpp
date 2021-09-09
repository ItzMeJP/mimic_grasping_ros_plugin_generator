//
// Created by joaopedro on 16/07/21.
//

#include "object_recognition_client/phoxi_cam_obj_localization_ros.h"

PhoxiCamObjLocalizationROS::PhoxiCamObjLocalizationROS() {

}

PhoxiCamObjLocalizationROS::~PhoxiCamObjLocalizationROS() {
    //stopApp();
}

bool PhoxiCamObjLocalizationROS::setAppConfigPath(std::string _file_with_path) {
    plugin_config_path_ = _file_with_path;
    return true;
}

bool PhoxiCamObjLocalizationROS::setAppExec(std::string _file_with_path_or_command) {
    plugin_exec_path_ = _file_with_path_or_command;
    return true;
}

bool PhoxiCamObjLocalizationROS::setAppTermination(std::string _file_with_path_or_command) {
    plugin_terminator_path_ = _file_with_path_or_command;
    return true;
}

bool PhoxiCamObjLocalizationROS::setTargetName(std::string _name_with_path) {
    target_name_ = _name_with_path;
    return true;
}
bool PhoxiCamObjLocalizationROS::loadAppConfiguration() {

    std::ifstream config_file(plugin_config_path_, std::ifstream::binary);
    if (config_file) {
        try {
            config_file >> config_data_;
        } catch (const std::exception &e) {
            output_string_ = e.what();
            return false;
        }
    } else {
        output_string_ = plugin_name + " configuration file not found. Current path: \"" + plugin_config_path_ + "\"";
        return false;
    }

    wait_for_recognition_server_timeout_in_seconds_ = config_data_["recognition_server"]["wait_for_server_timeout_in_seconds"].asInt();
    wait_for_recognition_result_timeout_in_seconds_ = config_data_["recognition_server"]["wait_for_result_timeout_in_seconds"].asInt();
    wait_for_camera_server_timeout_in_seconds_ = config_data_["camera_server"]["wait_for_server_timeout_in_seconds"].asInt();
    wait_for_camera_result_server_timeout_in_seconds = config_data_["camera_server"]["wait_for_result_timeout_in_seconds"].asInt();

    ros_recognition_namespace_ = config_data_["recognition_server"]["ros_namespace"].asString();
    ros_camera_namespace_ = config_data_["camera_server"]["ros_namespace"].asString();

    recognition_action_name_ = config_data_["recognition_server"]["action_name"].asString();
    camera_action_name_ = config_data_["camera_server"]["action_name"].asString();

    if (plugin_exec_path_.empty()) {
        output_string_ = plugin_name + " exec is not defined.";
        status_ = FEEDBACK::ERROR;
        return false;
    }

    if (plugin_terminator_path_.empty()) {
        output_string_ = plugin_name + " terminator is not defined.";
        status_ = FEEDBACK::ERROR;
        return false;
    }

    if(ros_recognition_namespace_.empty() || ros_camera_namespace_.empty()){
        output_string_ = "Ros namespaces cannot be empty";
        status_ = FEEDBACK::ERROR;
        return false;
    }

    return true;
}

bool PhoxiCamObjLocalizationROS::runApp() {

    status_ = FEEDBACK::RUNNING;
    stopApp();
    initRosNode();

    pipe_to_obj_localization_.reset(
            popen(plugin_exec_path_.c_str(), "r")); // TODO: find a solution to treat this error!

    int descriptor = fileno(pipe_to_obj_localization_.get());
    fcntl(descriptor, F_SETFL, O_NONBLOCK);

    obj_localization_thread_reader_.reset(
            new boost::thread(boost::bind(&PhoxiCamObjLocalizationROS::execCallback, this, descriptor)));

    recognition_action_server_ = std::make_shared<actionlib::SimpleActionClient<object_recognition_skill_msgs::ObjectRecognitionSkillAction>>(
            recognition_action_name_, true);

    camera_action_server_ = std::make_shared<actionlib::SimpleActionClient<photoneo_skill_msgs::PhotoneoSkillAction>>(
            camera_action_name_, true);

    first_obj_localization_communication_ = false;

    if (!err_flag_pipe_corrupted_) {

        status_ = FEEDBACK::INITIALIZING;
        output_string_ = "Waiting for Camera RosAction Server startup...";
        ROS_WARN_STREAM(output_string_);

        if (!camera_action_server_->waitForServer(ros::Duration(wait_for_camera_server_timeout_in_seconds_, 0))) {
            output_string_ = "The Camera Ros Action was not properly initialized";
            ROS_ERROR_STREAM(output_string_);
            status_ = FEEDBACK::ERROR;
            stopApp();
            return false;
        }

        if (!recognition_action_server_->waitForServer(ros::Duration(wait_for_recognition_server_timeout_in_seconds_, 0))) {
            output_string_ = "The Object Recognition Ros Action was not properly initialized";
            ROS_ERROR_STREAM(output_string_);
            status_ = FEEDBACK::ERROR;
            stopApp();
            return false;
        }
    } else
        stopApp();

    return true;
}

bool PhoxiCamObjLocalizationROS::initRosNode() {


    std::string node_name = ros_recognition_namespace_ + "_mimic_grasping_plugin_node";
    int argc;
    char **argv;
    ros::init(argc, argv, node_name);

    if (!ros::master::check()) {
        std::cerr<<"No roscore found, calling roscore automatically..."<<std::endl;
        popen("roscore", "r");
        sleep(1);
    }
    node_handle_.reset(new ros::NodeHandle());
    private_node_handle_.reset(new ros::NodeHandle("~"));
    spinner_.reset(new ros::AsyncSpinner(0));
    spinner_->start();

    return true;
}

bool PhoxiCamObjLocalizationROS::stopApp() {

    if (!first_obj_localization_communication_) {

        obj_localization_thread_reader_->interrupt();
        obj_localization_thread_reader_->join();
        spinner_->stop();
        //pclose(pipe_to_obj_localization_.get());
        DEBUG_MSG("Killing Phoxi Object Localization Server.");
        popen(plugin_terminator_path_.c_str(), "r");
        first_obj_localization_communication_ = true;
        return true;
    } else {
        output_string_ = "Cannot kill since the object recognition process is not running.";
        return false;
    }
}


bool PhoxiCamObjLocalizationROS::requestData(Pose &_result) {

    status_ = FEEDBACK::PROCESSING;

    if (err_flag_pipe_corrupted_) {
        output_string_ = "Corrupted pipe to " + plugin_name + " interface.";
        status_ = FEEDBACK::ERROR;
        return false;
    }

    photoneo_skill_msgs::PhotoneoSkillGoal camera_goal;
    camera_goal.start = true;
    camera_action_server_->sendGoal(camera_goal);

    bool finished_before_timeout = camera_action_server_->waitForResult(ros::Duration(wait_for_camera_result_server_timeout_in_seconds));

    if (finished_before_timeout && camera_action_server_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED ) {
        output_string_ = "Request camera image with success. " + camera_action_server_->getState().toString();
        DEBUG_MSG(output_string_ );
        status_ = FEEDBACK::PROCESSING;
    } else {
        output_string_ = "Fail to request camera image " + camera_action_server_->getState().toString();
        DEBUG_MSG( output_string_ );
        status_ = FEEDBACK::ERROR;
        return false;
    }


    //TODO: check if it is needed a delay time...

    object_recognition_skill_msgs::ObjectRecognitionSkillGoal goal;
    goal.clusterIndex = -1;
    goal.objectModel = target_name_;
    recognition_action_server_->sendGoal(goal);
    finished_before_timeout = recognition_action_server_->waitForResult(ros::Duration(wait_for_recognition_result_timeout_in_seconds_));

    if (finished_before_timeout && recognition_action_server_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        output_string_ = "Request data with success. " + recognition_action_server_->getState().toString();
        DEBUG_MSG( output_string_ );
        status_ = FEEDBACK::FINISHED;
    } else {
        output_string_ = "Fail to request data " + recognition_action_server_->getState().toString();
        DEBUG_MSG( output_string_ );
        status_ = FEEDBACK::ERROR;
        return false;
    }

    Eigen::Translation3d t(recognition_action_server_->getResult().get()->pose.pose.position.x,
                           recognition_action_server_->getResult().get()->pose.pose.position.y,
                           recognition_action_server_->getResult().get()->pose.pose.position.z);

    Eigen::Quaterniond q(recognition_action_server_->getResult().get()->pose.pose.orientation.w,
                         recognition_action_server_->getResult().get()->pose.pose.orientation.x,
                         recognition_action_server_->getResult().get()->pose.pose.orientation.y,
                         recognition_action_server_->getResult().get()->pose.pose.orientation.z);

    _result.setName(getNameFromPath(target_name_)+"_frame_id");
    _result.setParentName(recognition_action_server_->getResult().get()->pose.header.frame_id);
    _result.setPosition(t);
    _result.setQuaternionOrientation(q);

    return true;

}

std::string PhoxiCamObjLocalizationROS::getNameFromPath(std::string _s){
    size_t pos = 0;
    std::string token, delimiter = "/";

    while (((pos = _s.find(delimiter)) != std::string::npos) && (ros::ok())) {
        token = _s.substr(0, pos);
        _s.erase(0, pos + delimiter.length());
    }

    delimiter=".";
    _s =  _s.substr(0, _s.find(delimiter));
    return _s;
}

int PhoxiCamObjLocalizationROS::getStatus() {
    if (err_flag_pipe_corrupted_) {
        output_string_ = "Corrupted pipe to " + plugin_name + " interface. ";
        return FEEDBACK::ERROR;
    }
    return status_;
}

std::string PhoxiCamObjLocalizationROS::getOutputString() {
    return MSG_PREFIX + output_string_;
}

void PhoxiCamObjLocalizationROS::publisherResultCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
    current_received_msg_ = *_msg;
}


void PhoxiCamObjLocalizationROS::execCallback(int _file_descriptor) {
    for (;;) {
        try {
            this->exec(_file_descriptor);
            boost::this_thread::interruption_point();
            //boost::this_thread::sleep(boost::posix_time::milliseconds(500)); //interruption with sleep
        }
        catch (boost::thread_interrupted &) {
            DEBUG_MSG( plugin_name + " pipe thread is stopped." );
            return;
        }
    }
}

void PhoxiCamObjLocalizationROS::exec(int _file_descriptor) {

    char buffer[128];

    ssize_t r = read(_file_descriptor, buffer, 128);

    if (r == -1 && errno == EAGAIN) {}
    else if (r > 0) {
        current_pipe_output_str_ = buffer;
    } else {
        err_flag_pipe_corrupted_ = true;
        return;
    }

}

void PhoxiCamObjLocalizationROS::spin(int _usec) {
    obj_localization_thread_reader_->timed_join(boost::chrono::milliseconds(_usec));
    if (err_flag_pipe_corrupted_) {
        stopApp();
    }

}