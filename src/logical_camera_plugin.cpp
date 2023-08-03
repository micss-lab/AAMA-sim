#include "logical_camera_plugin.h"

using namespace gazebo;
using namespace std;
using namespace ros;

GZ_REGISTER_SENSOR_PLUGIN(LogicalCameraPlugin);

void LogicalCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf){
    // Get the parent sensor.
    gazebo::sensors::SensorPtr genericSensor = _sensor;
    this->parentSensor = std::dynamic_pointer_cast<sensors::LogicalCameraSensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor){
        gzerr << "LogicalCameraPlugin requires a LogicalCameraSensor.\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&LogicalCameraPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    ROS_INFO("LogicalCameraPlugin correctly loaded!!!");
    ROS_INFO("_near:=%g",this->parentSensor->Near());
    ROS_INFO("_far:=%g",this->parentSensor->Far());
    ROS_INFO("_horizontalFOV:=%g",this->parentSensor->HorizontalFOV());
    ROS_INFO("_aspect_ratio:=%g",this->parentSensor->AspectRatio());
    ROS_INFO("_asd:=%s",this->parentSensor->ParentName().c_str());

    std::string s =this->parentSensor->ParentName();
    std::string delimiter = "::";
    std::string modelName = s.substr(0, s.find(delimiter));
    nh = new ros::NodeHandle(modelName);
    image_pub = nh->advertise<aama_sim::LogicalImage>("logical_camera_image", 1, true);
}

void LogicalCameraPlugin::OnUpdate(){
    msgs::LogicalCameraImage logical_image;
    aama_sim::LogicalImage msg;

    logical_image = this->parentSensor->Image();
    gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();


    int number_of_models = logical_image.model_size();
    for(int i=0; i < number_of_models; i++){
        aama_sim::Model model_msg;

        std::string modelName = logical_image.model(i).name();
        if (modelName != "ground_plane" && modelName != "lab_model") {
            // Pose of camera in world coordinates
            ignition::math::Pose3d cameraPose = gazebo::msgs::ConvertIgn(logical_image.pose());

            // Pose of first model relative to camera
            ignition::math::Pose3d modelRelativePose = gazebo::msgs::ConvertIgn(logical_image.model(i).pose());

            // Pose of first model in world coordinates
            ignition::math::Pose3d modelWorldPose = modelRelativePose + cameraPose;

            model_msg.pose.position.x = modelWorldPose.Pos().X();
            model_msg.pose.position.y = modelWorldPose.Pos().Y();
            model_msg.pose.position.z = modelWorldPose.Pos().Z();

            model_msg.type = logical_image.model(i).name();

            msg.models.push_back(model_msg);
        }
    }

    this->image_pub.publish(msg);
}