/*
    sample1_tf2_connection.cpp
    TFサンプル1用のソースコード
*/
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

void tf2_static_connection(std::string, std::string, float, float, float, float, float, float);

template<typename T>
T get_ros_param(ros::NodeHandle pnh, std::string arg_name, T default_val){
    T arg_value = default_val;
    if(pnh.hasParam(arg_name))
        pnh.getParam(arg_name, arg_value);
    return arg_value; 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sample1_tf_connection");
    ros::NodeHandle pnh("~");

    //パラメータを取得する
    std::string parent_frame_id;
    std::string child_frame_id;
    float x, y, z, qx, qy, qz;
    parent_frame_id = get_ros_param<std::string>(pnh, "pframe", "map");
    child_frame_id = get_ros_param<std::string>(pnh, "cframe", "robot");
    x = get_ros_param<float>(pnh, "x", 0);
    y = get_ros_param<float>(pnh, "y", 0);
    z = get_ros_param<float>(pnh, "z", 0);
    qx = get_ros_param<float>(pnh, "qx", 0);
    qy = get_ros_param<float>(pnh, "qy", 0);
    qz = get_ros_param<float>(pnh, "qz", 0);

    //TFの関係を作成する
    tf2_static_connection(parent_frame_id, child_frame_id, x, y, z, qx, qy, qz);

    ROS_INFO("New TF connection %s -> %s",parent_frame_id.c_str(), child_frame_id.c_str());
    ros::spin();
    return 0;
}

void tf2_static_connection(std::string parent_frame, std::string child_frame, float x, float y, float z, float qx, float qy, float qz){
  
  //ROSに配信するためのクラス
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  
  //配信内容を保存するメッセージ
  geometry_msgs::TransformStamped static_transformStamped;
  
  static_transformStamped.header.stamp = ros::Time::now();
  
  //構造をここに記述（親子）
  static_transformStamped.header.frame_id = parent_frame;
  static_transformStamped.child_frame_id = child_frame;
  
  //相対位置・姿勢：親から見た子の位置，親を原点としたときの子の位置
  static_transformStamped.transform.translation.x = x;
  static_transformStamped.transform.translation.y = y;
  static_transformStamped.transform.translation.z = z;
  tf2::Quaternion quat;
  quat.setRPY(qx,qy,qz);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
  //配信する
  static_broadcaster.sendTransform(static_transformStamped);

}