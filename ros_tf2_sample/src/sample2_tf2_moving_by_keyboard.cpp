/*
    sample1_tf2_moving_by_keyboard.cpp
    TFサンプル2用のソースコード
*/
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

void tf2_connection(std::string, std::string, float, float, float, float, float, float);

template<typename T>
T get_ros_param(ros::NodeHandle pnh, std::string arg_name, T default_val){
    T arg_value = default_val;
    if(pnh.hasParam(arg_name))
        pnh.getParam(arg_name, arg_value);
    return arg_value; 
}

//--- キーボードを入力する
struct termios cooked, raw;
int kfd=0;

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int getch(){
    static bool first_flag=true;
    if(first_flag){
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);                       
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
        first_flag=false;
    }
    char c;
    if(read(kfd, &c, 1) < 0){
        ROS_ERROR("error! get keyboard states");
        exit(-1);
    }
    return c;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "sample2_tf_connection");
    ros::NodeHandle pnh("~");

    signal(SIGINT,quit);

    //パラメータを取得する
    std::string parent_frame_id;
    std::string child_frame_id;
    parent_frame_id = get_ros_param<std::string>(pnh, "pframe", "map");
    child_frame_id = get_ros_param<std::string>(pnh, "cframe", "robot");
    
    //動的TFのための位置
    float x=0, y=0, add = 0.02f;

    ros::Rate loop_rate(30);

    //初期配信
    tf2_connection(parent_frame_id, child_frame_id, x, y, 0, 0, 0, 0);
    ROS_INFO("Push KeyBoard WASD");
    while (ros::ok()){
        //キーボード入力を受け取る
        switch (getch()){
            case 'w': y+=add; break;
            case 's': y-=add; break;
            case 'a': x-=add; break;
            case 'd': x+=add; break;
        }
        //配信
        tf2_connection(parent_frame_id, child_frame_id, x, y, 0, 0, 0, 0);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void tf2_connection(std::string parent_frame, std::string child_frame, float x, float y, float z, float qx, float qy, float qz){
  
  //ROSに配信するためのクラス
  static tf2_ros::TransformBroadcaster broadcaster;
  
  //配信内容を保存するメッセージ
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  
  //構造をここに記述（親子）
  transformStamped.header.frame_id = parent_frame;
  transformStamped.child_frame_id = child_frame;
  
  //相対位置・姿勢：親から見た子の位置，親を原点としたときの子の位置
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = z;
  tf2::Quaternion quat;
  quat.setRPY(qx,qy,qz);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();
  
  //配信する
  broadcaster.sendTransform(transformStamped);

}