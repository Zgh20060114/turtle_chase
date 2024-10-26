#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h" //四元数

std::string turtle_name;

//将乌龟在 turtlesim 中的位置信息转换为ROS 坐标
void BroadcasterPose(const turtlesim::Pose::ConstPtr &pose){
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped ts;

    ts.header.frame_id="world"; //让系统知道变换是相对于哪个坐标系进行的
    ts.header.stamp=ros::Time::now(); //使用当前时间戳可以确保变换信息的时效性，帮助系统在处理动态场景时保持一致性
    ts.child_frame_id=turtle_name; //标识具体的对象,使得其他节点能够正确识别和使用这个对象的坐标变换。

    //平移部分
    ts.transform.translation.x= pose->x;  //translation平移
    ts.transform.translation.y= pose->y;
    ts.transform.translation.z= 0;

    //旋转部分
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,pose->theta);  //使用setRPY函数设置四元数的滚转（Roll）、俯仰（Pitch）和偏航（Yaw）角度
    //将一个角度值转换为四元数，以便在ROS中进行坐标变换
    ts.transform.rotation.x=qtn.getX();
    ts.transform.rotation.y=qtn.getY();
    ts.transform.rotation.z=qtn.getZ();
    ts.transform.rotation.w=qtn.getW();

    broadcaster.sendTransform(ts);

    ROS_INFO("Turtle Name: %s", turtle_name.c_str());
    ROS_INFO("Position: (%.2f, %.2f, %.2f)", ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z);
    ROS_INFO("Orientation: (%.2f, %.2f, %.2f, %.2f)",
             ts.transform.rotation.x,
             ts.transform.rotation.y,
             ts.transform.rotation.z,
             ts.transform.rotation.w);
}

int main(int argc,char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"pub_turtle");
    ros::NodeHandle nh;

    if (argc!=2){
        ROS_ERROR("towtowtow!!");
        return 1;
    } else{
        turtle_name=argv[1];
    }
    //订阅,在回调函数里对订阅的消息进行处理，转换到TF坐标系里
    ros::Subscriber sub=nh.subscribe<turtlesim::Pose>(turtle_name+"/pose",1000,BroadcasterPose);
    ROS_INFO("dydydydydy");
    ros::spin();
    return 0;
}