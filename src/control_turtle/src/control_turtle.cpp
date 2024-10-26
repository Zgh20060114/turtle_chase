#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"

double kp=10.0;
double ki=0.1;
double kd=1.0;

double last_error=0.0; //上次误差
double error=0.0; //比例项
double integral=0.0; //积分项
double derivative=0.0; //微分项


int main(int argc,char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "control_turtle");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    //发布对象
    ros::Publisher publisher =nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel",1000);

    ros::Rate rate(10);

    while (ros::ok()){
        try {

            geometry_msgs::TransformStamped tfs=buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
            double distance= sqrt(pow(tfs.transform.translation.x,2)+ pow(tfs.transform.translation.y,2));
            double angle= atan2(tfs.transform.translation.y,tfs.transform.translation.x);

            error=angle;
            integral+=error;
            derivative=error-last_error;

            double angle_z =kp*error+ki*integral+kd*derivative;


            last_error=error;
            geometry_msgs::Twist twist;
            twist.linear.x=distance;
            twist.angular.z=angle_z;
            publisher.publish(twist);
            //linear的PID



        }catch(const std::exception &e){
            ROS_ERROR("错误：%s",e.what());
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}