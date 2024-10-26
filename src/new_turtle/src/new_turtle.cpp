#include "ros/ros.h"
#include "turtlesim/Spawn.h" //生成海龟

int main(int argc,char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"new_turtle");
    ros::NodeHandle nh;
    ros::ServiceClient client=nh.serviceClient<turtlesim::Spawn>("/spawn"); //创建一个服务客户端与Spawn服务通信，/spawn是turtlesim包中预定义的服务名称
    client.waitForExistence();
    turtlesim::Spawn spawn;

    spawn.request.name="turtle2";
    spawn.request.x=10;
    spawn.request.y=10;
    spawn.request.theta=45;

    bool flag=client.call(spawn);
    if (flag){
        ROS_INFO("OKOKOKOKOK");
    }
}