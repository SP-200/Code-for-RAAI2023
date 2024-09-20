// 实验结束后，控制平台移动，并发布stm57_tarVelMsg
#include "ros/ros.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Float32.h"
#include <unistd.h>
#define num_of_wheel 1
int main(int argc, char **argv) {
    ros::init(argc, argv, "integration_publisher");
    ros::NodeHandle n; // 
    ros::Publisher stm57_tarVelPub = n.advertise<std_msgs::Float32>("/stm_tar_vel", 1);

    ros::Rate loop_rate(1);
    std_msgs::Float32 stm57_tarVelMsg;

    float stm57_tarVel =108;//60=5mm
    float stop = 0.0;
    int count1 = 0;
    int time1 = 30;

    while (ros::ok()) 
    {   
        if (count1<time1+1){
            
            stm57_tarVelMsg.data = stm57_tarVel;

        }else if (count1<time1+2){

            stm57_tarVelMsg.data = stop;

        }else{

            ros::shutdown();
        }

        stm57_tarVelPub.publish(stm57_tarVelMsg);
        ros::spinOnce();    
        count1++;
        loop_rate.sleep();
    }
    return 0;
}