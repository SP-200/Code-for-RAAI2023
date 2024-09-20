#include "ros/ros.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/Int8.h"

#include <signal.h>
#include "/home/roma/rover/rover_ws/src/rover_camera/devel/include/rover_camera/velocity.h"

//Keyboard listener
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include <termio.h>

using namespace dynamixel;

#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_GOAL_VELOCITY    104
#define ADDR_PROF_VELOCITY    112
#define ADD_OPERATING_MODE    11
#define ADDR_PRESENT_VELOCITY  128

#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3

#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"//Front
// #define DEVICE_NAME2          "/dev/ttyUSB0"//Back
#define PROTOCOL_VERSION      2.0

#define angle 0
#define targetVelocity 20
#define profileVelocity 10 //侧面速度

int32_t dxl_present_velocity = 0;
rover_camera::velocity velocity_with_stamp;
// int ELE_DATA[4]={624,0,669,1324};//升降台初始位置对应值
int SWEAR_DATA[4]={2820,4095,3593,0};//转向处初始位置对应值
// 总共运行11秒
// int targetPosition = 4092/2-int((angle)/360.0*4092);
// ros::Publisher controllerPub;

PortHandler * portHandler; //指针声明，星号表示 portHandler 是一个指向 PortHandler 类型对象的指针
PacketHandler * packetHandler;

// PortHandler * portHandler2;
// PacketHandler * packetHandler2;

int keyMode;
void* keyListener(void* args){
    // sleep(3);
    printf("press w to forward, s to back, a to turn left, d to turn right, q to quit, other to stop.\n");
    while(1){
        struct termios new_settings;
        struct termios stored_settings;
        tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
        new_settings = stored_settings;           //
        new_settings.c_lflag &= (~ICANON);        //
        new_settings.c_cc[VTIME] = 0;
        tcgetattr(STDIN_FILENO,&stored_settings); //获得stdin 输入
        new_settings.c_cc[VMIN] = 1;
        tcsetattr(STDIN_FILENO,TCSANOW,&new_settings); //

        keyMode = getchar();//返回键盘上按键对应的编码值；如w：119

        tcsetattr(STDIN_FILENO,TCSANOW,&stored_settings);
    }
}



int main(int argc,char **argv){
    ros::init(argc,argv,"demo");//初始化ros节点
    ros::NodeHandle n;//创建ros句柄
    // signal(SIGINT, signal_callback_handler);
    // controllerPub = n.advertise<std_msgs::Int8>("controller",1);

    
    // controllerMsg.data = 1;

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);// Get methods and members of PortHandler
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);// Get methods and members of PackageHandler

    // portHandler2 = PortHandler::getPortHandler(DEVICE_NAME2);
    // packetHandler2 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        ROS_ERROR("Failed to open the port!");
        return -1;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        ROS_ERROR("Failed to set the baudrate!");
        return -1;
    }

    // if (!portHandler2->openPort()) {
    //     ROS_ERROR("Failed to open the port!");
    //     return -1;
    // }

    // if (!portHandler2->setBaudRate(BAUDRATE)) {
    //     ROS_ERROR("Failed to set the baudrate!");
    //     return -1;
    // }

    // 设置模式；
    //升降台设为位置控制
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2,  ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 3, ADD_OPERATING_MODE, POSITION_CONTROL_MODE,  &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 4, ADD_OPERATING_MODE, POSITION_CONTROL_MODE,  &dxl_error);
    // //转向处设为位置控制
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5,  ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6,  ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7,  ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8,  ADD_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    //轮处设为速度控制
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 9, ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 10,ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE,  &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 11, ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE,  &dxl_error);
   // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 12,ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);


    
    // // 使能电机
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 3, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 4, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 9, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 10, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 11, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 12, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    



    // //设置升降台初始值
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 1, ADDR_GOAL_POSITION, ELE_DATA[0], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 2, ADDR_GOAL_POSITION, ELE_DATA[1], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 3, ADDR_GOAL_POSITION,ELE_DATA[2], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 4, ADDR_GOAL_POSITION, ELE_DATA[3], &dxl_error);

    // //设置转向处初始值
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 5, ADDR_PROF_VELOCITY, 8, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 6, ADDR_PROF_VELOCITY, 8, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 7, ADDR_PROF_VELOCITY,8, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 8, ADDR_PROF_VELOCITY, 8, &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 5, ADDR_GOAL_POSITION, SWEAR_DATA[0], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 6, ADDR_GOAL_POSITION, SWEAR_DATA[1], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 7, ADDR_GOAL_POSITION,SWEAR_DATA[2], &dxl_error);
    // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 8, ADDR_GOAL_POSITION, SWEAR_DATA[3], &dxl_error);

    //回正后将转向台改为速度控制
    // sleep(7);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5, ADDR_TORQUE_ENABLE, 0, &dxl_error);//&：获取地址
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5,  ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6,  ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7,  ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8,  ADD_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  
    //以上都是初始化操作
    printf("Initialize Done\n");
    //开启键盘监听
    pthread_t key_thread;
    int ret = pthread_create(&key_thread, NULL, keyListener, 0);
    uint32_t velo=0,swerve=0;
    uint32_t velo2=0;
    ros::Publisher pub = n.advertise<rover_camera::velocity>("present_velocity",10);
    
    while(true)
    {
        
        bool flag=true;
        switch (keyMode)
        {
        case 119://w means move forward
            velo=12;
            velo2=24; // 12, 13, 15, 17, 20, 24 对应滑移率 0.0～0.5
            swerve=0;
            break;
        case 115://s means move back
            velo=-5;
            velo2=-5;
            swerve=0;
            break;
        case 97://a means turn left
            velo=0;
            velo2=0;
            swerve=-5;
            break;
        case 100://d means turn right
            velo=0;
            velo2=0;
            swerve=5;
            break;
        case 113:// q means quit
            flag=false;
            break;
        default:
            velo=0;
            velo2=0;
            swerve=0;
            break;
        }
        if(!flag)break;
        //根据键盘输入设定各个轮 goal_velocity
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 9, ADDR_GOAL_VELOCITY, velo2, &dxl_error);
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 10, ADDR_GOAL_VELOCITY, velo, &dxl_error);
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 11, ADDR_GOAL_VELOCITY,velo, &dxl_error);
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler2, 12, ADDR_GOAL_VELOCITY, velo, &dxl_error);
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 5, ADDR_GOAL_VELOCITY, swerve, &dxl_error);
        // dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 6, ADDR_GOAL_VELOCITY, swerve, &dxl_error);
        ros::Time vel_time = ros::Time::now();
        //读取左前轮 present_velocity
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler,9, ADDR_PRESENT_VELOCITY, (uint32_t*)&dxl_present_velocity, &dxl_error);
        // printf("Present velocity: %03d\n",dxl_present_velocity);

        velocity_with_stamp.header.stamp = vel_time;  
        velocity_with_stamp.present_velocity = dxl_present_velocity;
        pub.publish(velocity_with_stamp);

    }

    // //结束并关闭电机
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 2, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 3, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 4, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 5, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 6, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 7, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 8, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 9, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, 10, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 11, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // dxl_comm_result = packetHandler->write1ByteTxRx(portHandler2, 12, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    // exit(0);
    printf("\nEXIT and TURN off SERVORS\n");
    return 0;
}