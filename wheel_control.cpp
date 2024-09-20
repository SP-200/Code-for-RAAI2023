// 跑实验用，控制舵机，并发布stm57_tarVelMsg
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include <unistd.h>
#define num_of_wheel 1

#include <stdlib.h>
#include <stdio.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>  // Uses DYNAMIXEL SDK library

/********* DYNAMIXEL Model definition *********
***** (Use only one definition at a time) *****/
#define X_SERIES // X330, X430, X540, 2X430
// #define PRO_SERIES // H54, H42, M54, M42, L54, L42
// #define PRO_A_SERIES // PRO series with (A) firmware update.
// #define P_SERIES  // PH54, PH42, PM54
// #define XL320  // [WARNING] Operating Voltage : 7.4V
// #define MX_SERIES // MX series with 2.0 firmware update.

// Control table address
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_GOAL_VELO          104
#define ADDR_PRO_OPERATION_MODE          11
#define ADDR_PRESENT_POSITION       132
#define MINIMUM_POSITION_LIMIT      0  // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT      4095  // Refer to the Maximum Position Limit of product eManual
#define BAUDRATE                    1000000


// DYNAMIXEL Protocol Version (1.0 / 2.0)
// https://emanual.robotis.com/docs/en/dxl/protocol2/
#define PROTOCOL_VERSION  2.0

// Factory default ID of all DYNAMIXEL is 1
#define DXL_ID  7

// Use the actual port assigned to the U2D2.
// ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
#define DEVICENAME  "/dev/ttyUSB2"

#define TORQUE_ENABLE                   1
#define TORQUE_DISABLE                  0
#define DXL_MOVING_STATUS_THRESHOLD     20  // DYNAMIXEL moving status threshold
#define ESC_ASCII_VALUE                 0x1b


#include <stdio.h>
#include <stdlib.h>
#include "NiMotionMotorSDK.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "signal.h"

#define velLimt 600

int nAddr = 1;
int addrs[20];
int nCount = 0;
int rc = 0;
int nPos;
bool isStable=false;
float tarF=20.0;
int K=0;
float currentF=0;

std_msgs::Float32 stm_pos_msg;

void force_callback(const std_msgs::Float32MultiArray msg){

    // TODO rev/min到寄存器值的转换
    // ROS_INFO("111111");
    double FN = msg.data[0];
    currentF=FN;
    printf("FN=%f\t",FN);
    printf("K=%d\t",K);
    if(isStable)
    printf("True\n");
    else
    printf("false\n");
    if (FN>=tarF)
	{
		K++;
		if(K>=100)
			isStable=true;
	}
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "integration_publisher");
    ros::NodeHandle n; // 
    ros::Publisher stm57_tarVelPub = n.advertise<std_msgs::Float32>("/stm_tar_vel", 1);
    ros::Subscriber force_sub = n.subscribe("/force_sensor/data",10,force_callback); //每订阅到一次力传感器消息，调用一次回调函数
    ros::Rate loop_rate(1);
    std_msgs::Float32 stm57_tarVelMsg; //？？


    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	int index = 0;
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result

	uint8_t dxl_error = 0;                          // DYNAMIXEL error

	// Open port
	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");
	}
	else {
		printf("Failed to open the port!\n");
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	}
	else {
		printf("Failed to change the baudrate!\n");	
		return 0;
	}

	// Enable DYNAMIXEL Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
	}
	else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
	}
	else {
		printf("Succeeded enabling DYNAMIXEL Torque.\n");
	}

	int GOAL_VELO=-5;//舵机速度 -3=5.4mm/s（角速度*半径）

    float stm57_tarVel = -108;//电机速度 60=5mm/s

    float sinkage_truth;
    float rw = GOAL_VELO*0.6*5.4;
    float v = -stm57_tarVel/108*60*5;

    if(rw>=v){
        sinkage_truth = 1- v/rw;
    }
    else
    {
        sinkage_truth = rw/v-1;
    }
    printf("Sinkage Truth: %f",sinkage_truth)
    float stop = 0.0;
    int count1 = 0;
    int time1 = 25; //限定运行时间为25s
    while(!isStable) //阻塞程序，直到系统达到稳态
    {
        // printf("FN=%f\t",currentF);
        // printf("K=%d\t",K);
        // if(isStable)
        // printf("True\n");
        // else
        // printf("false\n");
        ros::spinOnce(); //若不稳定，则不断调用回调函数直到稳定，稳定后跳出循环
    }
    while (ros::ok()) 
    {   
        // printf("current_Force:%f:\n",currentF);
        if(count1==0) //刚开始运动，赋goal_velocity值
        {   
            //写入goal_velocity
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELO, GOAL_VELO, &dxl_error);
            //异常显示
		    if (dxl_comm_result != COMM_SUCCESS) {
			    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		    }
		    else if (dxl_error != 0) {
			    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		    }
        }
        if (count1<time1+1){ //如果为true，则跳过else if语句
            
            stm57_tarVelMsg.data = stm57_tarVel; //持续运行

        }else if (count1<time1+2){ //在规定时间26s停止

            stm57_tarVelMsg.data = stop;
            dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELO, 0, &dxl_error);//停止舵机
		    if (dxl_comm_result != COMM_SUCCESS) {
			    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		    }
		    else if (dxl_error != 0) {
			    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		    }
            //disable力矩
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

        }else{ //在27s时关闭ros节点

            ros::shutdown();
        }

        stm57_tarVelPub.publish(stm57_tarVelMsg);
        ros::spinOnce();    
        count1++;
        loop_rate.sleep(); //每1s循环一次
    }
    return 0;
}