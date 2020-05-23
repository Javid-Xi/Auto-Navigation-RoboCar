 /**
************************************************************
* @file         uart.c
* @brief        串口节点，订阅cmd_vel话题并发布odometry话题
* 				从cmd_vel话题中分解出速度值通过串口送到移动底盘
* 				从底盘串口接收里程消息整合到odometry话题用于发布
* @author       Javid
* @date         2020-05-14
* @version      1.0
*
***********************************************************/

#include <ros/ros.h>
#include <ros/time.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define	sBUFFERSIZE	9	//串口发送缓存长度
#define	rBUFFERSIZE	19	//串口接收缓存长度
#define wheel_radius 0.0408 //轮胎半径
#define wheel_a_mec 0.095   //麦克纳姆运动模型A参数    
#define wheel_b_mec 0.075  //麦克纳姆运动模型B参数
#define wheel_ab_mec wheel_a_mec+wheel_b_mec   //麦克纳姆运动模型A参数    
#define	PI	3.1415926535897932f

unsigned char s_buffer[sBUFFERSIZE];//发送缓存
unsigned char r_buffer[rBUFFERSIZE];//接收缓存

//里程计信息
double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;


//发送数据枚举类型
enum SendCommandType {
	SPEED_INFO = 0,
	PID_INFO,
}SendCommandType;


//联合体，用于浮点数与16进制的快速转换
typedef union{
	float fv;
	unsigned char cv[4];
}float_union;

//short与HEX快速获取
typedef	union {
    short sv;
	unsigned char cv[2];
} short_union;

//发送数据结构
typedef	struct {

    //电机编码器读数
    short_union Speed_A;
    short_union Speed_B;
    short_union Speed_C;
    short_union Speed_D;

    //偏航角
    short_union yaw;

} rcv_data;

serial::Serial ser;

/*************************************************
* Function: uart_data_send
* Description: 数据打包，将获取的cmd_vel信息打包并通过串口发送
* Parameter: cmd_vel
* Return: none
*************************************************/
void uart_data_send(const geometry_msgs::Twist& cmd_vel){
	//unsigned char i;
	short_union Vx,Vy,Ang_v;
	Vx.sv = cmd_vel.linear.x * 72;//乘以72(0.05*1440)转换为编码器速度
	Vy.sv = cmd_vel.linear.y * 72;
	Ang_v.sv = cmd_vel.angular.z * 72;
	
	memset(s_buffer,0,sizeof(s_buffer));
	//数据打包
	s_buffer[0] = 0xff;
	s_buffer[1] = 0xff;
	//Vx
	s_buffer[2] = Vx.cv[0];
	s_buffer[3] = Vx.cv[1];
	//Vy
	s_buffer[4] = Vy.cv[0];
	s_buffer[5] = Vy.cv[1];
	//Ang_v
	s_buffer[6] = Ang_v.cv[0];
	s_buffer[7] = Ang_v.cv[1];

	//CRC
	s_buffer[8] = s_buffer[2]^s_buffer[3]^s_buffer[4]^s_buffer[5]^s_buffer[6]^s_buffer[7];
	/*
	for(i=0;i<15;i++){
		ROS_INFO("0x%02x",s_buffer[i]);
	}
	*/
	ser.write(s_buffer,sBUFFERSIZE);
	
}
//接收数据分析与校验
unsigned char data_check(unsigned char *buffer)
{
	unsigned char ret=0,csum;
	//int i;
	if((buffer[0]==0xff) && (buffer[1]==0xaa)){
		csum = buffer[2]^buffer[3]^buffer[4]^buffer[5]^buffer[6]^buffer[7]^
				buffer[8]^buffer[9]^buffer[10]^buffer[11];
		//ROS_INFO("check sum:0x%02x",csum);
		if(csum == buffer[12]){
			ret = 1;//校验通过，数据包正确
		}
		else 
		  ret =0;//校验失败，丢弃数据包
	}
	/*
	for(i=0;i<rBUFFERSIZE;i++)
	  ROS_INFO("0x%02x",buffer[i]);
	*/
	return ret;

}

//订阅/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel){
	ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	
	uart_data_send(cmd_vel);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

	//订阅/turtle1/cmd_vel话题用于测试 $ rosrun turtlesim turtle_teleop_key
	ros::Subscriber write_sub = nh.subscribe("/turtle1/cmd_vel",50,cmd_vel_callback);
	//发布里程计话题 odom
	ros::Publisher read_pub = nh.advertise<nav_msgs::Odometry>("odom",50);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }
	//定义tf 对象
	static tf::TransformBroadcaster odom_broadcaster;
	//定义tf发布时需要的类型消息
	geometry_msgs::TransformStamped odom_trans;
	//定义里程计消息对象
	nav_msgs::Odometry odom;
	//定义四元数变量
	geometry_msgs::Quaternion odom_quat;
	//位置 速度 角速度
	rcv_data uart_rcv_data;
	//定义时间
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    //20hz频率执行
    ros::Rate loop_rate(20);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
			ser.read(r_buffer,rBUFFERSIZE);
			/*int i;
			for(i=0;i<rBUFFERSIZE;i++)
				ROS_INFO("[0x%02x]",r_buffer[i]);
			ROS_INFO_STREAM("End reading from serial port");
			*/
			if(data_check(r_buffer) != 0){
				for(i=0;i<2;i++){
					uart_rcv_data.Speed_A.cv[i] = r_buffer[2+i];
					uart_rcv_data.Speed_B.cv[i] = r_buffer[4+i];
					uart_rcv_data.Speed_C.cv[i] = r_buffer[6+i];
					uart_rcv_data.Speed_D.cv[i] = r_buffer[8+i];
					uart_rcv_data.yaw.cv[i] = r_buffer[10+i];
				}

				//编码器读数/1440 × 2 × PI /50ms = 单个轮子与地面的相对速度(m/s)
				vx = ( uart_rcv_data.Speed_A.sv * (-0.25) + uart_rcv_data.Speed_B.sv * (+0.25) + uart_rcv_data.Speed_C.sv * (-0.25) + uart_rcv_data.Speed_D.sv * (+0.25) ) * wheel_radius * PI  / 36 ;
				vy = ( uart_rcv_data.Speed_A.sv * (+0.25) + uart_rcv_data.Speed_B.sv * (+0.25) + uart_rcv_data.Speed_C.sv * (+0.25) + uart_rcv_data.Speed_D.sv * (+0.25) ) * wheel_radius *	PI  / 36 ;
				vth = ( uart_rcv_data.Speed_A.sv * (+0.25) + uart_rcv_data.Speed_B.sv * (-0.25) + uart_rcv_data.Speed_C.sv * (-0.25) + uart_rcv_data.Speed_D.sv * (+0.25) ) * wheel_radius * PI  / 36 / wheel_ab_mec ;

				//获取当前时间
				current_time = ros::Time::now();
				double dt = (current_time - last_time).toSec();
				double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
				double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
				double delta_th = vth * dt;

				x += delta_x;
				y += delta_y;
				th += delta_th;
					
				//将偏航角转换成四元数才能发布
				odom_quat = tf::createQuaternionMsgFromYaw(th);
				
				//发布坐标变换父子坐标系
				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_link";
				//填充获取的数据
				odom_trans.transform.translation.x = x;//x坐标
				odom_trans.transform.translation.y = y;//y坐标
				odom_trans.transform.translation.z = 0.0;//z坐标				
				odom_trans.transform.rotation = odom_quat;//四元数
				//发布tf坐标变换
				odom_broadcaster.sendTransform(odom_trans);

				//载入里程计时间戳
				odom.header.stamp = current_time;
				//里程计父子坐标系
				odom.header.frame_id = "odom";
				odom.child_frame_id = "base_link";
				//里程计位置数据
				odom.pose.pose.position.x = x;
				odom.pose.pose.position.y = y;
				odom.pose.pose.position.z = 0;
				odom.pose.pose.orientation = odom_quat;
				//载入线速度和角速度
				odom.twist.twist.linear.x = vx;
				odom.twist.twist.linear.y = vy;
				odom.twist.twist.angular.z = vth;
				//发布里程计消息
				read_pub.publish(odom);

				ROS_INFO("publish odometry");
				last_time = current_time;				
			}
			memset(r_buffer,0,rBUFFERSIZE);
        }
        loop_rate.sleep();

    }
}

