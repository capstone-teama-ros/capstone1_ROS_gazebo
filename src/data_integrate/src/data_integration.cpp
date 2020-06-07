#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <boost/thread.hpp>


#include <ros/ros.h>
#include <ros/package.h>
#include "core_msgs/ball_position.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"


#include "opencv2/opencv.hpp"


#define RAD2DEG(x) ((x)*180./M_PI)

boost::mutex map_mutex;

int lidar_size;
float lidar_degree[400];
float lidar_distance[400];
float lidar_obs;

int ball_number;
float ball_X[20];
float ball_Y[20];
float ball_distance[20];
int near_ball;

int action;

int len;
int n;

#define RAD2DEG(x) ((x)*180./M_PI)

void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  map_mutex.lock();

  int count = scan->angle_max / scan->angle_increment;
  lidar_size = count;
  for (int i = 0; i < count; i++)
  {
    lidar_degree[i] = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    lidar_distance[i] = scan->ranges[i];
  }
  map_mutex.unlock();
}

void camera_Callback(const core_msgs::ball_position::ConstPtr& position)
{

    int count = position->size;
    ball_number=count;
    for(int i = 0; i < count; i++)
    {
        ball_X[i] = position->img_x[i];
        ball_Y[i] = position->img_y[i];
        // std::cout << "degree : "<< ball_degree[i];
        // std::cout << "   distance : "<< ball_distance[i]<<std::endl;
		ball_distance[i] = ball_X[i]*ball_X[i]+ball_Y[i]*ball_X[i];
    }

}
class WheelController
{
public:
  WheelController(const ros::Publisher& fl_wheel, const ros::Publisher& fr_wheel)
    : fl_wheel_(fl_wheel), fr_wheel_(fr_wheel)
  {
  }

  /**
   * 로봇의 선속도와 각속도를 설정한다.
   *
   * @param linear_speed 선속도. 전진은 +, 후진은 -, 정지는 0.
   *    단위는 m/s
   * @param angular_speed 각속도. 반시계방향(좌회전)은 +, 시계방향(우회전)은 -, 직진/정지는 0.
   *    단위는 rad/s
   */
  void setSpeed(double linear_speed, double angular_speed)
  {
    linear_speed_ = linear_speed;
    angular_speed_ = angular_speed;

    // turtlebot의 바퀴 반지름은 0.033m이다.
    // TODO: 나중에 우리 로봇의 바퀴 크기에 맞춰 비례상수를 다시 계산해야 한다.
    const double LINEAR_SPEED_FACTOR = 50;
    // TODO: 로봇의 각속도를 바퀴의 회전속도로 변환하는 비례상수는 지금은 알 수 없다.
    // 일단 1로 놓고 나중에 제대로 구해보자.
    const double ANGULAR_SPEED_FACTOR = 1.0;

    setWheelSpeeds(linear_speed_ * LINEAR_SPEED_FACTOR - angular_speed_ * ANGULAR_SPEED_FACTOR,
                   linear_speed_ * LINEAR_SPEED_FACTOR + angular_speed_ * ANGULAR_SPEED_FACTOR);
  }

  /**
   * 로봇의 선속도를 설정한다. 각속도는 기존의 값을 유지한다.
   *
   * @param linear_speed 선속도. 전진은 +, 후진은 -, 정지는 0.
   *    단위는 m/s
   */
  void setLinearSpeed(double linear_speed)
  {
    setSpeed(linear_speed, angular_speed_);
  }

  /**
   * 로봇의 각속도를 설정한다. 선속도는 기존의 값을 유지한다.
   *
   * @param angular_speed 각속도. 반시계방향(좌회전)은 +, 시계방향(우회전)은 -, 직진/정지는 0.
   *    단위는 rad/s
   */
  void setAngularSpeed(double angular_speed)
  {
    setSpeed(linear_speed_, angular_speed);
  }

private:
  const ros::Publisher& fl_wheel_;
  const ros::Publisher& fr_wheel_;


  double linear_speed_ = 10.0;
  double angular_speed_ = 0;

  /**
   * 4개의 바퀴에 각각의 속도를 지정한다.
   */
  void setWheelSpeeds(double fl_speed, double fr_speed) const
  {
    std_msgs::Float64 fl_wheel_msg;
    std_msgs::Float64 fr_wheel_msg;


    fl_wheel_msg.data = fl_speed;
    fr_wheel_msg.data = fr_speed;


    fl_wheel_.publish(fl_wheel_msg);
    fr_wheel_.publish(fr_wheel_msg);

  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_integation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = n.subscribe<core_msgs::ball_position>("/position", 1000, camera_Callback);
    ros::Publisher fl_wheel = n.advertise<std_msgs::Float64>("/myrobot/FLwheel_velocity_controller/command", 10);
    ros::Publisher fr_wheel = n.advertise<std_msgs::Float64>("/myrobot/FRwheel_velocity_controller/command", 10);
	ros::Publisher fl_publish= n.advertise<std_msgs::Float64>("myrobot/FLsuspension_position_controller/command", 10);
    ros::Publisher fr_publish = n.advertise<std_msgs::Float64>("/myrobot/FRsuspension_position_controller/command", 10);
    ros::Publisher cs_publish = n.advertise<std_msgs::Float64>("/myrobot/CSsuspension_position_controller/command", 10);


    WheelController wheelController(fl_wheel, fr_wheel);

    while(ros::ok){
		std_msgs::Float64 FL_position_msg;
		std_msgs::Float64 FR_position_msg;
		std_msgs::Float64 CS_position_msg;
		
		FL_position_msg.data = 0.05;
		FR_position_msg.data = 0.05;
		CS_position_msg.data = 0.05;
			
			double linear_speed = 50;
      double angular_speed = 0;
	fl_publish.publish(FL_position_msg);
	fr_publish.publish(FR_position_msg);
	cs_publish.publish(CS_position_msg);
      wheelController.setSpeed(linear_speed, angular_speed);
	  std::cout << "moving" << std::endl;
			

		  


	    ros::Duration(0.025).sleep();
	    ros::spinOnce();
    }

    return 0;
}
