#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <algorithm>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>

using namespace cv;

std_msgs::Float32 msg_minDist;

void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    int len = range.size();
    int case_num = 1;
    float angle[len];
    float angle_temp;
    std::vector< int > avoid_range;
    std::vector< int > pole_wall;

    

    for(int i = 0; i < len-1; i++){
        angle_temp = angle_min + i*angle_increment;
        angle[i] = angle_temp;
        
        // // indexes where obstacles are within avoiding range
        // if(range[i] <= 0.5 && range[i] >= 0.2){
        //     avoid_range.push_back(i);
        // }
    }
    float min_range = *min_element(range.begin(), range.end());
    msg_minDist.data = min_range;

   

    for(int j=0; j < len-1; j++){
        std::cout << j << ":"<<range[j]<< std::endl;
    }
   

    // need to get rid of infinite / robot body indexes (if inf, set this to 100, then difference will be over 0.7)

    // if the ranges have difference of more than 0.7, we think of this as pole

    // we 

   
    waitKey(1);

    

    //minDist일때의 각도 추가

  
}

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_avoid_node2");
    ros::NodeHandle nh;

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
    ros::Publisher pub_lidar_tutorial = nh.advertise<std_msgs::Float32>("/minDist",1);

    ros::Rate loop_rate(5);

    while(ros::ok()){
        ros::spinOnce();
        pub_lidar_tutorial.publish(msg_minDist);
        loop_rate.sleep();
    }
    return 0;
}
