#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <algorithm>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <vector>
#include "string.h"

using namespace cv;

std_msgs::Float32 msg_minDist;

void lidar_cb(sensor_msgs::LaserScan msg)
{
  // angle in radian
  float angle_min = msg.angle_min;
  float angle_max = msg.angle_max;
  float angle_increment = msg.angle_increment;
  std::vector<float> range = msg.ranges;

  int len = range.size();
  int case_num = 1;
  float angle[len];
  float angle_temp;
  int i, pole_idx;
  std::vector<int> avoid_range;
  std::vector<int> pole_wall;
  std::vector<int> new_polewall;
  std::vector<int> last_polewall;
  std::vector<double> pole_x, pole_y;

  Mat plot_result;
  Ptr<plot::Plot2d> plot_xy;
  Mat x(Size(1, len), CV_64F, Scalar(0));
  Mat y(Size(1, len), CV_64F, Scalar(0));

  for (i = 0; i < len - 1; i++)
  {
    angle_temp = angle_min + i * angle_increment;
    angle[i] = angle_temp;
    if (std::isinf(range[i]) == false && range[i] * sin(angle_temp) != -0.000000)
    {
      x.at<double>(i) = range[i] * cos(angle_temp);
      y.at<double>(i) = -range[i] * sin(angle_temp);
    }
    // // indexes where obstacles are within avoiding range
    // if(range[i] <= 0.5 && range[i] >= 0.2){
    //     avoid_range.push_back(i);
    // }
  }
  float min_range = *min_element(range.begin(), range.end());
  msg_minDist.data = min_range;

  for (i = 0; i < len - 1; i++)
  {
    if (std::isinf(range[i]))
      range[i] = 4.2;
  }

  // search for poles in 359 angles (range[i])
  for (i = 0; i < len - 1; i++)
  {
    if (i != len - 2)
    {
      if ((range[i + 1] - range[i]) <= -0.7)
        pole_wall.push_back(i + 1);
      else if ((range[i + 1] - range[i]) >= 0.7)
        pole_wall.push_back(i);
      else
        ;
    }
    else
    {
      if ((range[len - 2] - range[0]) <= -0.7)
        pole_wall.push_back(len - 2);
      else if ((range[len - 2] - range[0]) >= 0.7)
        pole_wall.insert(pole_wall.begin(), 0);
      else
        ;
    }
  }

  std::cout << "pole_wall : ";
  for (int j = 0; j < pole_wall.size(); j++)
  {
    std::cout << pole_wall[j] << "\t";
  }
  std::cout << std::endl;
  int len1 = pole_wall.size();

  i = 0;
  while (i < len1 - 1)
  {
    if (i != 0)
    {
      if (range[pole_wall[i] - 1] > range[pole_wall[i]] && range[pole_wall[i + 1]] < range[pole_wall[i + 1] + 1])
      {
        new_polewall.push_back(pole_wall[i]);
        new_polewall.push_back(pole_wall[i + 1]);
        i += 2;
      }
      else
        i++;
    }
    else
    {  // first index of pole_wall[]
      if (pole_wall[i] == 0)
      {  // when the first index is 0 degrees
        // when pole_wall[len1-1] ~ pole_wall[0] is a pole
        if (range[pole_wall[len1 - 1] - 1] > range[pole_wall[len1 - 1]] &&
            range[pole_wall[i]] < range[pole_wall[i]] + 1)
        {
          new_polewall.push_back(pole_wall[len1 - 1]);
          new_polewall.push_back(pole_wall[0]);
          i++;
        }
        // when range[0] ~ range[pole_wall[i]+1] is a pole
        else if (range[len - 1] > range[0] && range[pole_wall[i + 1]] < range[pole_wall[i + 1] + 1])
        {
          new_polewall.push_back(pole_wall[i]);
          new_polewall.push_back(pole_wall[i + 1]);
          i += 2;
        }
        else
          i++;
      }
      else
      {  // when the first index is not 0 degrees
        if (range[pole_wall[len1 - 1] - 1] > range[pole_wall[len1 - 1]] &&
            range[pole_wall[i]] < range[pole_wall[i]] + 1)
        {
          new_polewall.push_back(pole_wall[len1 - 1]);
          new_polewall.push_back(pole_wall[0]);
          i++;
        }
        // when range[0] ~ range[pole_wall[i]+1] is a pole
        else if (range[pole_wall[i] - 1] > range[pole_wall[i]] && range[pole_wall[i + 1]] < range[pole_wall[i + 1] + 1])
        {
          new_polewall.push_back(pole_wall[i]);
          new_polewall.push_back(pole_wall[i + 1]);
          i += 2;
        }
        else
          i++;
      }
    }
  }
  std::cout << "new_polewall 1 : ";
  for (int j = 0; j < new_polewall.size(); j++)
  {
    std::cout << new_polewall[j] << "\t";
  }
  std::cout << std::endl;

  // Now let's remove elements that are too far from each other
  for (i = 0; i < new_polewall.size(); i += 2)
  {
    int del_angle = new_polewall[i + 1] - new_polewall[i];
    if (del_angle > 0 && del_angle <= 15)
    {
      last_polewall.push_back(new_polewall[i]);
      last_polewall.push_back(new_polewall[i + 1]);
    }
    else if (del_angle <= -345 && del_angle < 0)
    {
      last_polewall.push_back(new_polewall[i]);
      last_polewall.push_back(new_polewall[i + 1]);
    }
    else
      ;
  }
  std::cout << "last_polewall : ";
  for (int j = 0; j < last_polewall.size(); j++)
  {
    std::cout << last_polewall[j] << "\t";
  }
  std::cout << std::endl;

  // obtain pole's x, y coordinate
  for (i = 0; i < last_polewall.size(); i += 2)
  {
    if (i == 0 && last_polewall[i] > 340)
    {  // first idx is over 340
      pole_idx = (last_polewall[i + 1] - last_polewall[i] + 360) / 2 + last_polewall[i];

      if (pole_idx > 358)
        pole_idx -= 358;
      pole_x.push_back(range[pole_idx] * cos(pole_idx * M_PI / 180));
      pole_y.push_back(range[pole_idx] * sin(pole_idx * M_PI / 180));
    }
    else
    {
      pole_idx = (last_polewall[i] + last_polewall[i + 1]) / 2;
      pole_x.push_back(range[pole_idx] * cos(pole_idx * M_PI / 180));
      pole_y.push_back(range[pole_idx] * sin(pole_idx * M_PI / 180));
    }
    std::cout << "idx:" << pole_idx << std::endl;
  }

  std::cout << "pole coordinate" << std::endl;
  for (int j = 0; j < pole_x.size(); j++)
  {
    std::cout << "x: " << pole_x[j] << " y: " << pole_y[j] << std::endl;
  }

  // get (x, y) of each pole

  plot_xy = plot::createPlot2d(x, y);

  plot_xy->setMaxX(5);
  plot_xy->setMinX(-5);
  plot_xy->setMaxY(5);
  plot_xy->setMinY(-5);
  plot_xy->setPlotSize(720, 720);
  plot_xy->setNeedPlotLine(false);
  plot_xy->render(plot_result);

  imshow("plot", plot_result);
  waitKey(1);

  // minDist일때의 각도 추가

  plot_result.release();
  x.release();
  y.release();
  last_polewall.clear();
  new_polewall.clear();
  pole_wall.clear();
  range.clear();
  pole_x.clear();
  pole_y.clear();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_avoid_node1");
  ros::NodeHandle nh;

  ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
  ros::Publisher pub_lidar_tutorial = nh.advertise<std_msgs::Float32>("/minDist", 1);

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    ros::spinOnce();
    pub_lidar_tutorial.publish(msg_minDist);
    loop_rate.sleep();
  }
  return 0;
}
