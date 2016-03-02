#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include "opencv_apps/Point2D.h"
#include "std_msgs/Empty.h"
#include <string>
#include <cstdio>

#define Pixel(M,x,y,c) *(M.data+M.step[0]*x+M.step[1]*y+c)

using namespace cv;
using namespace std;

Mat img=imread("PS1.png");
Mat cpy;
bool task_st=false;

void Plot(const opencv_apps::Point2D::ConstPtr& Q);
void Refresh(const std_msgs::Empty::ConstPtr& R);


void Plot(const opencv_apps::Point2D::ConstPtr& Q)
{
  if(task_st)
    {
	    Pixel(cpy, cvRound(Q->y), cvRound(Q->x), 0) = 255;
	    Pixel(cpy, cvRound(Q->y), cvRound(Q->x), 1) = 0;
	    Pixel(cpy, cvRound(Q->y), cvRound(Q->x), 2) = 0;
    }
}

void Refresh(const std_msgs::Empty::ConstPtr& R)
{
  static int count=0;
  char str[50];
  sprintf(str,"Path %d.png",count);
  if(count==1||count==2)
    imwrite(str,cpy);
  cpy=img.clone();
  if(count<2)
    task_st=true;
  else
    task_st=false;
  count++;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"plotter");
  ros::NodeHandle n;
  ros::Subscriber plot_sub=n.subscribe("plot",10000,Plot);
  ros::Subscriber msg_sub=n.subscribe("intercom",10,Refresh);
  ros::spin();
  return(0);
}




