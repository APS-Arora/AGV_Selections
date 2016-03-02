#include "opencv2/imgproc/imgproc.hpp"
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <queue>
#include <stack>
#include <list>
#include "ros/ros.h"
#include "opencv_apps/Point2D.h"
#include "std_msgs/Empty.h"

#define Pixel(M,x,y,c) *(M.data+M.step[0]*x+M.step[1]*y+c)

using namespace cv;
using namespace std;

list<Point> BFS(Point st, Point end, Mat img);
list<Point> DFS(Point st, Point end, Mat img);

int main(int argc, char **argv)
{
  Mat img = imread("PS1.png");
	Mat cimg;
	vector<Vec3f> circles;
	Point Start, End;
	list<Point> BFS_Plots,DFS_Plots;
  	ros::init(argc, argv, "thinker");
  	ros::NodeHandle n;
	ros::Rate loop_rate(50);
	namedWindow("AGV");
	cvtColor(img, cimg, CV_BGR2GRAY);
	for (int i = 0; i < cimg.size[0]; i++)
	{
		for (int j = 0; j < img.size[1]; j++)
		{
			if ((Pixel(img, i, j, 0)==36 && Pixel(img, i, j, 1)==28 && Pixel(img, i, j, 2)==237) || (Pixel(img, i, j, 0) == 0 && Pixel(img, i, j, 1) == 255 && Pixel(img, i, j, 2) == 0))
				Pixel(cimg, i, j, 0) = 0;
			else
				Pixel(cimg, i, j, 0) = 255;
		}
	}
	HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, 10, 200, 15, 0, 0);
	if (Pixel(img, cvRound(circles[0][0]), cvRound(circles[0][1]), 2) == 255)
	{
		End=Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
		Start = Point(cvRound(circles[1][0]), cvRound(circles[1][1]));
	}
	else
	{
		Start = Point(cvRound(circles[0][0]), cvRound(circles[0][1]));
		End = Point(cvRound(circles[1][0]), cvRound(circles[1][1]));
	}
	for (int i = 0; i < cimg.size[0]; i++)
	{
		for (int j = 0; j < img.size[1]; j++)
		{
			if (Pixel(img, i, j, 0) == 255 && Pixel(img, i, j, 1) == 255 && Pixel(img, i, j, 2) == 255)
			{
				Pixel(cimg, i, j, 0) = 0;
			}
			else
				Pixel(cimg, i, j, 0) = 255;
		}
	}
	cimg.at<uchar>(Start) = 128;
	BFS_Plots=BFS(Start, End, cimg.clone());
	DFS_Plots=DFS(Start, End, cimg);
	ros::Publisher plot_pub = n.advertise<opencv_apps::Point2D>("plot", 1000);
	ros::Publisher msg_pub = n.advertise<std_msgs::Empty>("intercom",1000);

	opencv_apps::Point2D toPlot;
	list<Point>::iterator it;
	while(ros::ok())
	  {
	    ROS_INFO("%s", "Publishing BFS Plots");
	    msg_pub.publish(std_msgs::Empty());
	    loop_rate.sleep();
	    for(it=BFS_Plots.begin();ros::ok()&&it!=BFS_Plots.end();it++)
	      {
		toPlot.x=it->x;
		toPlot.y=it->y;
		plot_pub.publish(toPlot);
		loop_rate.sleep();
	      }
	    ROS_INFO("Publishing DFS Plots");
	    msg_pub.publish(std_msgs::Empty());
	    loop_rate.sleep();
	    for(it=DFS_Plots.begin();ros::ok()&&it!=DFS_Plots.end();it++)
	      {
		toPlot.x=it->x;
		toPlot.y=it->y;
		plot_pub.publish(toPlot);
		loop_rate.sleep();
	      }
	  }	
	return(0);	
}

list<Point> BFS(Point st, Point end, Mat img)
{
	int a[3] = { img.size[0], img.size[1], 2 };
	Mat parents(3,a, CV_8U);
	Point curr,leaf;
	queue<Point> Check;
	Check.push(st);
	while(leaf!=end)
	{
			curr = Check.front();
			Check.pop();
		for (int i = 1; i > -2; i--)
			for (int j = 1; j > -2; j--)
			{
				Point temp = curr + Point(j, i);
				if (temp.x >= img.size[1] || temp.y >= img.size[0]|| temp.x<0 || temp.y<0)
					continue;
				if (img.at<uchar>(temp) < 129)
					continue;
				else if (temp == end)
				{
					leaf = temp;
					Pixel(parents, leaf.y, leaf.x, 0) = curr.x;
					Pixel(parents, leaf.y, leaf.x, 1) = curr.y;
				}
				else
				{
					Check.push(temp);
					Pixel(parents, temp.y, temp.x, 0) = curr.x;
					Pixel(parents, temp.y, temp.x, 1) = curr.y;
					img.at<uchar>(temp) = 128;
				}
			}
	}
	list<Point> path;
	curr = leaf;
	path.push_front(leaf);
	int temp;
	while (curr!=st)
	{
		temp = curr.x;
		curr.x = Pixel(parents, curr.y, curr.x, 0);
		curr.y = Pixel(parents, curr.y, temp, 1);
		path.push_front(curr);
	}
	return(path);
}

list<Point> DFS(Point st, Point end, Mat img)
{
	int a[3] = { img.size[0], img.size[1], 2 };
	Mat parents(3, a, CV_8U);
	Point curr, leaf;
	stack<Point> Check;
	Check.push(st);
	while (leaf != end)
	{
		curr = Check.top();
		Check.pop();
		for (int i = -1; i <2; i++)
			for (int j =-1; j <2; j++)
			{
				Point temp = curr + Point(j, i);
				if (temp.x >= img.size[1] || temp.y >= img.size[0] || temp.x<0 || temp.y<0)
					continue;
				if (img.at<uchar>(temp) < 129)
					continue;
				else if (temp == end)
				{
					leaf = temp;
					Pixel(parents, leaf.y, leaf.x, 0) = curr.x;
					Pixel(parents, leaf.y, leaf.x, 1) = curr.y;
				}
				else
				{
					Check.push(temp);
					Pixel(parents, temp.y, temp.x, 0) = curr.x;
					Pixel(parents, temp.y, temp.x, 1) = curr.y;
					img.at<uchar>(temp) = 128;
				}
			}
	}
	list<Point> path;
	curr = leaf;
	path.push_front(leaf);
	int temp;
	while (curr != st)
	{
		temp = curr.x;
		curr.x = Pixel(parents, curr.y, curr.x, 0);
		curr.y = Pixel(parents, curr.y, temp, 1);
		path.push_front(curr);
	}
	return(path);
}
