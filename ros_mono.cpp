/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <opencv2/opencv.hpp>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h> 
#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
using namespace cv;

ros::Publisher pub;  // Advertiser for 6df

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/cv_camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    pub = nodeHandler.advertise<std_msgs::Float64MultiArray>("slam_6dof",1,false); //initialize the advertiser

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


    // Edit by Diego Casabuena on 13/06/2017
    // Save all map points to a file 
    SLAM.SaveMapPointsToFile("MapPoints.txt");
     
    ros::shutdown();

    return 0;
}

// TODO: - Use from Euler Angles file
// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Mat rotationMatrixToEulerAngles(Mat &R)
{
 
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    cout << x << y << z << endl;
    Mat M(3,1,CV_64FC1,{0,0,0});
    M.at<double>(0,0) = x;
    M.at<double>(1,0) = y;
    M.at<double>(2,0) = z;
    cout << M <<endl;
    return M;   
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat pose = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    
    if(!pose.empty()){
        std_msgs::Float64MultiArray rt_package;
        rt_package.data.resize(7);

        // Image index
        rt_package.data[6] = msg->header.seq;
    
        /*
        // Rotation
        Mat rot;
        pose.rowRange(0,3).colRange(0,3).copyTo(rot);
        Mat angles = rotationMatrixToEulerAngles(rot);
        rt_package.data[0] = angles.at<double>(0,0);
        rt_package.data[1] = angles.at<double>(1,0);
        rt_package.data[2] = angles.at<double>(2,0);

	    //Translation
	    rt_package.data[3] = pose.at<float>(0,3)*1000;
        rt_package.data[4] = pose.at<float>(1,3)*1000;
        rt_package.data[5] = -pose.at<float>(2,3)*1000;
        //rt.at<double>(pose.at<float>(0,3),pose.at<float>(1,3),-pose.at<float>(2,3));
	    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
	    //const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
			//0, 0, 1,
			//1, 0, 0);
            */

        /* global left handed coordinate system */
        static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
        static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
        // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
        static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
                -1, 1,-1, 1,
                -1,-1, 1, 1,
                1, 1, 1, 1);

        //prev_pose * T = pose
        cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
        world_lh = world_lh * translation;
        pose_prev = pose.clone();


        // Rotation
        Mat rot;
        pose.rowRange(0,3).colRange(0,3).copyTo(rot);
        Mat angles = rotationMatrixToEulerAngles(rot);
        rt_package.data[0] = angles.at<double>(0,0);
        rt_package.data[1] = angles.at<double>(1,0);
        rt_package.data[2] = angles.at<double>(2,0);

	    //Translation
	    rt_package.data[3] = world_lh.at<float>(0,3)*1000;
        rt_package.data[4] = world_lh.at<float>(1,3)*1000;
        rt_package.data[5] = -world_lh.at<float>(2,3)*1000;

            cout << rt_package << endl;
            pub.publish(rt_package);       // Publish image matrix
        }

}


