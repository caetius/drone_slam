/**
*  chessCalib.cpp
*  Diego Lorenzo Casabuena, 06/06/2017
*/


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h> 
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using std::vector;
using cv::VideoCapture;
using cv::Mat;
using cv::Size;
using cv::Point2f;
using cv::Point3f;
using cv::FileStorage;
using cv::Affine3d;
using cv::Vec3f;
using std::cout;
using std::endl;

#define H (4)
#define W (4)


// TODO: - Store this file in the right directory
const char* cameraCalibData = "/home/diego/cam_calib.yaml"; // File with camera calibration data
const Size chessboardSize{9,6}; // 9x6 chessboard inner corners
const float chessSquareSize = 7.4; // Using chessboard of same size as they used.
Mat cameraMatrix;
vector<Point3f> initialPoints;
ros::Publisher pub;  // Advertiser for 6df
//ros::Publisher pub2;  // Advertiser for 6df

int loadCalibData(){

    // Read camera matrix from config file: Calibration is assumed across all program
    FileStorage fs(cameraCalibData, FileStorage::READ);
    if (!fs.isOpened()) {   // Could not open camera calibration file
        std::cerr << "Couldn't open " << cameraCalibData << std::endl;
        return -1;
    }
    fs["cameraMatrix"] >> cameraMatrix;
    
    // Calculate the chessboard's initial points given size and lengths
    for (int z = 0; z < chessboardSize.height; z++)	{
        for(int x = 0; x < chessboardSize.width; x++) {
            initialPoints.push_back(Point3f{x * chessSquareSize, 0, z * chessSquareSize});
        }
    }
    return 1;
}


// Get average point given a set of points
Point3f centroid(const vector<Point3f>& points)
{
    Vec3f sum;
    for (Point3f p : points) {
        sum += Vec3f{p};
    }
    int count = points.size();
    return sum / count;
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

// True if chessboard is found, false otherwise
bool findChessboards(const Mat image, Size chessboardSize, vector<Point2f>& point)
{
    bool found = cv::findChessboardCorners(
                                            image,
                                            chessboardSize,
                                            point,
                                            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);
    if (!found) {
        return false;
    }
    return true;
}


// Extract 6df from a given frame
Mat poseFromChess(Mat frame) {

    vector<Point2f> imagePoints;
    Mat rvec;
    Mat tvec;

    cv::imshow("view", frame);
    cv::waitKey(30);

    // Camera matrix and initial points already declared 
    if(!findChessboards(frame, chessboardSize, imagePoints)){      // Find chess 2D points
        std::cerr << "Chessboard corners not found." << std::endl; 
        return Mat();  // Empty matrix will lead to end of execution in image callback
    }

    // Solve pnp
    bool found = cv::solvePnP(initialPoints,imagePoints, cameraMatrix, Mat{}, rvec, tvec);
    if(!found){     // Chessboard not found
        std::cerr << "Couldn't calibrate camera with chessboard" << std::endl;
        return Mat();
    }

    Affine3d cameraTransform = Affine3d{rvec, tvec};
	// The chessboard doesn't move, we are moving.
    cameraTransform = cameraTransform.inv();
    Mat rotM(3,3,CV_64FC1);
    rotM = Mat{cameraTransform.rotation()};
    cv::Vec3d translation = cameraTransform.translation();
    Mat new_tvec = Mat{3, 1, CV_64F};
    // TODO: - Remember to check axis
    new_tvec.at<double>(0, 0) = translation[0];
    new_tvec.at<double>(1, 0) = -translation[2];
    new_tvec.at<double>(2, 0) = translation[1];
   // cout << "tvec: " << cameraTransform.translation() << endl;
    
    //cv::Rodrigues(rvec,rotM);
    //cout << "rot M: " << rotM << endl;
    Mat angles = rotationMatrixToEulerAngles(rotM);
    //cout << "angles1: " << angles << endl;
    cv::hconcat(angles,new_tvec,angles);
    cout << "matrix: " << angles << endl;
    return angles;
}
#include "std_msgs/String.h"


/**
   *  Called when frames are received. 
   *  Each call advertises a translation matrix specifying 6df. Assume images are retrieved from   
   *  subscription.
   */
void imageCallback(const sensor_msgs::ImageConstPtr& Img_msg)
{ 
    //cout << msg.header << endl;
  try
  {
    Mat frame = cv_bridge::toCvShare(Img_msg, "")->image;
    Mat rtMatrix = poseFromChess(frame); // Get pose
    if (rtMatrix.rows == 0)
        return;
    cout << rtMatrix << endl;
    std_msgs::Float64MultiArray msg;
    msg.data.resize(rtMatrix.rows*rtMatrix.cols + 1);
    for (int j = 0; j < rtMatrix.cols;j++){
        for (int i = 0; i < rtMatrix.rows;i++){
            msg.data[i+rtMatrix.rows*j] = rtMatrix.at<double>(i,j);
        }
    }
    msg.data[6] = Img_msg->header.seq;
    //ros::Rate loop_rate(5);
    cout << msg << endl;
    pub.publish(msg);       // Publish image matrix
  //  pub2.publish(msg);
    ros::spinOnce();
    //loop_rate.sleep();        
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", Img_msg->encoding.c_str());
  }
}



int main(int argc, char **argv)
{

   // Get camera matrix and chessboard points matrix
  loadCalibData();
   /**
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "chessCalib");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   */
  ros::NodeHandle n;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(n);
  // Decode and save image
  pub = n.advertise<std_msgs::Float64MultiArray>("chess_6dof",1,false); //initialize the advertiser
  //pub2 = n.advertise<std_msgs::Float64MultiArray>("slam_6dof",1,false); //initialize the advertiser
  
  image_transport::Subscriber sub = it.subscribe("/cv_camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");

  return 0;
}
