//
//  eulerAngles.cpp
//  
//
//  Created by Diego Fernando Lorenzo-Casabuena González on 18/06/2017.
//
//

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h> 
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

using namespace cv;
using namespace std;



// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    
    cout << "--------------------------" << endl;
    cout << "R: " << endl << R << endl;
    cout << "shouldBeIdentityL: " << endl << shouldBeIdentity << endl;
    cout << "--------------------------" << endl;

    return  norm(I, shouldBeIdentity) < 1e-6;
     
}


Mat eulerAnglesToRotationMatrix(Vec3f &theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;

    //cout << "-----------------------" << endl;
    //cout << R << endl;
    //cout << "-----------------------" << endl;
    assert(isRotationMatrix(R));


    return R;
 
}




// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Mat rotationMatrixToEulerAngles(Mat &R)
{
    //cout << "-----------------------" << endl;
    //cout << R << endl;
    //cout << "-----------------------" << endl;
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; 
 
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
    //cout << x << y << z << endl;
    Mat M(3,1,CV_64FC1,{0,0,0});
    M.at<double>(0,0) = x;
    M.at<double>(1,0) = y;
    M.at<double>(2,0) = z;
    //cout << M <<endl;
    return M;   
}