//
//  SLAM_transform.cpp
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
#include <stack>
#include <iostream>

using namespace cv;
using namespace std;

// Define publisher
ros::Publisher pub;

//
// VARIABLES TO ADJUST
//
//
const int min_points = 100;
const int max_points = 10;
double threshold_ = 0;
double ransac = 1;
double confidence = .99;

// Temp arrays
vector<vector<double> > temp_slam_trans;
vector<vector<double> > temp_chess_trans;

// Multi-point structures
Mat chess_translation;
Mat slam_translation;

// Boolean value that determines when to stop collecting r|t data points and start broadcasting
// transformed slam 6dof.
bool start_broadcast = false;
// Mats to store the final matrices calculated
Mat trans_M; 
Mat rot_M;  
bool matching = false;


//method declare
Mat eulerAnglesToRotationMatrix(Vec3f &theta);
bool isRotationMatrix(Mat R);
Mat rotationMatrixToEulerAngles(Mat &R);



Mat findMatrixOfTransformation(Mat lhs, Mat rhs){
    
    Mat M;
    Mat inliers;
    rhs.rowRange(0,3).copyTo(rhs);
    lhs.rowRange(0,3).copyTo(lhs);
     cout << "chess matrix : " << lhs << endl << endl;
    cout << "slam matrix: " << rhs << endl << endl;
    estimateAffine3D(rhs.t(),lhs.t(),M,inliers,ransac,confidence);
    cout << "inliers at : " << inliers.size() << endl << endl;
    
    //cout << "M: " << M << endl << endl;
/* 
    Mat rhs_tr = rhs.t();
    cout << "1: " << rhs_tr << endl;
   // cout << rhs.size() << rhs_tr.size() << endl;
    Mat p_inv = (rhs_tr*rhs).inv()*rhs_tr;
    cout << "2: " << rhs_tr*rhs << endl;
    cout << "3: " << determinant(rhs_tr*rhs) << endl;
    cout << "4: " << (rhs_tr*rhs).inv() << endl;    
    cout << "5: " << p_inv << endl << endl;
    Mat transformation = lhs*p_inv;   // Solve for transformation matrix
    return lhs*rhs.inv();
    return transformation;
    */
    return M;
}

int getNextSlamIndex(int curr_vec_index, int element_index){
    while((curr_vec_index < temp_slam_trans.size() - 1) && (temp_slam_trans.at(curr_vec_index).at(3) < element_index)){
        curr_vec_index = curr_vec_index + 1;
        //cout << "aux index: " << curr_vec_index << endl;
    }
    return curr_vec_index;
}

bool is_different(vector<double> new_element, Mat current_matrix){
    for(int i = 0; i < current_matrix.cols; i++){
        //cout << "matrix: " << current_matrix.col(i)-Mat(new_element) << endl;
        Mat first;
        current_matrix.col(i).rowRange(0,3).copyTo(first);
        Mat second;
        Mat(new_element).rowRange(0,3).copyTo(second);
        //cout << "norm min: " << norm(second) * 0.35 <<endl;
        cout << "norm: " << norm(first-second)<<endl;
        if(norm(first-second) < threshold_){
            return false;
        }
    }
    return true;
}

// Check for elements in slam and chess structures with matching ids and push those to matrices
void findMatches(){
   
    int chess_vector_index = 0;
    int chess_element_index = temp_chess_trans.at(chess_vector_index).at(3);
    int slam_vector_index = getNextSlamIndex(0,chess_element_index);
    int slam_element_index = temp_slam_trans.at(slam_vector_index).at(3);
    int count = 0;
    cout << "vector sizes: " << temp_chess_trans.size() << " , " << temp_slam_trans.size() << endl;

    do{
        if(chess_element_index == slam_element_index){

            // Add vectors to matrices: remember they still contain extra index element
            if(chess_translation.cols == min_points){break;}
            else if(chess_translation.cols == 0){
                chess_translation = Mat(temp_chess_trans.at(chess_vector_index));
                slam_translation = Mat(temp_slam_trans.at(slam_vector_index));
            }else{                
                if(is_different(temp_chess_trans.at(chess_vector_index), chess_translation)){
                    hconcat(chess_translation, Mat(temp_chess_trans.at(chess_vector_index)), chess_translation);
                    hconcat(slam_translation, Mat(temp_slam_trans.at(slam_vector_index)), slam_translation);
                    cout << "chess_translation: " << chess_translation << endl;
                    cout << "slam_translation: " << slam_translation << endl;
                    cv::waitKey(0);
                }
            }
            chess_vector_index = chess_vector_index + 1;
            if(chess_vector_index >= max_points)
                break;
            chess_element_index = temp_chess_trans.at(chess_vector_index).at(3);
            if(temp_slam_trans.at(slam_vector_index).empty()){break;}; // Check there's more indices left, else stop
            slam_vector_index = getNextSlamIndex(slam_vector_index, chess_element_index);
            slam_element_index = temp_slam_trans.at(slam_vector_index).at(3);
            count++;
        }else{
            chess_vector_index = chess_vector_index + 1;
             if(chess_vector_index >= max_points)
                break;
            chess_element_index = temp_chess_trans.at(chess_vector_index).at(3);
            
        }
        //cout << "slam index: " << slam_vector_index << endl;
        // cout << "chess index: " << chess_vector_index << endl;

    }while(chess_vector_index < temp_chess_trans.size());

    cout << "matrix size: " << chess_translation.size() << endl;
    for(int i = 0;i<temp_chess_trans.size(); i++)
    {   break;
        vector<double> slam_vec = temp_slam_trans.at(i);
        vector<double> chess_vec = temp_chess_trans.at(i);

        cout << "ID:    SLAM=" << slam_vec.at(3) << "   chess"<< chess_vec.at(3) << endl;
    }
    // Erase matrices
    temp_chess_trans.clear();
    temp_slam_trans.clear();
    matching = false;
}


Mat extractRfromAffine(Mat aff){

    Mat rot(3,3,CV_64FC1);
    Mat rot3(3,3,CV_64FC1);
    
    double sx, sy, sz;
    sx = norm(aff.col(0));
    sy = norm(aff.col(1));
    sz = norm(aff.col(2));
    cout << "scale: " <<  sx << ", " << sy << ", " << sz << endl;
    rot.col(0) = (aff.col(0) / sx);
    rot.col(1) = (aff.col(1) / sy);
    rot.col(2) = (aff.col(2) / sz);
    //cout << "rot" << endl;
    //cout << rot << endl;
    //rot.rowRange(0,3)//.colRange(0,3).copyTo(rot3);
    //cout << rot3 << endl;
    
    assert(isRotationMatrix(rot));
    return rot;
}


void checkDataArray(int caller, vector<double> rot, vector<double> trans){
     
    // Add new data
    if (caller == 1 && temp_slam_trans.size() < max_points) {    // (SLAM calls the method)
        temp_slam_trans.push_back(trans);
    }else if (caller == 2 && temp_chess_trans.size() < max_points) {  // (Chess calls the method)
        temp_chess_trans.push_back(trans);
    }
    
    // Approx transformation matrices when enough points are gathered
    if(chess_translation.cols == min_points && slam_translation.cols == min_points){

        // Set 4th coordinate of matrices to one
        for(int i = 0; i < chess_translation.cols;i++){
            chess_translation.at<double>(3,i) = 1;
            slam_translation.at<double>(3,i) = 1;
        }
        //cout << "chess rotation" << chess_rotation << endl << endl;
        //cout << "slam rotation" << slam_rotation << endl << endl;
        // Find transformation matrices
        //cout << "---------------------------------------------------" <<endl;
        trans_M = findMatrixOfTransformation(chess_translation, slam_translation);
        //trans_M.setTo(0, trans_M < 10^-10); 
        //cout << "Affine transform on points: " << trans_M << endl;
        //cout << "chess: " << chess_translation << endl;
        //cout << "slam: " << slam_translation << endl;
        cout << "Final transform matrix = " << trans_M.size() << endl;
        cout << trans_M << endl;
        rot_M = extractRfromAffine(trans_M);
        cout << "predicted scale: " << endl;
        //cout << norm(chess_translation.at())
        cout << "Final rot = " << endl;
        cout << rot_M << endl;
        start_broadcast = true;

    }else if(temp_slam_trans.size() == max_points && temp_chess_trans.size() == max_points && !matching){
        cout << "array filled up" << endl;
        matching = true;
        findMatches();
    }
}


vector<double> transformRotation(vector<double> rvec){
    //cout << "+++++++++++++++++" << endl;
    vector<double> result;
    Vec3f rvec_cv; rvec_cv << rvec[0], rvec[1], rvec[2];
    //cout << "rvec_cv: " << rvec_cv << endl; 
    Mat rot = eulerAnglesToRotationMatrix(rvec_cv);
    rot = rot_M*rot;
    rot = rotationMatrixToEulerAngles(rot);
    //cout << "rot: " << rot << endl; 
    result.push_back(rot.at<double>(0,0));
    result.push_back(rot.at<double>(1,0));
    result.push_back(rot.at<double>(2,0));
    //cout << "+++++++++++++++++" << endl;
    return result;
}



// TODO: - Check ordering of array elements here
void slamCallback(const std_msgs::Float64MultiArray::ConstPtr& array){
   // cout << "slam: " << temp_slam_trans.size() << endl; 
    int i = 0;
    int index;
    vector<double> rot_vector;
    vector<double> trans_vector;
    for(vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	    {
		    if(i < 3){
                rot_vector.push_back(*it);
            }else if(i < 6){
                trans_vector.push_back(*it);
            }else if(i == 6){
                rot_vector.push_back(*it);
                trans_vector.push_back(*it);
            }
		    i++;
	    }
   
    if (!start_broadcast){
        checkDataArray(1,rot_vector,trans_vector);
    }else{
        // Calculate scaled slam 6dof and publish
        cout << "--------------------------------" << endl;
        trans_vector.at(3) = 1;
        rot_vector.at(3) = 1;
        //cout << trans_vector.size() << endl;
        //cout << rot_vector.size() << endl;
        Mat tm(trans_vector,false); Mat rm(rot_vector,false);
        Mat new_trans = trans_M*tm;
        //cout << "trans_vector : " <<  tm << endl;
        //cout << "new_trans : " << new_trans << endl;
        //Mat new_rot = rot_M*rm;
        vector<double> new_rot = transformRotation(rot_vector);
        //cout << "rot_vector : " << Mat(rot_vector) << endl;
        ///cout << "new_rot : " << new_rot.at(0) << "," << new_rot.at(1) << "," << new_rot.at(2) << endl;
        //cout << "--------------------------------" << endl;
        //Mat new_trans(ntrans.ToMat()); // ntrans.ToMat();  Last piece of puzzle
        //Mat new_rot(nrot.ToMat()); // nrot.ToMat();
        std_msgs::Float64MultiArray msg;
        msg.data.resize(new_rot.size() + new_trans.rows - 1);
        for (int i = 0; i <3;i++){
            msg.data[i] = new_trans.at<double>(i,0);
        }
        
        for (int i = 3; i < 6;i++){
            msg.data[i] = new_rot.at(i-3);
        }
        
    
        //ros::Rate loop_rate(5);
        pub.publish(msg);       // Publish image matrix
        cout << msg << endl;
        //loop_rate.sleep();
    }
}

void chessCallback(const std_msgs::Float64MultiArray::ConstPtr& array){
    // Broadcasting happens after enough data is collected
    //cout << "chess: " << temp_chess_trans.size() << endl;
    if (!start_broadcast){
        vector<double> rot_vector;
        vector<double> trans_vector;
        int i = 0;
        for(vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	    {
		    if(i < 3){
                rot_vector.push_back(*it);
            }else if(i < 6){
                trans_vector.push_back(*it);
               // cout << "translation vector: " << trans_vector << endl;
            }else if(i == 6){
                rot_vector.push_back(*it);
                trans_vector.push_back(*it);
            }
		    i++;
	    }
        checkDataArray(2,rot_vector,trans_vector);
    } 
    
}

int main(int argc, char** argv){

    // Init ROS
    ros::init(argc, argv, "transformer");
    ros::NodeHandle n;
    
    // Publisher
    pub = n.advertise<std_msgs::Float64MultiArray>("final_6dof", 10); 
    
    // Subscriptions
    ros::Subscriber sub = n.subscribe("chess_6dof", 10, chessCallback);
    ros::Subscriber sub2 = n.subscribe("slam_6dof", 10, slamCallback);

    ros::spin();
    
    return 0;
}
