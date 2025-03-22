/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/**
 *  Inspired by the amazing Tina
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include"../../../include/System.h"

#include"ViewerAR.h"

using namespace std;


ORB_SLAM3::ViewerAR viewerAR;
bool bRGB = true;

cv::Mat K;
cv::Mat DistCoef;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    void GrabPinchActive(const std_msgs::Bool::ConstPtr& msg);
    void GrabPinchPosition(const geometry_msgs::Point::ConstPtr& msg);
    
    // Methods for handling rotation pinch
    void GrabRotationPinchActive(const std_msgs::Bool::ConstPtr& msg);
    void GrabRotationPinchPosition(const geometry_msgs::Point::ConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
    
    // Translation pinch state
    bool isPinching;
    float pinchX, pinchY, pinchZ;
    
    // Rotation pinch state
    bool isRotationPinching;
    float rotationPinchX, rotationPinchY, rotationPinchZ;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,false);


    cout << endl << endl;
    cout << "-----------------------" << endl;
    cout << "Augmented Reality Demo" << endl;
    cout << "1) Translate the camera to initialize SLAM." << endl;
    cout << "2) Look at a planar region and translate the camera." << endl;
    cout << "3) Press Insert Cube to place a virtual cube in the plane. " << endl;
    cout << "4) Use pinch gesture (thumb-index) to move the cube on the plane." << endl;
    cout << "5) Use second pinch gesture (thumb-middle) to rotate the cube." << endl;
    cout << endl;
    cout << "You can place several cubes in different planes." << endl;
    cout << "-----------------------" << endl;
    cout << endl;


    viewerAR.SetSLAM(&SLAM);

    ImageGrabber igb(&SLAM);
    
    // Initialize pinch states
    igb.isPinching = false;
    igb.pinchX = 0.0f;
    igb.pinchY = 0.0f;
    igb.pinchZ = 0.0f;
    
    igb.isRotationPinching = false;
    igb.rotationPinchX = 0.0f;
    igb.rotationPinchY = 0.0f;
    igb.rotationPinchZ = 0.0f;

    ros::NodeHandle nodeHandler;
    
    // Subscribe to camera image
    ros::Subscriber sub_image = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);
    
    // Subscribe to pinch gesture topics
    ros::Subscriber sub_pinch_active = nodeHandler.subscribe("/pinch_active", 1, &ImageGrabber::GrabPinchActive, &igb);
    ros::Subscriber sub_pinch_position = nodeHandler.subscribe("/pinch_position", 1, &ImageGrabber::GrabPinchPosition, &igb);
    
    // Subscribe to rotation pinch gesture topics
    ros::Subscriber sub_rotation_pinch_active = nodeHandler.subscribe("/rotation_pinch_active", 1, 
                                                                    &ImageGrabber::GrabRotationPinchActive, &igb);
    ros::Subscriber sub_rotation_pinch_position = nodeHandler.subscribe("/rotation_pinch_position", 1, 
                                                                      &ImageGrabber::GrabRotationPinchPosition, &igb);


    cv::FileStorage fSettings(argv[2], cv::FileStorage::READ);
    bRGB = static_cast<bool>((int)fSettings["Camera.RGB"]);
    float fps = fSettings["Camera.fps"];
    viewerAR.SetFPS(fps);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    viewerAR.SetCameraCalibration(fx,fy,cx,cy);

    K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;

    DistCoef = cv::Mat::zeros(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }

    thread tViewer = thread(&ORB_SLAM3::ViewerAR::Run,&viewerAR);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
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
    cv::Mat im = cv_ptr->image.clone();
    cv::Mat imu;
    
    // Track frame with ORB-SLAM3
    cv::Mat Tcw = ORB_SLAM3::Converter::toCvMat(mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()).matrix());

    int state = mpSLAM->GetTrackingState();
    vector<ORB_SLAM3::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();

    cv::undistort(im,imu,K,DistCoef);

    // Update ViewerAR with both pinch states
    viewerAR.SetPinchGesture(isPinching, pinchX, pinchY, pinchZ);
    viewerAR.SetRotationPinchGesture(isRotationPinching, rotationPinchX, rotationPinchY, rotationPinchZ);
    
    if(bRGB)
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    else
    {
        cv::cvtColor(imu,imu,CV_RGB2BGR);
        viewerAR.SetImagePose(imu,Tcw,state,vKeys,vMPs);
    }    
}

void ImageGrabber::GrabPinchActive(const std_msgs::Bool::ConstPtr& msg)
{
    bool wasActive = isPinching;
    isPinching = msg->data;
    
    // Debug output - more verbose
    if (isPinching && !wasActive)
        ROS_INFO("PINCH ACTIVATED - CUBE SHOULD START MOVING");
    else if (!isPinching && wasActive)
        ROS_INFO("PINCH DEACTIVATED - CUBE SHOULD STOP MOVING");
}

void ImageGrabber::GrabPinchPosition(const geometry_msgs::Point::ConstPtr& msg)
{
    // Update position even if not pinching to maintain latest hand position
    pinchX = msg->x;
    pinchY = msg->y;
    pinchZ = msg->z;
    
    // Debug output
    if (isPinching) {
        ROS_INFO_THROTTLE(0.2, "ACTIVE PINCH - position: %.3f, %.3f, %.3f", pinchX, pinchY, pinchZ);
    } else {
        ROS_INFO_THROTTLE(1.0, "Hand position (no pinch): %.3f, %.3f, %.3f", pinchX, pinchY, pinchZ);
    }
    
    // Force update the ViewerAR with current pinch state for instant feedback
    viewerAR.SetPinchGesture(isPinching, pinchX, pinchY, pinchZ);
}

void ImageGrabber::GrabRotationPinchActive(const std_msgs::Bool::ConstPtr& msg)
{
    bool wasActive = isRotationPinching;
    isRotationPinching = msg->data;
    
    // Debug output
    if (isRotationPinching && !wasActive)
        ROS_INFO("ROTATION PINCH ACTIVATED - CUBE SHOULD START ROTATING");
    else if (!isRotationPinching && wasActive)
        ROS_INFO("ROTATION PINCH DEACTIVATED - CUBE SHOULD STOP ROTATING");
}

void ImageGrabber::GrabRotationPinchPosition(const geometry_msgs::Point::ConstPtr& msg)
{
    // Update position even if not pinching to maintain latest hand position
    rotationPinchX = msg->x;
    rotationPinchY = msg->y;
    rotationPinchZ = msg->z;
    
    // Debug output
    if (isRotationPinching) {
        ROS_INFO_THROTTLE(0.2, "ACTIVE ROTATION PINCH - position: %.3f, %.3f, %.3f", 
                         rotationPinchX, rotationPinchY, rotationPinchZ);
    }
    
    // Force update the ViewerAR with current rotation pinch state
    viewerAR.SetRotationPinchGesture(isRotationPinching, rotationPinchX, rotationPinchY, rotationPinchZ);
}