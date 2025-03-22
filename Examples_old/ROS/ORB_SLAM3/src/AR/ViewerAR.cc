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

#include "ViewerAR.h"

#include <opencv2/highgui/highgui.hpp>

#include <mutex>
#include <thread>
#include <cstdlib>

using namespace std;

namespace ORB_SLAM3
{

const float eps = 1e-4;

cv::Mat ExpSO3(const float &x, const float &y, const float &z)
{
    cv::Mat I = cv::Mat::eye(3,3,CV_32F);
    const float d2 = x*x+y*y+z*z;
    const float d = sqrt(d2);
    cv::Mat W = (cv::Mat_<float>(3,3) << 0, -z, y,
                 z, 0, -x,
                 -y,  x, 0);
    if(d<eps)
        return (I + W + 0.5f*W*W);
    else
        return (I + W*sin(d)/d + W*W*(1.0f-cos(d))/d2);
}

cv::Mat ExpSO3(const cv::Mat &v)
{
    return ExpSO3(v.at<float>(0),v.at<float>(1),v.at<float>(2));
}

ViewerAR::ViewerAR() : mbPinchActive(false), mbRotationPinchActive(false), mbCubeInitialized(false), mpActivePlane(NULL)
{
    mPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    mLastPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    mRotationPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    mLastRotationPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    mCubePosition = cv::Mat::zeros(3, 1, CV_32F);
    mCubeRotation = cv::Mat::zeros(3, 1, CV_32F); // Initialize rotation as 0 around all axes
}

void ViewerAR::Run()
{
    int w,h,wui;

    cv::Mat im, Tcw;
    int status;
    vector<cv::KeyPoint> vKeys;
    vector<MapPoint*> vMPs;

    while(1)
    {
        GetImagePose(im,Tcw,status,vKeys,vMPs);
        if(im.empty())
            cv::waitKey(mT);
        else
        {
            w = im.cols;
            h = im.rows;
            break;
        }
    }

    wui=200;

    pangolin::CreateWindowAndBind("Viewer",w+wui,h);

    glEnable(GL_DEPTH_TEST);
    glEnable (GL_BLEND);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(wui));
    pangolin::Var<bool> menu_detectplane("menu.Insert Cube",false,false);
    pangolin::Var<bool> menu_clear("menu.Clear All",false,false);
    pangolin::Var<bool> menu_drawim("menu.Draw Image",true,true);
    pangolin::Var<bool> menu_drawcube("menu.Draw Cube",true,true);
    pangolin::Var<float> menu_cubesize("menu. Cube Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawgrid("menu.Draw Grid",true,true);
    pangolin::Var<int> menu_ngrid("menu. Grid Elements",3,1,10);
    pangolin::Var<float> menu_sizegrid("menu. Element Size",0.05,0.01,0.3);
    pangolin::Var<bool> menu_drawpoints("menu.Draw Points",false,true);

    pangolin::Var<bool> menu_LocalizationMode("menu.Localization Mode",false,true);
    bool bLocalizationMode = false;

    pangolin::View& d_image = pangolin::Display("image")
            .SetBounds(0,1.0f,pangolin::Attach::Pix(wui),1.0f,(float)w/h)
            .SetLock(pangolin::LockLeft, pangolin::LockTop);

    pangolin::GlTexture imageTexture(w,h,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);

    pangolin::OpenGlMatrixSpec P = pangolin::ProjectionMatrixRDF_TopLeft(w,h,fx,fy,cx,cy,0.001,1000);

    vector<Plane*> vpPlane;

    while(1)
    {

        if(menu_LocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menu_LocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Activate camera view
        d_image.Activate();
        glColor3f(1.0,1.0,1.0);

        // Get last image and its computed pose from SLAM
        GetImagePose(im,Tcw,status,vKeys,vMPs);

        // Add text to image
        PrintStatus(status,bLocalizationMode,im);

        if(menu_drawpoints)
            DrawTrackedPoints(vKeys,vMPs,im);

        // Draw image
        if(menu_drawim)
            DrawImageTexture(imageTexture,im);

        glClear(GL_DEPTH_BUFFER_BIT);

        // Load camera projection
        glMatrixMode(GL_PROJECTION);
        P.Load();

        glMatrixMode(GL_MODELVIEW);

        // Load camera pose
        LoadCameraPose(Tcw);

        // Draw virtual things
        if(status==2)
        {
            if(menu_clear)
            {
                if(!vpPlane.empty())
                {
                    for(size_t i=0; i<vpPlane.size(); i++)
                    {
                        delete vpPlane[i];
                    }
                    vpPlane.clear();
                    mpActivePlane = NULL;
                    mbCubeInitialized = false;
                    cout << "All cubes erased!" << endl;
                }
                menu_clear = false;
            }
            if(menu_detectplane)
            {
                Plane* pPlane = DetectPlane(Tcw,vMPs,50);
                if(pPlane)
                {
                    cout << "New virtual cube inserted!" << endl;
                    vpPlane.push_back(pPlane);
                    mpActivePlane = pPlane;
                    
                    // Initialize cube at the center of the plane
                    mCubePosition = pPlane->o.clone();
                    mbCubeInitialized = true;
                }
                else
                {
                    cout << "No plane detected. Point the camera to a planar region." << endl;
                }
                menu_detectplane = false;
            }

            if(!vpPlane.empty())
            {
                // Recompute plane if there has been a loop closure or global BA
                // In localization mode, map is not updated so we do not need to recompute
                bool bRecompute = false;
                if(!bLocalizationMode)
                {
                    if(mpSystem->MapChanged())
                    {
                        cout << "Map changed. All virtual elements are recomputed!" << endl;
                        bRecompute = true;
                    }
                }

                // Check if we need to update cube position based on pinch gesture
                {
                    unique_lock<mutex> lock(mMutexPinch);
                    
                    // Direct manipulation using SetPinchGesture instead of updating here
                    // The SetPinchGesture method now handles the movement logic
                    
                    // Additional debug for active plane
                    if (mpActivePlane)
                    {
                        string planeInfo = "Active plane origin: " + 
                                          to_string(mpActivePlane->o.at<float>(0)) + 
                                          ", " + to_string(mpActivePlane->o.at<float>(1)) + 
                                          ", " + to_string(mpActivePlane->o.at<float>(2));
                        AddTextToImage(planeInfo, im, 255, 165, 0);
                    }
                    else
                    {
                        AddTextToImage("No active plane detected", im, 255, 0, 0);
                    }
                }
                
                // Check for rotation pinch gesture
                {
                    unique_lock<mutex> lock(mMutexRotationPinch);
                    if (mbRotationPinchActive)
                    {
                        string s = "Rotation Pinch Active: " + to_string(mRotationPinchPosition.at<float>(0)) + 
                                  ", " + to_string(mRotationPinchPosition.at<float>(1));
                        AddTextToImage(s, im, 255, 0, 255);
                        
                        // Add cube rotation info
                        if (mbCubeInitialized)
                        {
                            string rotInfo = "Cube Rotation: " + to_string(mCubeRotation.at<float>(0)) + 
                                          ", " + to_string(mCubeRotation.at<float>(1)) + 
                                          ", " + to_string(mCubeRotation.at<float>(2));
                            AddTextToImage(rotInfo, im, 255, 0, 255);
                        }
                    }
                }

                for(size_t i=0; i<vpPlane.size(); i++)
                {
                    Plane* pPlane = vpPlane[i];

                    if(pPlane)
                    {
                        if(bRecompute)
                        {
                            pPlane->Recompute();
                        }
                        
                        glPushMatrix();
                        pPlane->glTpw.Multiply();

                        // Draw grid plane
                        if(menu_drawgrid)
                        {
                            DrawPlane(menu_ngrid,menu_sizegrid);
                        }

                        glPopMatrix();
                    }
                }
                
                // Draw the cube at its current position if initialized
                if (mbCubeInitialized && menu_drawcube && mpActivePlane)
                {
                    glPushMatrix();
                    mpActivePlane->glTpw.Multiply();
                    
                    // Convert from world coordinates to plane-relative coordinates
                    cv::Mat planeRelativePos = mpActivePlane->Tpw.rowRange(0,3).colRange(0,3) * 
                                              (mCubePosition - mpActivePlane->o);
                    
                    // Print cube position for debugging
                    cout << "Drawing cube at plane-relative position: " 
                         << planeRelativePos.at<float>(0) << ", "
                         << planeRelativePos.at<float>(1) << ", "
                         << planeRelativePos.at<float>(2) << endl;
                    
                    // Draw a bigger cube for better visibility
                    float cubeSize = menu_cubesize * 1.5;
                    
                    // Draw the main cube
                    DrawCube(cubeSize, 
                            planeRelativePos.at<float>(0),
                            planeRelativePos.at<float>(1), 
                            planeRelativePos.at<float>(2));
                    
                    glPopMatrix();
                    
                    // Add visual indicator of cube position on the image
                    if (!mTcw.empty())
                    {
                        // Project 3D cube position to 2D image
                        cv::Mat Rcw = mTcw.rowRange(0,3).colRange(0,3);
                        cv::Mat tcw = mTcw.rowRange(0,3).col(3);
                        
                        // World point to camera coords
                        cv::Mat Pc = Rcw*mCubePosition + tcw;
                        
                        // Camera coords to image
                        if (Pc.at<float>(2) > 0)  // Check if point is in front of camera
                        {
                            float invZ = 1.0f/Pc.at<float>(2);
                            float u = fx*Pc.at<float>(0)*invZ + cx;
                            float v = fy*Pc.at<float>(1)*invZ + cy;
                            
                            // Draw a prominent marker at the projected cube position
                            if (u >= 0 && u < im.cols && v >= 0 && v < im.rows)
                            {
                                cv::circle(im, cv::Point(u, v), 15, cv::Scalar(0, 0, 255), -1);
                                cv::circle(im, cv::Point(u, v), 17, cv::Scalar(255, 255, 255), 2);
                                
                                // Add text label
                                cv::putText(im, "CUBE", cv::Point(u+20, v), 
                                            cv::FONT_HERSHEY_SIMPLEX, 0.7, 
                                            cv::Scalar(0, 0, 255), 2);
                            }
                        }
                    }
                }
            }
        }

        pangolin::FinishFrame();
        usleep(mT*1000);
    }
}

void ViewerAR::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM3::MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
    mvMPs = vMPs;
}

void ViewerAR::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

void ViewerAR::SetPinchGesture(const bool isPinching, const float x, const float y, const float z)
{
    unique_lock<mutex> lock(mMutexPinch);
    
    // Check if we're transitioning from not pinching to pinching
    bool startingPinch = !mbPinchActive && isPinching;
    
    // Update pinch state
    mbPinchActive = isPinching;
    
    if (isPinching)
    {
        // Update pinch position
        mPinchPosition.at<float>(0) = x;
        mPinchPosition.at<float>(1) = y;
        mPinchPosition.at<float>(2) = z;
        
        // If we have an active plane and a cube is initialized
        if (mpActivePlane && mbCubeInitialized)
        {
            // First pinch - store the initial position for reference
            if (startingPinch)
            {
                mLastPinchPosition = MapPinchTo3D(x, y, mpActivePlane);
                cout << "Starting pinch at: " << x << ", " << y << endl;
            }
            else if (!mLastPinchPosition.empty() && cv::norm(mLastPinchPosition) > 0)
            {
                // Direct cube movement - more reliable than differential movement
                cv::Mat newPosition = MapPinchTo3D(x, y, mpActivePlane);
                cv::Mat delta = newPosition - mLastPinchPosition;
                
                // Apply movement to cube
                mCubePosition += delta;
                
                // Update last position
                mLastPinchPosition = newPosition.clone();
                
                cout << "Moving cube by: " << delta.at<float>(0) << ", " 
                     << delta.at<float>(1) << ", " 
                     << delta.at<float>(2) << endl;
            }
        }
    }
    else
    {
        // Reset last pinch position when not pinching
        mLastPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    }
}

void ViewerAR::SetRotationPinchGesture(const bool isRotationPinching, const float x, const float y, const float z)
{
    unique_lock<mutex> lock(mMutexRotationPinch);
    
    // Check if we're transitioning from not pinching to pinching
    bool startingRotationPinch = !mbRotationPinchActive && isRotationPinching;
    
    // Update pinch state
    mbRotationPinchActive = isRotationPinching;
    
    if (isRotationPinching)
    {
        // Update rotation pinch position
        mRotationPinchPosition.at<float>(0) = x;
        mRotationPinchPosition.at<float>(1) = y;
        mRotationPinchPosition.at<float>(2) = z;
        
        // If we have an active plane and a cube is initialized
        if (mpActivePlane && mbCubeInitialized)
        {
            // First pinch - store the initial position for reference
            if (startingRotationPinch)
            {
                mLastRotationPinchPosition = mRotationPinchPosition.clone();
                cout << "Starting rotation pinch at: " << x << ", " << y << endl;
            }
            else if (!mLastRotationPinchPosition.empty() && cv::norm(mLastRotationPinchPosition) > 0)
            {
                // Calculate rotation based on 2D pinch movement
                cv::Mat rotation = MapRotationPinch(x, y, mpActivePlane);
                
                // Apply rotation to cube
                mCubeRotation.at<float>(0) += rotation.at<float>(0); // Roll
                mCubeRotation.at<float>(1) += rotation.at<float>(1); // Pitch
                mCubeRotation.at<float>(2) += rotation.at<float>(2); // Yaw
                
                // Update last position
                mLastRotationPinchPosition = mRotationPinchPosition.clone();
                
                cout << "Rotating cube by: " << rotation.at<float>(0) << ", " 
                     << rotation.at<float>(1) << ", " 
                     << rotation.at<float>(2) << endl;
            }
        }
    }
    else
    {
        // Reset last rotation pinch position when not pinching
        mLastRotationPinchPosition = cv::Mat::zeros(3, 1, CV_32F);
    }
}

void ViewerAR::MoveCube(const float x, const float y, const float z)
{
    if (mbCubeInitialized)
    {
        mCubePosition.at<float>(0) += x;
        mCubePosition.at<float>(1) += y;
        mCubePosition.at<float>(2) += z;
    }
}

void ViewerAR::RotateCube(const float rx, const float ry, const float rz)
{
    if (mbCubeInitialized)
    {
        mCubeRotation.at<float>(0) += rx;
        mCubeRotation.at<float>(1) += ry;
        mCubeRotation.at<float>(2) += rz;
    }
}

cv::Mat ViewerAR::MapPinchTo3D(const float pinchX, const float pinchY, Plane* pPlane)
{
    if (!pPlane)
        return cv::Mat::zeros(3, 1, CV_32F);
    
    // Use a simplified approach for more direct mapping
    // This makes movements larger and more noticeable
    
    // Create ray direction from camera center through pinch point
    // Amplify the movement by scaling the pinch coordinates
    const float MOVEMENT_SCALE = 1.5; // Increase this value to make movements more obvious
    
    cv::Mat ray = cv::Mat::ones(3, 1, CV_32F);
    ray.at<float>(0) = ((pinchX - 0.5) * MOVEMENT_SCALE) * -1;  // Center and scale x
    ray.at<float>(1) = ((pinchY - 0.5) * MOVEMENT_SCALE) * -1;  // Center and scale y
    gi
    // Transform ray to world coordinates (using current camera pose)
    cv::Mat Rwc = mTcw.rowRange(0,3).colRange(0,3).t();
    ray = Rwc * ray;
    
    // Get camera position in world coordinates
    cv::Mat Ow = -Rwc * mTcw.rowRange(0,3).col(3);
    
    // Project movement onto the plane
    cv::Mat planeNormal = pPlane->n;
    
    // Calculate a point on the plane (we'll use the origin of the plane)
    cv::Mat pointOnPlane = pPlane->o;
    
    // Project ray onto plane to get movement direction on the plane
    cv::Mat projectedRay = ray - (planeNormal * (ray.dot(planeNormal)));
    
    // Normalize and scale the projected ray for consistent movement
    float projNorm = cv::norm(projectedRay);
    if (projNorm > 0)
    {
        projectedRay = projectedRay * (1.0 / projNorm) * 0.05; // Scale for appropriate movement speed
    }
    
    // Log for debugging
    std::cout << "Pinch coords: " << pinchX << ", " << pinchY << std::endl;
    std::cout << "Projected ray: " << projectedRay.at<float>(0) << ", " 
              << projectedRay.at<float>(1) << ", " 
              << projectedRay.at<float>(2) << std::endl;
    
    // Return the current cube position plus the projected movement
    // This ensures the cube moves relative to its current position
    return mCubePosition + projectedRay;
}

cv::Mat ViewerAR::MapRotationPinch(const float pinchX, const float pinchY, Plane* pPlane)
{
    cv::Mat rotation = cv::Mat::zeros(3, 1, CV_32F);
    
    if (!pPlane || mLastRotationPinchPosition.empty())
        return rotation;
    
    // Calculate deltas in pinch position
    float deltaX = pinchX - mLastRotationPinchPosition.at<float>(0);
    float deltaY = pinchY - mLastRotationPinchPosition.at<float>(1);
    
    // Scale factors for rotation (adjust these to control rotation sensitivity)
    const float ROTATION_SCALE_X = 12.0;
    const float ROTATION_SCALE_Y = 12.0;
    const float ROTATION_SCALE_Z = 12.0;
    
    // Map x movement to yaw (rotation around vertical axis)
    rotation.at<float>(2) = deltaX * ROTATION_SCALE_Z;
    
    // Map y movement to pitch (rotation around horizontal axis)
    rotation.at<float>(1) = deltaY * ROTATION_SCALE_Y;
    
    // Roll can be controlled by additional gestures if needed
    // Here we'll leave it at 0
    rotation.at<float>(0) = 0.0f;
    
    return rotation;
}

void ViewerAR::LoadCameraPose(const cv::Mat &Tcw)
{
    if(!Tcw.empty())
    {
        pangolin::OpenGlMatrix M;

        M.m[0] = Tcw.at<float>(0,0);
        M.m[1] = Tcw.at<float>(1,0);
        M.m[2] = Tcw.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Tcw.at<float>(0,1);
        M.m[5] = Tcw.at<float>(1,1);
        M.m[6] = Tcw.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Tcw.at<float>(0,2);
        M.m[9] = Tcw.at<float>(1,2);
        M.m[10] = Tcw.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = Tcw.at<float>(0,3);
        M.m[13] = Tcw.at<float>(1,3);
        M.m[14] = Tcw.at<float>(2,3);
        M.m[15]  = 1.0;

        M.Load();
    }
}

void ViewerAR::PrintStatus(const int &status, const bool &bLocMode, cv::Mat &im)
{
    if(!bLocMode)
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("SLAM ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("SLAM LOST",im,255,0,0); break;}
        }
    }
    else
    {
        switch(status)
        {
        case 1:  {AddTextToImage("SLAM NOT INITIALIZED",im,255,0,0); break;}
        case 2:  {AddTextToImage("LOCALIZATION ON",im,0,255,0); break;}
        case 3:  {AddTextToImage("LOCALIZATION LOST",im,255,0,0); break;}
        }
    }
    
    // Add pinch status info
    {
        unique_lock<mutex> lock(mMutexPinch);
        if (mbPinchActive)
        {
            string s = "Pinch Active: " + to_string(mPinchPosition.at<float>(0)) + 
                      ", " + to_string(mPinchPosition.at<float>(1));
            AddTextToImage(s, im, 0, 255, 255);
            
            // Add cube position info
            if (mbCubeInitialized && mpActivePlane)
            {
                string cubePos = "Cube Position: " + to_string(mCubePosition.at<float>(0)) + 
                              ", " + to_string(mCubePosition.at<float>(1)) + 
                              ", " + to_string(mCubePosition.at<float>(2));
                AddTextToImage(cubePos, im, 0, 255, 0);
            }
        }
    }
    
    // Add rotation pinch status info
    {
        unique_lock<mutex> lock(mMutexRotationPinch);
        if (mbRotationPinchActive)
        {
            string s = "Rotation Pinch Active: " + to_string(mRotationPinchPosition.at<float>(0)) + 
                      ", " + to_string(mRotationPinchPosition.at<float>(1));
            AddTextToImage(s, im, 255, 0, 255);
            
            // Add cube rotation info
            if (mbCubeInitialized)
            {
                string rotInfo = "Cube Rotation: " + to_string(mCubeRotation.at<float>(0)) + 
                              ", " + to_string(mCubeRotation.at<float>(1)) + 
                              ", " + to_string(mCubeRotation.at<float>(2));
                AddTextToImage(rotInfo, im, 255, 0, 255);
            }
        }
    }
}

void ViewerAR::AddTextToImage(const string &s, cv::Mat &im, const int r, const int g, const int b)
{
    int l = 10;
    //imText.rowRange(im.rows-imText.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l-1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l-1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);
    cv::putText(im,s,cv::Point(l+1,im.rows-(l+1)),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(255,255,255),2,8);

    cv::putText(im,s,cv::Point(l,im.rows-l),cv::FONT_HERSHEY_PLAIN,1.5,cv::Scalar(r,g,b),2,8);
}

void ViewerAR::DrawImageTexture(pangolin::GlTexture &imageTexture, cv::Mat &im)
{
    if(!im.empty())
    {
        imageTexture.Upload(im.data,GL_RGB,GL_UNSIGNED_BYTE);
        imageTexture.RenderToViewportFlipY();
    }
}

void ViewerAR::DrawCube(const float &size, const float x, const float y, const float z)
{
    // Create a translation matrix for the cube position
    pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(-x, -size-y, -z);
    
    // If we have rotation data and the cube is initialized, apply rotations
    if (mbCubeInitialized)
    {
        // Get rotation angles from cube rotation state
        float roll = mCubeRotation.at<float>(0);
        float pitch = mCubeRotation.at<float>(1);
        float yaw = mCubeRotation.at<float>(2);
        
        // Apply rotations in order: first roll, then pitch, then yaw
        // This order matters! Different orders produce different final orientations
        pangolin::OpenGlMatrix Rx = pangolin::OpenGlMatrix::RotateX(roll);
        pangolin::OpenGlMatrix Ry = pangolin::OpenGlMatrix::RotateY(pitch);
        pangolin::OpenGlMatrix Rz = pangolin::OpenGlMatrix::RotateZ(yaw);
        
        // Combine the rotations with the translation
        M = M * Rz * Ry * Rx;
    }
    
    // Apply the transformation and draw the cube
    glPushMatrix();
    M.Multiply();
    pangolin::glDrawColouredCube(-size, size);
    glPopMatrix();
}

void ViewerAR::DrawPlane(Plane *pPlane, int ndivs, float ndivsize)
{
    glPushMatrix();
    pPlane->glTpw.Multiply();
    DrawPlane(ndivs,ndivsize);
    glPopMatrix();
}

void ViewerAR::DrawPlane(int ndivs, float ndivsize)
{
    // Plane parallel to x-z at origin with normal -y
    const float minx = -ndivs*ndivsize;
    const float minz = -ndivs*ndivsize;
    const float maxx = ndivs*ndivsize;
    const float maxz = ndivs*ndivsize;


    glLineWidth(2);
    glColor3f(0.7f,0.7f,1.0f);
    glBegin(GL_LINES);

    for(int n = 0; n<=2*ndivs; n++)
    {
        glVertex3f(minx+ndivsize*n,0,minz);
        glVertex3f(minx+ndivsize*n,0,maxz);
        glVertex3f(minx,0,minz+ndivsize*n);
        glVertex3f(maxx,0,minz+ndivsize*n);
    }

    glEnd();

}

void ViewerAR::DrawTrackedPoints(const std::vector<cv::KeyPoint> &vKeys, const std::vector<MapPoint *> &vMPs, cv::Mat &im)
{
    const int N = vKeys.size();


    for(int i=0; i<N; i++)
    {
        if(vMPs[i])
        {
            cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
        }
    }
}

Plane* ViewerAR::DetectPlane(const cv::Mat Tcw, const std::vector<MapPoint*> &vMPs, const int iterations)
{
    // Retrieve 3D points
    vector<cv::Mat> vPoints;
    vPoints.reserve(vMPs.size());
    vector<MapPoint*> vPointMP;
    vPointMP.reserve(vMPs.size());

    for(size_t i=0; i<vMPs.size(); i++)
    {
        MapPoint* pMP=vMPs[i];
        if(pMP)
        {
            if(pMP->Observations()>5)
            {
                vPoints.push_back(ORB_SLAM3::Converter::toCvMat(pMP->GetWorldPos()));
                vPointMP.push_back(pMP);
            }
        }
    }

    const int N = vPoints.size();

    if(N<50)
        return NULL;


    // Indices for minimum set selection
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    float bestDist = 1e10;
    vector<float> bestvDist;

    //RANSAC
    for(int n=0; n<iterations; n++)
    {
        vAvailableIndices = vAllIndices;

        cv::Mat A(3,4,CV_32F);
        A.col(3) = cv::Mat::ones(3,1,CV_32F);

        // Get min set of points
        for(short i = 0; i < 3; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];

            A.row(i).colRange(0,3) = vPoints[idx].t();

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

        cv::Mat u,w,vt;
        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        const float a = vt.at<float>(3,0);
        const float b = vt.at<float>(3,1);
        const float c = vt.at<float>(3,2);
        const float d = vt.at<float>(3,3);

        vector<float> vDistances(N,0);

        const float f = 1.0f/sqrt(a*a+b*b+c*c+d*d);

        for(int i=0; i<N; i++)
        {
            vDistances[i] = fabs(vPoints[i].at<float>(0)*a+vPoints[i].at<float>(1)*b+vPoints[i].at<float>(2)*c+d)*f;
        }

        vector<float> vSorted = vDistances;
        sort(vSorted.begin(),vSorted.end());

        int nth = max((int)(0.2*N),20);
        const float medianDist = vSorted[nth];

        if(medianDist<bestDist)
        {
            bestDist = medianDist;
            bestvDist = vDistances;
        }
    }

    // Compute threshold inlier/outlier
    const float th = 1.4*bestDist;
    vector<bool> vbInliers(N,false);
    int nInliers = 0;
    for(int i=0; i<N; i++)
    {
        if(bestvDist[i]<th)
        {
            nInliers++;
            vbInliers[i]=true;
        }
    }

    vector<MapPoint*> vInlierMPs(nInliers,NULL);
    int nin = 0;
    for(int i=0; i<N; i++)
    {
        if(vbInliers[i])
        {
            vInlierMPs[nin] = vPointMP[i];
            nin++;
        }
    }

    return new Plane(vInlierMPs,Tcw);
}

Plane::Plane(const std::vector<MapPoint *> &vMPs, const cv::Mat &Tcw):mvMPs(vMPs),mTcw(Tcw.clone())
{
    rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    Recompute();
}

void Plane::Recompute()
{
    const int N = mvMPs.size();

    // Recompute plane with all points
    cv::Mat A = cv::Mat(N,4,CV_32F);
    A.col(3) = cv::Mat::ones(N,1,CV_32F);

    o = cv::Mat::zeros(3,1,CV_32F);

    int nPoints = 0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvMPs[i];
        if(!pMP->isBad())
        {
            cv::Mat Xw = ORB_SLAM3::Converter::toCvMat(pMP->GetWorldPos());
            o+=Xw;
            A.row(nPoints).colRange(0,3) = Xw.t();
            nPoints++;
        }
    }
    A.resize(nPoints);

    cv::Mat u,w,vt;
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    float a = vt.at<float>(3,0);
    float b = vt.at<float>(3,1);
    float c = vt.at<float>(3,2);

    o = o*(1.0f/nPoints);
    const float f = 1.0f/sqrt(a*a+b*b+c*c);

    // Compute XC just the first time
    if(XC.empty())
    {
        cv::Mat Oc = -mTcw.colRange(0,3).rowRange(0,3).t()*mTcw.rowRange(0,3).col(3);
        XC = Oc-o;
    }

    if((XC.at<float>(0)*a+XC.at<float>(1)*b+XC.at<float>(2)*c)>0)
    {
        a=-a;
        b=-b;
        c=-c;
    }

    const float nx = a*f;
    const float ny = b*f;
    const float nz = c*f;

    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float sa = cv::norm(v);
    const float ca = up.dot(n);
    const float ang = atan2(sa,ca);
    Tpw = cv::Mat::eye(4,4,CV_32F);


    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*ang/sa)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;

}

Plane::Plane(const float &nx, const float &ny, const float &nz, const float &ox, const float &oy, const float &oz)
{
    n = (cv::Mat_<float>(3,1)<<nx,ny,nz);
    o = (cv::Mat_<float>(3,1)<<ox,oy,oz);

    cv::Mat up = (cv::Mat_<float>(3,1) << 0.0f, 1.0f, 0.0f);

    cv::Mat v = up.cross(n);
    const float s = cv::norm(v);
    const float c = up.dot(n);
    const float a = atan2(s,c);
    Tpw = cv::Mat::eye(4,4,CV_32F);
    const float rang = -3.14f/2+((float)rand()/RAND_MAX)*3.14f;
    cout << rang;
    Tpw.rowRange(0,3).colRange(0,3) = ExpSO3(v*a/s)*ExpSO3(up*rang);
    o.copyTo(Tpw.col(3).rowRange(0,3));

    glTpw.m[0] = Tpw.at<float>(0,0);
    glTpw.m[1] = Tpw.at<float>(1,0);
    glTpw.m[2] = Tpw.at<float>(2,0);
    glTpw.m[3]  = 0.0;

    glTpw.m[4] = Tpw.at<float>(0,1);
    glTpw.m[5] = Tpw.at<float>(1,1);
    glTpw.m[6] = Tpw.at<float>(2,1);
    glTpw.m[7]  = 0.0;

    glTpw.m[8] = Tpw.at<float>(0,2);
    glTpw.m[9] = Tpw.at<float>(1,2);
    glTpw.m[10] = Tpw.at<float>(2,2);
    glTpw.m[11]  = 0.0;

    glTpw.m[12] = Tpw.at<float>(0,3);
    glTpw.m[13] = Tpw.at<float>(1,3);
    glTpw.m[14] = Tpw.at<float>(2,3);
    glTpw.m[15]  = 1.0;
}

}