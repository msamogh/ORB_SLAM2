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

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include <functional>
using namespace std::placeholders;

#include <mutex>


namespace pangolin {
    struct MyHandler3D : Handler3D {

        MyHandler3D(
            OpenGlRenderState &cam_state,
            AxisDirection enforce_up = AxisNone,
            float trans_scale = 0.01f,
            float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF) : Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction) {};

        void Mouse(View &display, MouseButton button, int x, int y, bool pressed, int button_state) {
            std::cout << "Clicked at " << x << ", " << y << std::endl;
        }
    };
}

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void IdentifyKeyPoint(int event, int x, int y, int flags, void* userdata)
{
    // Consider only left mouse clicks
    if (event != cv::EVENT_LBUTTONDOWN) return;

    // Unpack user data
    Tracking* pTracker = (Tracking*) userdata;

    // Find closest keypoint from the clicked point
    vector<cv::KeyPoint> vCurrentKeys = pTracker->mCurrentFrame.mvKeys;
    vector<MapPoint*> vpMapPoints = pTracker->mCurrentFrame.mvpMapPoints;

    cout << "vCurrentKeys: " << vCurrentKeys.size() << ", vpMapPoints: " << vpMapPoints.size() << endl;

    // Sanity check to verify if all keypoints have a corresponding non-null map point
    bool flag = true;
    for (int i = 0; i < vCurrentKeys.size(); i++) {
        if (!vpMapPoints[i]) {
            flag = false;
            break;
        }
    }
    if (flag) {
        cout << "Things going smoothly" << endl;
    } else {
        cout << "Something is wrong!" << endl;
    }

    // Initialize variables needed to store closest keypoint
    uint closestIndex = 0;
    float closestDistanceSquared = pow(vCurrentKeys[0].pt.x - x, 2) + pow(vCurrentKeys[0].pt.y - y, 2);
    cv::KeyPoint closestKey = vCurrentKeys[0];

    for (uint i = 0; i < vCurrentKeys.size(); i++) {
        cv::KeyPoint key = vCurrentKeys[i];
        float distanceSquared = pow(key.pt.x - x, 2) + pow(key.pt.y - y, 2);
        if (distanceSquared < closestDistanceSquared) {
            closestDistanceSquared = distanceSquared;
            closestKey = key;
            closestIndex = i;
            key.class_id = 1;
            cout << key.class_id << endl;
        }
    }

    // Change class_id to mark the selected point
    vCurrentKeys[closestIndex].class_id = 1;

    // Get corresponding world coordinates of the closest keypoint
    cout << "CKPT 1" << endl;
    vector<MapPoint*> vpMPs = pTracker->mpMap->GetAllMapPoints();
    cout << "World: " << pTracker->mpMap->GetAllMapPoints().size() << "; Keyframes: " << pTracker->mCurrentFrame.mvKeys.size() << endl;
    cout << "CKPT 2" << endl;
    if (vpMPs.size() > closestIndex) {
        cv::Mat worldPoint = vpMPs[closestIndex]->GetWorldPos();
        cout << "CKPT 3" << endl;
        float worldX = worldPoint.at<float>(0);
        float worldY = worldPoint.at<float>(1);
        float worldZ = worldPoint.at<float>(2);
        cout << "CKPT 4" << endl;

        cout << "Closest point from here (squared): " << closestDistanceSquared << endl;
        cout << "Closest key point: " << closestKey.pt.x << ", " << closestKey.pt.y << endl;
        cout << setprecision(7) << "World coordinates: " << worldX << ", " << worldY << ", " << worldZ << endl << endl;
    }
}


void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    pangolin::MyHandler3D handler(s_cam);

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(&handler);
            //.SetHandler(new pangolin::Handler3D(s_cam));


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    cv::namedWindow("ORB-SLAM2: Current Frame");

    cv::setMouseCallback("ORB-SLAM2: Current Frame", IdentifyKeyPoint, mpTracker);

    bool bFollow = true;
    bool bLocalizationMode = false;

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
