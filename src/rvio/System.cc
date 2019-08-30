/**
* This file is part of R-VIO.
*
* Copyright (C) 2019 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO> 
*
* R-VIO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO. If not, see <http://www.gnu.org/licenses/>.
*/

#include <fstream>

#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "rvio/System.h"
#include "numerics.h"


namespace RVIO
{

nav_msgs::Path path;

std::ofstream fPoseResults;

System::System(const std::string& strSettingsFile)
{
    // Output welcome message
    std::cout << "\n" <<
    "R-VIO: Robocentric visual-inertial odometry" << "\n" << "\n"
    "Copyright (C) 2019 Zheng Huai and Guoquan Huang" << "\n" <<
    "Robot Perception and Navigation Group, University of Delaware." << "\n" << "\n";

    // Read settings file
    cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
    {
       ROS_ERROR("Failed to open settings file at: %s", strSettingsFile.c_str());
       exit(-1);
    }

    mnImuRate = fsSettings["IMU.dps"];

    msigmaGyroNoise = fsSettings["IMU.sigma_g"];
    msigmaGyroBias = fsSettings["IMU.sigma_wg"];
    msigmaAccelNoise = fsSettings["IMU.sigma_a"];
    msigmaAccelBias = fsSettings["IMU.sigma_wa"];

    mnGravity = fsSettings["IMU.nG"];

    mnCamTimeOffset = fsSettings["Camera.nTimeOffset"];

    const int nMaxTrackingLength = fsSettings["Tracker.nMaxTrackingLength"];
    mnSlidingWindowSize = nMaxTrackingLength-1;

    const int nMinTrackingLength = fsSettings["Tracker.nMinTrackingLength"];
    mnMinCloneStates = nMinTrackingLength-1;

    const int bEnableAlignment = fsSettings["INI.EnableAlignment"];
    mbEnableAlignment = bEnableAlignment;

    const int bRecordOutputs = fsSettings["INI.RecordOutputs"];
    mbRecordOutputs = bRecordOutputs;

    if (mbRecordOutputs)
    {
        std::string pkg_path = ros::package::getPath("rvio");
        fPoseResults.open(pkg_path+"/stamped_pose_ests.dat", std::ofstream::out | std::ofstream::app);
    }

    mnThresholdAngle = fsSettings["INI.nThresholdAngle"];
    mnThresholdDispl = fsSettings["INI.nThresholdDispl"];

    mbIsMoving = false;
    mbIsReady = false;

    mpTracker = new Tracker(fsSettings);
    mpUpdater = new Updater(fsSettings);
    mpPreIntegrator = new PreIntegrator(fsSettings);
    mpSensorDatabase = new SensorDatabase();

    mPathPub = mSystemNode.advertise<nav_msgs::Path>("/rvio/trajectory", 1);
}


System::~System()
{
    delete mpTracker;
    delete mpUpdater;
    delete mpPreIntegrator;
    delete mpSensorDatabase;
}


void System::initialize(const Eigen::Vector3d& w, const Eigen::Vector3d& a,
                        const int nImuData, const bool bEnableAlignment)
{
    Eigen::Vector3d g = a;
    g.normalize();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (bEnableAlignment)
    {
        // i. Align z-axis with gravity
        Eigen::Vector3d zv = g;

        // ii. Make x-axis perpendicular to z-axis
        Eigen::Vector3d ex = Eigen::Vector3d(1,0,0);
        Eigen::Vector3d xv = ex-zv*zv.transpose()*ex;
        xv.normalize();

        // iii. Get y-axis
        Eigen::Vector3d yv = SkewSymm(zv)*xv;
        yv.normalize();

        // iv. The orientation of {G} in {R0}
        Eigen::Matrix3d tempR;
        tempR << xv, yv, zv;
        R = tempR;
    }

    xkk.setZero(26,1);
    xkk.block(0,0,4,1) = RotToQuat(R);
    xkk.block(7,0,3,1) = g;

    if (nImuData>1)
    {
        xkk.block(20,0,3,1) = w; // bg
        xkk.block(23,0,3,1) = a-mnGravity*g; // ba
    }

    double dt = 1/mnImuRate;

    Pkk.setZero(24,24);
    Pkk(0,0) = pow(1e-3,2); // qG
    Pkk(1,1) = pow(1e-3,2);
    Pkk(2,2) = pow(1e-3,2);
    Pkk(3,3) = pow(1e-3,2); // pG
    Pkk(4,4) = pow(1e-3,2);
    Pkk(5,5) = pow(1e-3,2);
    Pkk(6,6) = nImuData*dt*pow(msigmaAccelNoise,2); // g
    Pkk(7,7) = nImuData*dt*pow(msigmaAccelNoise,2);
    Pkk(8,8) = nImuData*dt*pow(msigmaAccelNoise,2);
    Pkk(18,18) = nImuData*dt*pow(msigmaGyroBias,2); // bg
    Pkk(19,19) = nImuData*dt*pow(msigmaGyroBias,2);
    Pkk(20,20) = nImuData*dt*pow(msigmaGyroBias,2);
    Pkk(21,21) = nImuData*dt*pow(msigmaAccelBias,2); // ba
    Pkk(22,22) = nImuData*dt*pow(msigmaAccelBias,2);
    Pkk(23,23) = nImuData*dt*pow(msigmaAccelBias,2);
}


void System::MonoVIO(const cv::Mat& im, const double& timestamp)
{
    static int nCloneStates = 0;
    static int nImageCountAfterInit = 0;

    std::list<ImuData*> lImuDataSeq;
    if (mpSensorDatabase->GetImuDataByTimestamp(timestamp+mnCamTimeOffset, lImuDataSeq)==0)
        return;


    /**
     * Initialization
     */
    if (!mbIsReady)
    {
        static Eigen::Vector3d wm = Eigen::Vector3d::Zero();
        static Eigen::Vector3d am = Eigen::Vector3d::Zero();
        static int nImuDataCount = 0;

        if (!mbIsMoving)
        {
            Eigen::Vector3d ang;
            Eigen::Vector3d vel;
            Eigen::Vector3d displ;
            ang.setZero();
            vel.setZero();
            displ.setZero();

            for (std::list<ImuData*>::const_iterator lit=lImuDataSeq.begin();
                 lit!=lImuDataSeq.end(); ++lit)
            {
                Eigen::Vector3d w = (*lit)->AngularVel;
                Eigen::Vector3d a = (*lit)->LinearAccel;
                double dt = (*lit)->TimeInterval;

                a -= mnGravity*a/a.norm();

                ang += dt*w;
                vel += dt*a;
                displ += dt*vel+.5*pow(dt,2)*a;
            }

            // If the change is larger than the threshold
            if (ang.norm()>mnThresholdAngle || displ.norm()>mnThresholdDispl)
                mbIsMoving = true;
        }

        while (!lImuDataSeq.empty())
        {
            if (!mbIsMoving)
            {
                wm += (lImuDataSeq.front())->AngularVel;
                am += (lImuDataSeq.front())->LinearAccel;
                lImuDataSeq.pop_front();
                nImuDataCount++;
            }
            else
            {
                if (nImuDataCount==0)
                {
                    wm = (lImuDataSeq.front())->AngularVel;
                    am = (lImuDataSeq.front())->LinearAccel;
                    nImuDataCount = 1;
                }
                else
                {
                    wm = wm/nImuDataCount;
                    am = am/nImuDataCount;
                }

                initialize(wm, am, nImuDataCount, mbEnableAlignment);

                mbIsReady = true;
                break;
            }
        }

        if (!mbIsReady)
            return;
    }

    nImageCountAfterInit++;


    /**
     * Visual tracking & Propagation
     */
    boost::thread thdTracking(&Tracker::track, mpTracker, std::ref(im), std::ref(lImuDataSeq));
    boost::thread thdPropagate(&PreIntegrator::propagate, mpPreIntegrator, std::ref(xkk), std::ref(Pkk), std::ref(lImuDataSeq));

    thdTracking.join();
    thdPropagate.join();


    /**
     * Update
     */
    if (nCloneStates>mnMinCloneStates)
    {
        mpUpdater->update(mpPreIntegrator->xk1k, mpPreIntegrator->Pk1k, mpTracker->mvFeatTypesForUpdate, mpTracker->mvlFeatMeasForUpdate);

        xkk = mpUpdater->xk1k1;
        Pkk = mpUpdater->Pk1k1;
    }
    else
    {
        xkk = mpPreIntegrator->xk1k;
        Pkk = mpPreIntegrator->Pk1k;
    }


    /**
     * State augmentation
     */
    if (nImageCountAfterInit>1)
    {
        if (nCloneStates<mnSlidingWindowSize)
        {
            // xkk
            Eigen::MatrixXd tempx(26+7*(nCloneStates+1),1);
            tempx << xkk, xkk.block(10,0,7,1);
            xkk = tempx;

            // Pkk
            Eigen::MatrixXd J(24+6*(nCloneStates+1),24+6*nCloneStates);
            J.setZero();
            J.block(0,0,24+6*nCloneStates,24+6*nCloneStates).setIdentity();
            J.block(24+6*nCloneStates,9,3,3).setIdentity();
            J.block(24+6*nCloneStates+3,12,3,3).setIdentity();

            Eigen::MatrixXd tempP = J*Pkk*(J.transpose());
            tempP = .5*(tempP+tempP.transpose());
            Pkk = tempP;

            nCloneStates++;
        }
        else
        {
            // xkk
            Eigen::MatrixXd tempx(26+7*mnSlidingWindowSize,1);
            tempx << xkk.block(0,0,26,1), xkk.block(26+7,0,7*(mnSlidingWindowSize-1),1), xkk.block(10,0,7,1);
            xkk = tempx;

            // Pkk
            Eigen::MatrixXd J(24+6*(mnSlidingWindowSize+1),24+6*mnSlidingWindowSize);
            J.setZero();
            J.block(0,0,24+6*mnSlidingWindowSize,24+6*mnSlidingWindowSize).setIdentity();
            J.block(24+6*mnSlidingWindowSize,9,3,3).setIdentity();
            J.block(24+6*mnSlidingWindowSize+3,12,3,3).setIdentity();

            Eigen::MatrixXd tempP = J*Pkk*(J.transpose());
            tempP = .5*(tempP+tempP.transpose());
            Pkk.block(0,0,24,24) = tempP.block(0,0,24,24);
            Pkk.block(0,24,24,6*mnSlidingWindowSize) = tempP.block(0,24+6,24,6*mnSlidingWindowSize);
            Pkk.block(24,0,6*mnSlidingWindowSize,24) = tempP.block(24+6,0,6*mnSlidingWindowSize,24);
            Pkk.block(24,24,6*mnSlidingWindowSize,6*mnSlidingWindowSize) = tempP.block(24+6,24+6,6*mnSlidingWindowSize,6*mnSlidingWindowSize);
        }
    }


    /**
     * Composition
     */
    Eigen::Vector4d qG = xkk.block(0,0,4,1);
    Eigen::Vector3d pG = xkk.block(4,0,3,1);
    Eigen::Vector3d gk = xkk.block(7,0,3,1);
    Eigen::Vector4d qk = xkk.block(10,0,4,1);
    Eigen::Vector3d pk = xkk.block(14,0,3,1);

    Eigen::Matrix3d RG = QuatToRot(qG);
    Eigen::Matrix3d Rk = QuatToRot(qk);

    gk = Rk*gk;
    gk.normalize();

    Eigen::Vector4d qkG = QuatMul(qk,qG);
    Eigen::Vector3d pkG = Rk*(pG-pk);
    Eigen::Vector3d pGk = RG.transpose()*(pk-pG);

    if (mbRecordOutputs)
    {
        fPoseResults << std::setprecision(19) << timestamp << " "
                     << pGk(0) << " " << pGk(1) << " " << pGk(2) << " "
                     << qkG(0) << " " << qkG(1) << " " << qkG(2) << " " << qkG(3) << "\n";
        fPoseResults.flush();
    }

    // Pk
    Eigen::Matrix<double,24,24> Vk;
    Vk.setZero();
    Vk.block(0,0,3,3) = Rk;
    Vk.block(0,9,3,3).setIdentity();
    Vk.block(3,3,3,3) = Rk;
    Vk.block(3,9,3,3) = SkewSymm(pkG);
    Vk.block(3,12,3,3) = -Rk;
    Vk.block(6,6,3,3) = Rk;
    Vk.block(6,9,3,3) = SkewSymm(gk);
    Vk.block(15,15,9,9).setIdentity();

    Pkk.block(0,0,24,24) = Vk*Pkk.block(0,0,24,24)*(Vk.transpose());
    Pkk.block(0,24,24,6*nCloneStates) = Vk*Pkk.block(0,24,24,6*nCloneStates);
    Pkk.block(24,0,6*nCloneStates,24) = Pkk.block(0,24,24,6*nCloneStates).transpose();
    Pkk = .5*(Pkk+Pkk.transpose());

    // xk
    xkk.block(0,0,4,1) = qkG;
    xkk.block(4,0,3,1) = pkG;
    xkk.block(7,0,3,1) = gk;
    xkk.block(10,0,4,1) = Eigen::Vector4d(0,0,0,1);
    xkk.block(14,0,3,1) = Eigen::Vector3d(0,0,0);

    ROS_INFO("qkG: %5f, %5f, %5f, %5f", qkG(0), qkG(1), qkG(2), qkG(3));
    ROS_INFO("pGk: %5f, %5f, %5f\n", pGk(0), pGk(1), pGk(2));


    /**
     * Interact with ROS rviz
     */
    // Broadcast tf
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "imu";
    transformStamped.transform.translation.x = pGk(0);
    transformStamped.transform.translation.y = pGk(1);
    transformStamped.transform.translation.z = pGk(2);
    transformStamped.transform.rotation.x = qkG(0);
    transformStamped.transform.rotation.y = qkG(1);
    transformStamped.transform.rotation.z = qkG(2);
    transformStamped.transform.rotation.w = qkG(3);

    mTfPub.sendTransform(transformStamped);

    // Visualize the trajectory
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "world";
    pose.pose.position.x = pGk(0);
    pose.pose.position.y = pGk(1);
    pose.pose.position.z = pGk(2);
    pose.pose.orientation.x = qkG(0);
    pose.pose.orientation.y = qkG(1);
    pose.pose.orientation.z = qkG(2);
    pose.pose.orientation.w = qkG(3);

    path.header.frame_id = "world";
    path.poses.push_back(pose);

    mPathPub.publish(path);

    usleep(1000);
}

} //namespace RVIO
