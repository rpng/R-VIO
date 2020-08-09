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

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>

#include "Ransac.h"
#include "../util/Numerics.h"


namespace RVIO
{

Ransac::Ransac(const cv::FileStorage& fsSettings)
{
    const int bUseSampson = fsSettings["Tracker.UseSampson"];
    mbUseSampson = bUseSampson;

    mnInlierThreshold = fsSettings["Tracker.nInlierThrd"];

    mnSmallAngle = fsSettings["IMU.nSmallAngle"];

    cv::Mat T(4,4,CV_32F);
    fsSettings["Camera.T_BC0"] >> T;
    Eigen::Matrix4d Tic;
    cv::cv2eigen(T,Tic);
    mRic = Tic.block<3,3>(0,0);
    mRci = mRic.transpose();
}


void Ransac::SetPointPair(const int nInlierCandidates,
                          const int nIterations)
{
    std::vector<int> vIndices(nInlierCandidates);
    for (int i=0; i<nInlierCandidates; ++i)
        vIndices.at(i) = i;

    int nIter = 0;
    for (;;)
    {
        int idxA, idxB;
        do
        {
            idxA = rand()%nInlierCandidates;
        }
        while (vIndices.at(idxA)==-1);

        do
        {
            idxB = rand()%nInlierCandidates;
        }
        while (vIndices.at(idxB)==-1 || idxA==idxB);

        mRansacModel.twoPoints(nIter,0) = mvInlierCandidateIndices.at(vIndices.at(idxA));
        mRansacModel.twoPoints(nIter,1) = mvInlierCandidateIndices.at(vIndices.at(idxB));

        vIndices.at(idxA) = -1;
        vIndices.at(idxB) = -1;
        nIter++;

        if (nIter==nIterations)
            break;
    }
}


void Ransac::SetRansacModel(const Eigen::MatrixXd& Points1,
                            const Eigen::MatrixXd& Points2,
                            const Eigen::Matrix3d& R,
                            const int nIterNum)
{
    // point(i)(j): feature i=(A,B) in reference frame j=(1,2)
    Eigen::Vector3d pointA1 = Points1.col(mRansacModel.twoPoints(nIterNum,0));
    Eigen::Vector3d pointA2 = Points2.col(mRansacModel.twoPoints(nIterNum,0));
    Eigen::Vector3d pointB1 = Points1.col(mRansacModel.twoPoints(nIterNum,1));
    Eigen::Vector3d pointB2 = Points2.col(mRansacModel.twoPoints(nIterNum,1));

    // p0= R*p1
    Eigen::Vector3d pointA0 = R*pointA1;
    Eigen::Vector3d pointB0 = R*pointB1;

    // The solution of (p2^T)[tx]p0=0, where
    // t is the function of two directional angles: alpha and beta.
    // We need two correspondences for solving t: {A0,A2} and {B0,B2}.
    double c1 = pointA2(0)*pointA0(1)-pointA0(0)*pointA2(1);
    double c2 = pointA0(1)*pointA2(2)-pointA2(1)*pointA0(2);
    double c3 = pointA2(0)*pointA0(2)-pointA0(0)*pointA2(2);
    double c4 = pointB2(0)*pointB0(1)-pointB0(0)*pointB2(1);
    double c5 = pointB0(1)*pointB2(2)-pointB2(1)*pointB0(2);
    double c6 = pointB2(0)*pointB0(2)-pointB0(0)*pointB2(2);

    double alpha = atan2(c3*c5-c2*c6,c1*c6-c3*c4);
    double beta = atan2(-c3,c1*sin(alpha)+c2*cos(alpha));
    Eigen::Vector3d t = Eigen::Vector3d(sin(beta)*cos(alpha),cos(beta),-sin(beta)*sin(alpha));

    // Add result to the RANSAC model
    mRansacModel.hypotheses.block<3,3>(3*nIterNum,0) = SkewSymm(t)*R;
}


void Ransac::GetRotation(std::list<ImuData*>& lImuData,
                         Eigen::Matrix3d& R)
{
    Eigen::Matrix3d tempR;
    tempR.setIdentity();

    Eigen::Matrix3d I;
    I.setIdentity();

    for (std::list<ImuData*>::const_iterator lit=lImuData.begin();
         lit!=lImuData.end(); ++lit)
    {
        Eigen::Vector3d wm = (*lit)->AngularVel;
        double dt = (*lit)->TimeInterval;

        bool bIsSmallAngle = false;
        if (wm.norm()<mnSmallAngle)
            bIsSmallAngle = true;

        double w1 = wm.norm();
        double wdt = w1*dt;
        Eigen::Matrix3d wx = SkewSymm(wm);
        Eigen::Matrix3d wx2 = wx*wx;

        Eigen::Matrix3d deltaR;
        if (bIsSmallAngle)
            deltaR = I-dt*wx+(.5*pow(dt,2))*wx2;
        else
            deltaR = I-(sin(wdt)/w1)*wx+((1-cos(wdt))/pow(w1,2))*wx2;
        assert(std::isnan(deltaR.norm())!=true);

        tempR = deltaR*tempR;
    }

    R = mRci*tempR*mRic;
}


void Ransac::CountInliers(const Eigen::MatrixXd& Points1,
                          const Eigen::MatrixXd& Points2,
                          const int nIterNum)
{
    // Use all inlier candidates to test the nIterNum-th hypothesis
    for (std::vector<int>::const_iterator vit=mvInlierCandidateIndices.begin();
         vit!=mvInlierCandidateIndices.end(); ++vit)
    {
        int idx = *vit;

        double nDistance;
        if (mbUseSampson)
            nDistance = SampsonError(Points1.col(idx), Points2.col(idx), mRansacModel.hypotheses.block<3,3>(3*nIterNum,0));
        else
            nDistance = AlgebraicError(Points1.col(idx), Points2.col(idx), mRansacModel.hypotheses.block<3,3>(3*nIterNum,0));

        if (nDistance<mnInlierThreshold)
            mRansacModel.nInliers(nIterNum) += 1;
    }
}


int Ransac::FindInliers(const Eigen::MatrixXd& Points1,
                        const Eigen::MatrixXd& Points2,
                        std::list<ImuData*>& lImuData,
                        std::vector<unsigned char>& vInlierFlag)
{
    mRansacModel.hypotheses.setZero();
    mRansacModel.nInliers.setZero();
    mRansacModel.twoPoints.setZero();

    mvInlierCandidateIndices.clear();

    int nInlierCandidates = 0;
    for (int i=0; i<(int)vInlierFlag.size(); ++i)
    {
        if (vInlierFlag.at(i))
        {
            mvInlierCandidateIndices.push_back(i);
            nInlierCandidates++;
        }
    }

    if (nInlierCandidates>mRansacModel.nIterations)
        SetPointPair(nInlierCandidates, mRansacModel.nIterations);
    else
        // Too few inliers
        return 0;

    Eigen::Matrix3d R;
    GetRotation(lImuData, R);

    int nWinnerInliersNumber = 0;
    int nWinnerHypothesisIdx = 0;
    for (int i=0; i<mRansacModel.nIterations; ++i)
    {
        SetRansacModel(Points1, Points2, R, i);
        CountInliers(Points1, Points2, i);

        // Find the most-voted hypothesis
        if (mRansacModel.nInliers(i)>nWinnerInliersNumber)
        {
            nWinnerInliersNumber = mRansacModel.nInliers(i);
            nWinnerHypothesisIdx = i;
        }
    }

    Eigen::Matrix3d WinnerE = mRansacModel.hypotheses.block<3,3>(3*nWinnerHypothesisIdx,0);

    int nNewOutliers = 0;
    for (int i=0; i<nInlierCandidates; ++i)
    {
        int idx = mvInlierCandidateIndices.at(i);

        double nDistance;
        if (mbUseSampson)
            nDistance = SampsonError(Points1.col(idx), Points2.col(idx), WinnerE);
        else
            nDistance = AlgebraicError(Points1.col(idx), Points2.col(idx), WinnerE);

        if (nDistance>mnInlierThreshold || std::isnan(nDistance))
        {
            // Mark as outlier
            vInlierFlag.at(idx) = 0;
            nNewOutliers++;
        }
    }

    return nInlierCandidates-nNewOutliers;
}


double Ransac::SampsonError(const Eigen::Vector3d& pt1,
                            const Eigen::Vector3d& pt2,
                            const Eigen::Matrix3d& E) const
{
    Eigen::Vector3d Fx1 = E*pt1;
    Eigen::Vector3d Fx2 = E.transpose()*pt2;

    return (pow(pt2.transpose()*E*pt1,2))/(pow(Fx1(0),2)+pow(Fx1(1),2)+pow(Fx2(0),2)+pow(Fx2(1),2));
}


double Ransac::AlgebraicError(const Eigen::Vector3d& pt1,
                              const Eigen::Vector3d& pt2,
                              const Eigen::Matrix3d& E) const
{
    return fabs(pt2.transpose()*E*pt1);
}

} // namespace RVIO
