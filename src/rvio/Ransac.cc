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

#include "rvio/Ransac.h"
#include "numerics.h"


namespace RVIO
{

Ransac::Ransac(bool bUseSampson,
               const double nInlierThreshold) :
    mbUseSampson(bUseSampson),
    mnInlierThreshold(nInlierThreshold) {}


void Ransac::SetPointSet(const int nInlierCandidates,
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
    // Two directional angles need to be solved: alpha, beta.
    // Two correspondences for solving it: {A0,A2} and {B0,B2}.
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
                        const Eigen::Matrix3d& R,
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
            // Store the index of inlier candidate
            mvInlierCandidateIndices.push_back(i);
            nInlierCandidates++;
        }
    }

    if (nInlierCandidates>mRansacModel.nIterations)
        SetPointSet(nInlierCandidates, mRansacModel.nIterations);
    else
        // Too few inliers
        return 0;

    int nWinnerInliersNumber = 0;
    int nWinnerHypothesisIdx = 0;
    for (int i=0; i<mRansacModel.nIterations; ++i)
    {
        // Do Ransac
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

    // Find outliers
    int nNewFoundOutliers = 0;
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
            nNewFoundOutliers++;
        }
    }

    return nInlierCandidates-nNewFoundOutliers;
}


double Ransac::SampsonError(const Eigen::Vector3d& pt1,
                            const Eigen::Vector3d& pt2,
                            const Eigen::Matrix3d& E)
{
    Eigen::Vector3d Fx1 = E*pt1;
    Eigen::Vector3d Fx2 = E.transpose()*pt2;

    return (pow(pt2.transpose()*E*pt1,2))/(pow(Fx1(0),2)+pow(Fx1(1),2)+pow(Fx2(0),2)+pow(Fx2(1),2));
}


double Ransac::AlgebraicError(const Eigen::Vector3d& pt1,
                              const Eigen::Vector3d& pt2,
                              const Eigen::Matrix3d& E)
{
    return fabs(pt2.transpose()*E*pt1);
}

} // namespace RVIO
