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

#include <Eigen/Dense>

#include <opencv2/core/eigen.hpp>

#include <visualization_msgs/Marker.h>

#include "Updater.h"
#include "../util/Numerics.h"


namespace RVIO
{

static int cloud_id = 0;
std_msgs::ColorRGBA colorLandmark;
geometry_msgs::Vector3 scaleLandmark;

Updater::Updater(const cv::FileStorage& fsSettings)
{
    mnCamRate = fsSettings["Camera.fps"];

    const float nImageNoiseSigmaX = fsSettings["Camera.sigma_px"];
    const float nImageNoiseSigmaY = fsSettings["Camera.sigma_py"];
    mnImageNoiseSigma = std::max(nImageNoiseSigmaX, nImageNoiseSigmaY);

    cv::Mat T(4,4,CV_32F);
    fsSettings["Camera.T_BC0"] >> T;
    Eigen::Matrix4d Tic;
    cv::cv2eigen(T,Tic);
    mRic = Tic.block<3,3>(0,0);
    mtic = Tic.block<3,1>(0,3);
    mRci = mRic.transpose();
    mtci = -mRci*mtic;

    xk1k1.setZero(26,1);
    Pk1k1.setZero(24,24);

    mFeatPub = mUpdaterNode.advertise<visualization_msgs::Marker>("/rvio/landmarks", 1);
    mnPubRate = fsSettings["Landmark.nPubRate"];

    scaleLandmark.x = fsSettings["Landmark.nScale"];
    scaleLandmark.y = fsSettings["Landmark.nScale"];
    scaleLandmark.z = fsSettings["Landmark.nScale"];

    colorLandmark.a = 1;
    colorLandmark.r = 0;
    colorLandmark.b = 1;
    colorLandmark.g = 0;
}


void Updater::update(Eigen::VectorXd& xk1k,
                     Eigen::MatrixXd& Pk1k,
                     std::vector<unsigned char>& vFeatTypesForUpdate,
                     std::vector<std::list<cv::Point2f> >& vlFeatMeasForUpdate)
{
    // Interact with ROS rviz
    visualization_msgs::Marker cloud;
    cloud.header.frame_id = "imu";
    cloud.ns = "points";
    cloud.id = ++cloud_id;
    cloud.color = colorLandmark;
    cloud.scale = scaleLandmark;
    cloud.pose.orientation.w = 1.0;
    cloud.lifetime = ros::Duration(1/mnPubRate);
    cloud.action = visualization_msgs::Marker::ADD;
    cloud.type = visualization_msgs::Marker::POINTS;

    // Number of features
    int nFeat = (int)vFeatTypesForUpdate.size();

    // Number of (max) rows of matrices
    int nRows = 0;
    for (int i=0; i<nFeat; ++i)
        nRows += 2*(int)vlFeatMeasForUpdate.at(i).size();

    // Number of clone states
    int nCloneStates = (xk1k.rows()-26)/7;

    // Residual and Jacobian
    Eigen::VectorXd r(nRows,1);
    Eigen::MatrixXd Hx(nRows,24+6*nCloneStates);
    r.setZero();
    Hx.setZero();

    int nRowCount = 0;
    int nGoodFeatCount = 0;

    for (int featIdx=0; featIdx<nFeat; ++featIdx)
    {
        char featType = vFeatTypesForUpdate.at(featIdx);
        std::list<cv::Point2f> lFeatMeas = vlFeatMeasForUpdate.at(featIdx);

        int nTrackLength = (int)lFeatMeas.size();
        int nTrackPhases = nTrackLength-1;
        int nRelPosesDim = 7*nTrackPhases;

        Eigen::VectorXd mRelPoses;
        if (featType=='1')
            mRelPoses = xk1k.tail(nRelPosesDim);
        else
            mRelPoses = xk1k.block(26,0,nRelPosesDim,1);

        // [qIi1,tIi1]
        Eigen::VectorXd mRelPosesToFirst(nRelPosesDim,1);
        mRelPosesToFirst.block(0,0,7,1) << mRelPoses.block(0,0,4,1), -QuatToRot(mRelPoses.block(0,0,4,1))*mRelPoses.block(4,0,3,1);
        for (int i=1; i<nTrackPhases; ++i)
        {
            Eigen::Vector4d qI = QuatMul(mRelPoses.block(7*i,0,4,1),mRelPosesToFirst.block(7*(i-1),0,4,1));
            Eigen::Vector3d tI = QuatToRot(mRelPoses.block(7*i,0,4,1))*(mRelPosesToFirst.block(7*(i-1)+4,0,3,1)-mRelPoses.block(7*i+4,0,3,1));
            mRelPosesToFirst.block(7*i,0,7,1) << qI, tI;
        }

        // [qCi1,tCi1]
        Eigen::VectorXd mCamRelPosesToFirst(nRelPosesDim,1);
        for (int i=0; i<nTrackPhases; ++i)
        {
            Eigen::Vector4d qC = RotToQuat(mRci*QuatToRot(mRelPosesToFirst.block(7*i,0,4,1))*mRic);
            Eigen::Vector3d tC = mRci*QuatToRot(mRelPosesToFirst.block(7*i,0,4,1))*mtic+mRci*mRelPosesToFirst.block(7*i+4,0,3,1)+mtci;
            mCamRelPosesToFirst.block(7*i,0,7,1) << qC, tC;
        }

        //============================================
        // Feature initialization
        // Use the 1st measurement for initialization.
        cv::Point2f ptFirst = lFeatMeas.front();
        lFeatMeas.pop_front();

        // Initialize pfinv=[phi,psi,rho]'
        double phi = atan2(ptFirst.y, sqrt(pow(ptFirst.x,2)+1));
        double psi = atan2(ptFirst.x,1);
        double rho = 0.;

        if (fabs(phi)>.5*3.14 || fabs(psi)>.5*3.14)
        {
            ROS_DEBUG("Invalid inverse-depth feature estimate (0)!");
            continue;
        }

        //======================================================================
        // Feature estimate refinement
        // Use Levenberg–Marquardt (LM) algorithm with inverse-depth parameters.
        // Unit vector of pfinv
        Eigen::Vector3d epfinv;
        epfinv << cos(phi)*sin(psi), sin(phi), cos(phi)*cos(psi);

        // Jacobian of epfinv wrt. [phi,psi]'
        Eigen::Matrix<double,3,2> Jang;
        Jang << -sin(phi)*sin(psi), cos(phi)*cos(psi),
                 cos(phi), 0,
                -sin(phi)*cos(psi), -cos(phi)*sin(psi);

        Eigen::Matrix2d Rinv;
        Rinv << 1./pow(mnImageNoiseSigma,2), 0,
                0, 1./pow(mnImageNoiseSigma,2);

        int maxIter = 10;
        double lambda = 0.01;
        double lastCost = std::numeric_limits<double>::infinity();

        for (int nIter=0; nIter<maxIter; ++nIter)
        {
            Eigen::Matrix3d HTRinvH = Eigen::Matrix3d::Zero();
            Eigen::Vector3d HTRinve = Eigen::Vector3d::Zero();
            double cost = 0;

            // The 1st measurement
            Eigen::Vector3d h1 = epfinv;

            Eigen::Matrix<double,2,3> Hproj1;
            Hproj1 << 1/h1(2), 0, -h1(0)/pow(h1(2),2),
                      0, 1/h1(2), -h1(1)/pow(h1(2),2);

            Eigen::Matrix<double,2,3> H1;
            H1 << Hproj1*Jang, Eigen::Vector2d::Zero();

            cv::Point2f pt1;
            pt1.x = h1(0)/h1(2);
            pt1.y = h1(1)/h1(2);

            Eigen::Vector2d e1;
            e1 << (ptFirst-pt1).x, (ptFirst-pt1).y;

            cost += e1.transpose()*Rinv*e1;
            HTRinvH.noalias() += H1.transpose()*Rinv*H1;
            HTRinve.noalias() += H1.transpose()*Rinv*e1;

            // The following measurements
            std::list<cv::Point2f>::const_iterator lit = lFeatMeas.begin();
            for (int i=0; i<nTrackPhases; ++i, ++lit)
            {
                Eigen::Matrix3d Rc = QuatToRot(mCamRelPosesToFirst.block(7*i,0,4,1));
                Eigen::Vector3d tc = mCamRelPosesToFirst.block(7*i+4,0,3,1);
                Eigen::Vector3d h = Rc*epfinv+rho*tc;

                Eigen::Matrix<double,2,3> Hproj;
                Hproj << 1/h(2), 0, -h(0)/pow(h(2),2),
                         0, 1/h(2), -h(1)/pow(h(2),2);

                Eigen::Matrix<double,2,3> H;
                H << Hproj*Rc*Jang, Hproj*tc;

                cv::Point2f pt;
                pt.x = h(0)/h(2);
                pt.y = h(1)/h(2);

                Eigen::Vector2d e;
                e << ((*lit)-pt).x, ((*lit)-pt).y;

                cost += e.transpose()*Rinv*e;
                HTRinvH.noalias() += H.transpose()*Rinv*H;
                HTRinve.noalias() += H.transpose()*Rinv*e;
            }

            if (cost<=lastCost)
            {
                // Down
                HTRinvH.diagonal() += lambda*HTRinvH.diagonal();
                Eigen::Vector3d dpfinv = HTRinvH.colPivHouseholderQr().solve(HTRinve);

                phi += dpfinv(0);
                psi += dpfinv(1);
                rho += dpfinv(2);

                epfinv << cos(phi)*sin(psi), sin(phi), cos(phi)*cos(psi);

                Jang << -sin(phi)*sin(psi), cos(phi)*cos(psi),
                         cos(phi), 0,
                        -sin(phi)*cos(psi), -cos(phi)*sin(psi);

                if (fabs(lastCost-cost)<1e-6 && dpfinv(2)<1e-6)
                    break;

                lambda *= .1;
                lastCost = cost;
            }
            else
            {
                // Up
                lambda *= 10;
                lastCost = cost;
            }
        }

        if (fabs(phi)>.5*3.14 || fabs(psi)>.5*3.14 || std::isinf(rho) || rho<0)
        {
            ROS_DEBUG("Invalid inverse-depth feature estimate (1)!");
            continue;
        }

        if (featType=='2')
        {
            nTrackLength = std::ceil(.5*nTrackLength);
            nTrackPhases = nTrackLength-1;
        }


        //==========================================
        // Construct inverse-depth measurement model
        // i. Construct residual and Jacobians (the nonzero subblocks)
        Eigen::VectorXd tempr(2*nTrackLength,1);
        Eigen::MatrixXd tempHx(2*nTrackLength,6*nCloneStates);
        Eigen::MatrixXd tempHf(2*nTrackLength,3);
        tempr.setZero();
        tempHx.setZero();
        tempHf.setZero();

        int nStartRow = 0;
        int nStartCol;
        if (featType=='1')
            nStartCol = 6*(nCloneStates-nTrackPhases);
        else
            nStartCol = 0;

        // For the 1st measurement
        Eigen::Vector3d h1 = epfinv;

        cv::Point2f pt1;
        pt1.x = h1(0)/h1(2);
        pt1.y = h1(1)/h1(2);

        Eigen::Matrix<double,2,3> Hproj1;
        Hproj1 << 1/h1(2), 0, -h1(0)/pow(h1(2),2),
                  0, 1/h1(2), -h1(1)/pow(h1(2),2);

        // r
        cv::Point2f e1 = ptFirst-pt1;
        tempr.block(0,0,2,1) << e1.x, e1.y;

        // Hx: zero matrix

        // Hf
        Eigen::Matrix3d tempm = Eigen::Matrix3d::Zero();
        tempm.block<3,2>(0,0) = Jang;
        tempHf.block<2,3>(0,0) = Hproj1*tempm;

        nStartRow += 2;

        // For the following measurements
        std::list<cv::Point2f>::const_iterator lit = lFeatMeas.begin();
        for (int i=1; i<nTrackLength; ++i, ++lit)
        {
            Eigen::Matrix3d R = QuatToRot(mRelPosesToFirst.block(7*(i-1),0,4,1));

            Eigen::Matrix3d Rc = QuatToRot(mCamRelPosesToFirst.block(7*(i-1),0,4,1));
            Eigen::Vector3d tc = mCamRelPosesToFirst.block(7*(i-1)+4,0,3,1);
            Eigen::Vector3d h = Rc*epfinv+rho*tc;

            cv::Point2f pt;
            pt.x = h(0)/h(2);
            pt.y = h(1)/h(2);

            Eigen::Matrix<double,2,3> Hproj;
            Hproj << 1/h(2), 0, -h(0)/pow(h(2),2),
                     0, 1/h(2), -h(1)/pow(h(2),2);

            // r
            cv::Point2f e = (*lit)-pt;
            tempr.block(2*i,0,2,1) << e.x, e.y;

            // Hx: the 1st clone's subblock
            Eigen::Matrix3d R0T = QuatToRot(mRelPosesToFirst.block(0,0,4,1)).transpose();
            Eigen::Vector3d t0 = mRelPosesToFirst.block(4,0,3,1);

            Eigen::Matrix3d dpx0 = SkewSymm(mRic*epfinv+rho*mtic+rho*R0T*t0);
            Eigen::Matrix<double,3,6> subH;
            subH << dpx0*R0T, -rho*Eigen::Matrix3d::Identity();

            tempHx.block(nStartRow,nStartCol,2,6) = Hproj*mRci*R*subH;

            // Hx: the following clones' subblocks
            for (int j=1; j<i; ++j)
            {
                Eigen::Matrix3d R1T = QuatToRot(mRelPosesToFirst.block(7*j,0,4,1)).transpose();
                Eigen::Vector3d t1 = mRelPosesToFirst.block(7*j+4,0,3,1);
                Eigen::Matrix3d R2T = QuatToRot(mRelPosesToFirst.block(7*(j-1),0,4,1)).transpose();

                Eigen::Matrix3d dpx = SkewSymm(mRic*epfinv+rho*mtic+rho*R1T*t1);
                subH << dpx*R1T, -rho*R2T;

                tempHx.block(nStartRow,nStartCol+6*j,2,6) = Hproj*mRci*R*subH;
            }

            // Hf
            tempHf.block(nStartRow,0,2,3) << Hproj*Rc*Jang, Hproj*tc;

            nStartRow += 2;
        }

        // ii. Feature marginalization
        int M = nStartRow;
        int N = tempHf.cols();

        if (tempHf.col(N-1).norm()<1e-4)
        {
            ROS_DEBUG("Hf is rank deficient!");
            N--;
        }

        // Use Givens rotations (QR)
        Eigen::JacobiRotation<double> tempHf_GR;

        for (int n=0; n<N; ++n)
        {
            for (int m=M-1; m>n; m--)
            {
                // Givens matrix G
                tempHf_GR.makeGivens(tempHf(m-1,n), tempHf(m,n));

                // Multiply G' to the corresponding lines (m-1,m) in each matrix
                // Note: we only apply G' to the nonzero cols [n:N-1], which is
                //       equivalent to applying G' to the entire row [0:N-1].
                // G'*Hf
                (tempHf.block(m-1,n,2,N-n)).applyOnTheLeft(0,1,tempHf_GR.adjoint());

                // G'*Hx
                (tempHx.block(m-1,0,2,tempHx.cols())).applyOnTheLeft(0,1,tempHf_GR.adjoint());

                // G'*r
                (tempr.block(m-1,0,2,1)).applyOnTheLeft(0,1,tempHf_GR.adjoint());
            }
        }

        // iii. Mahalanobis distance test
        // Note: this tests the dissimilarity between the measurement and the estimate,
        //       D=r'*Sinv*r, where Sinv*r is the solution of Sx=r, and S=H*P*H'+R.
        int nDOF = M-N;
        Eigen::VectorXd tempr_ = tempr.block(N,0,nDOF,1);
        Eigen::MatrixXd tempHx_ = tempHx.block(N,0,nDOF,tempHx.cols());

        Eigen::VectorXd tempR;
        tempR.setOnes(nDOF,1);
        tempR *= pow(mnImageNoiseSigma, 2);

        Eigen::MatrixXd tempS;
        tempS = tempHx_*Pk1k.block(24,24,6*nCloneStates,6*nCloneStates)*(tempHx_.transpose());
        tempS.diagonal() += tempR;
        tempS = .5*(tempS+tempS.transpose());

        double nMahalanobisDist = (tempr_.transpose()*(tempS.colPivHouseholderQr().solve(tempr_))).norm();

        if (nMahalanobisDist<CHI_THRESHOLD[nDOF-1])
        {
            r.block(nRowCount,0,nDOF,1) = tempr_;
            Hx.block(nRowCount,24,nDOF,6*nCloneStates) = tempHx_;

            nRowCount += nDOF;
            nGoodFeatCount++;

            if (rho>0)
            {
                // Feature visualization (rviz)
                // Note: pf is in the current reference frame {Rk}.
                Eigen::VectorXd posek = mRelPosesToFirst.tail(7);
                Eigen::Matrix3d Rk = QuatToRot(posek.head(4));
                Eigen::Vector3d tk = posek.tail(3);

                // pf in {C1}->{R1}->{Rk}
                Eigen::Vector3d pfc = 1/rho*epfinv;
                Eigen::Vector3d pf1 = mRic*pfc+mtic;
                Eigen::Vector3d pfk = Rk*pf1+tk;

                geometry_msgs::Point feat;
                feat.x = pfk(0);
                feat.y = pfk(1);
                feat.z = pfk(2);
                cloud.points.push_back(feat);
            }
        }
        else
        {
            ROS_DEBUG("Failed in Mahalanobis distance test!");
            continue;
        }
    }

    // Visualize features in rviz
    mFeatPub.publish(cloud);

    if (nGoodFeatCount>2)
    {
        Eigen::VectorXd ro = r.block(0,0,nRowCount,1);
        Eigen::MatrixXd Ho = Hx.block(0,0,nRowCount,Hx.cols());

        Eigen::VectorXd Ro;
        Ro.setOnes(nRowCount,1);
        Ro *= pow(mnImageNoiseSigma, 2);

        // Model compression
        Eigen::VectorXd rn;
        Eigen::MatrixXd Hn;
        Eigen::VectorXd Rn;

        if (Ho.rows()>Ho.cols()-24)
        {
            // If Hw is a tall matrix
            int M = Ho.rows();
            int N = Ho.cols()-24;

            Eigen::MatrixXd tempHw = Ho.block(0,24,M,N);

            for (int i=N; i>0; i--)
            {
                if (tempHw.col(i-1).norm()==0)
                {
                    ROS_DEBUG("Hw is rank deficient!");
                    N--;
                }
                else
                    break;
            }

            // Use Givens rotations (QR)
            Eigen::JacobiRotation<double> tempHw_GR;

            for (int n=0; n<N; ++n)
            {
                for (int m=M-1; m>n; m--)
                {
                    // Givens matrix G
                    tempHw_GR.makeGivens(tempHw(m-1,n), tempHw(m,n));

                    // Multiply G' to the corresponding lines (m-1,m) in each matrix
                    // Note: we only apply G' to the nonzero cols [n:N-1], which is
                    //       equivalent to applying G' to the entire row [0:N-1].
                    // G'*H
                    (tempHw.block(m-1,n,2,N-n)).applyOnTheLeft(0,1,tempHw_GR.adjoint());

                    // G'*r
                    (ro.block(m-1,0,2,1)).applyOnTheLeft(0,1,tempHw_GR.adjoint());
                }
            }

            Ho.block(0,24,M,N) = tempHw.block(0,0,M,N);

            int nRank = 0;
            for (int i=0; i<M; ++i)
            {
                if (Ho.row(i).norm()<1e-4)
                    break;
                else
                    nRank++;
            }

            rn = ro.block(0,0,nRank,1);
            Hn = Ho.block(0,0,nRank,Ho.cols());
            Rn.setOnes(nRank,1);
            Rn *= pow(mnImageNoiseSigma, 2);
        }
        else
        {
            // If Hw is a fat matrix
            rn = ro;
            Hn = Ho;
            Rn = Ro;
        }

        //===========
        // EKF update
        Eigen::MatrixXd S = Hn*Pk1k*(Hn.transpose());
        S.diagonal() += Rn;
        S = .5*(S+S.transpose());
        Eigen::MatrixXd K = Pk1k*(Hn.transpose())*(S.inverse());
        Eigen::VectorXd dx = K*rn;

        xk1k1.resize(xk1k.rows(),1);

        // xG
        Eigen::Vector4d dqG;
        dqG(0) = .5*dx(0);
        dqG(1) = .5*dx(1);
        dqG(2) = .5*dx(2);

        double dqGvn = (dqG.head(3)).norm();
        if (dqGvn<1)
        {
            dqG(3) = sqrt(1-pow(dqGvn,2));
        }
        else
        {
            dqG.head(3) *= (1/sqrt(1+pow(dqGvn,2)));
            dqG(3) = 1/sqrt(1+pow(dqGvn,2));
        }

        xk1k1.block(0,0,4,1) = QuatMul(dqG,xk1k.block(0,0,4,1));
        xk1k1.block(4,0,6,1) = dx.block(3,0,6,1)+xk1k.block(4,0,6,1);

        Eigen::Vector3d g = xk1k1.block(7,0,3,1);
        g.normalize();
        xk1k1.block(7,0,3,1) = g;

        // xR
        Eigen::Vector4d dqR;
        dqR(0) = .5*dx(9);
        dqR(1) = .5*dx(10);
        dqR(2) = .5*dx(11);

        double dqRvn = (dqR.head(3)).norm();
        if (dqRvn<1)
        {
            dqR(3) = sqrt(1-pow(dqRvn,2));
        }
        else
        {
            dqR.head(3) *= (1/sqrt(1+pow(dqRvn,2)));
            dqR(3) = 1/sqrt(1+pow(dqRvn,2));
        }

        xk1k1.block(10,0,4,1) = QuatMul(dqR,xk1k.block(10,0,4,1));
        xk1k1.block(14,0,12,1) = dx.block(12,0,12,1)+xk1k.block(14,0,12,1);

        // xW
        for (int poseIdx=0; poseIdx<nCloneStates; ++poseIdx)
        {
            Eigen::Vector4d dqc;
            dqc(0) = .5*dx(24+6*poseIdx);
            dqc(1) = .5*dx(24+6*poseIdx+1);
            dqc(2) = .5*dx(24+6*poseIdx+2);

            double dqcvn = (dqc.head(3)).norm();
            if (dqcvn<1)
            {
                dqc(3) = sqrt(1-pow(dqcvn,2));
            }
            else
            {
                dqc.head(3) *= (1/sqrt(1+pow(dqcvn,2)));
                dqc(3) = 1/sqrt(1+pow(dqcvn,2));
            }

            xk1k1.block(26+7*poseIdx,0,4,1) = QuatMul(dqc,xk1k.block(26+7*poseIdx,0,4,1));
            xk1k1.block(26+7*poseIdx+4,0,3,1) = dx.block(24+6*poseIdx+3,0,3,1)+xk1k.block(26+7*poseIdx+4,0,3,1);
        }

        Eigen::MatrixXd I_ = Eigen::MatrixXd::Identity(Pk1k.rows(),Pk1k.cols());
        Eigen::MatrixXd I_KH = I_-K*Hn;
        Pk1k1 = I_KH*Pk1k*(I_KH.transpose());
        Pk1k1 += Rn(0)*K*(K.transpose());
        Pk1k1 = .5*(Pk1k1+Pk1k1.transpose());
    }
    else
    {
        ROS_DEBUG("Too few measurements for update!");

        xk1k1 = xk1k;
        Pk1k1 = Pk1k;
    }
}

} // namespace RVIO
