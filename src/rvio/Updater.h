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

#ifndef UPDATER_H
#define UPDATER_H

#include <list>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>


namespace RVIO
{

class Updater
{
public:

    Updater(const cv::FileStorage& fsSettings);

    void update(Eigen::VectorXd& xk1k, Eigen::MatrixXd& Pk1k, std::vector<unsigned char>& vFeatTypesForUpdate,
                std::vector<std::list<cv::Point2f> >& vlFeatMeasForUpdate);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    // Outputs
    Eigen::VectorXd xk1k1;
    Eigen::MatrixXd Pk1k1;

private:

    double mnCamRate;

    // Sigma{pixel}
    double mnImageNoiseSigma;

    // Extrinsics
    Eigen::Matrix3d mRic;
    Eigen::Vector3d mtic;
    Eigen::Matrix3d mRci;
    Eigen::Vector3d mtci;

    // Interact with rviz
    ros::NodeHandle mUpdaterNode;
    ros::Publisher mFeatPub;
    double mnPubRate;
};

} // namespace RVIO

#endif
