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

#ifndef PREINTEGRATOR_H
#define PREINTEGRATOR_H

#include <list>

#include <Eigen/Core>

#include "InputBuffer.h"


namespace RVIO
{

class PreIntegrator
{
public:

    PreIntegrator(const cv::FileStorage& fsSettings);

    void propagate(Eigen::VectorXd& xkk, Eigen::MatrixXd& Pkk, std::list<ImuData*>& lImuData);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    // Outputs
    Eigen::VectorXd xk1k;
    Eigen::MatrixXd Pk1k;

private:

    double mnGravity;
    double mnSmallAngle;

    // Sigma{g,wg,a,wa}
    double mnGyroNoiseSigma;
    double mnGyroRandomWalkSigma;
    double mnAccelNoiseSigma;
    double mnAccelRandomWalkSigma;

    Eigen::Matrix<double,12,12> ImuNoiseMatrix;
};

} // namespace RVIO

#endif
