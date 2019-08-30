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

#ifndef RANSAC_H
#define RANSAC_H

#include <vector>

#include <Eigen/Core>

#include "SensorDatabase.h"


namespace RVIO
{

/**
 * 2-Point RANSAC model
 *
 * @do @p nIterations (min. 16) trials to find the best hypothesis (of essential matrix).
 * @all the hypotheses (of essential matrix E) are stored in @p hypotheses.
 * @the number of inliers for each hypothesis is stored in @p nInliers.
 * @the indices of two correspondences are stored in @p twoPoints.
 */
struct RansacModel
{
    Eigen::MatrixXd hypotheses;
    Eigen::MatrixXi nInliers;
    Eigen::MatrixXi twoPoints;
    int nIterations;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RansacModel()
    {
        nIterations = 16;
        hypotheses.resize(nIterations*3,3);
        nInliers.resize(nIterations,1);
        twoPoints.resize(nIterations,2);
    }
};


class Ransac
{
public:

    Ransac(const cv::FileStorage& fsSettings);

    /**
     * @create a set of random 2-point indices for RANSAC.
     * @the number of inliers is @p nInlierCandidates.
     * @the number of iterations needed is @p nIterations.
     */
    void SetPointSet(const int nInlierCandidates, const int nIterations);

    /**
     * @use two (normalized) correspondences point(i)(j), where seq i=(A,B) and
     *  reference frame j = (1,2).
     * @use @p R to rotate the points in frame 1 to frame 0 having the same orientation as frame 2.
     * @compute essential matrix by solving a problem with unknow translation only, (p2^T)[tx]p0=0.
     */
    void SetRansacModel(const Eigen::MatrixXd& Points1, const Eigen::MatrixXd& Points2,
                        const Eigen::Matrix3d& R, const int nIterNum);

    /**
     * @get rotation matrix @p R by integrating the IMU (gyro) measurements.
     */
    void GetRotation(std::list<ImuData*>& lImuData, Eigen::Matrix3d& R);

    /**
     * @count the number of inliers in the @nIterNum-th trial.
     */
    void CountInliers(const Eigen::MatrixXd& Points1, const Eigen::MatrixXd& Points2,
                      const int nIterNum);

    /**
     * 2-Point RANSAC 
     *
     * @the two sets used are @p Points1 and @p Points2.
     * @the original inlier flag vector @p vInlierFlag is obtained from the tracking,
     *  i.e. @p status of function cv::calcOpticalFlowPyrLK().
     * @the output is the refined flag vector @p vInlierFlag.
     */
    int FindInliers(const Eigen::MatrixXd& Points1, const Eigen::MatrixXd& Points2,
                    std::list<ImuData*>& lImuData, std::vector<unsigned char>& vInlierFlag);

    /**
     * Metric for evaluation
     */
    double SampsonError(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2,
                        const Eigen::Matrix3d& E);

    double AlgebraicError(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2,
                          const Eigen::Matrix3d& E);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

    bool mbUseSampson;

    double mnInlierThreshold;

    double mnSmallAngle;

    // Extrinsics
    Eigen::Matrix3d mRci;
    Eigen::Matrix3d mRic;

    RansacModel mRansacModel;

    std::vector<int> mvInlierCandidateIndices;
};

} // namespace RVIO

#endif
