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

#ifndef TRACKER_H
#define TRACKER_H

#include <list>
#include <vector>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "CornerDetector.h"
#include "CornerCluster.h"
#include "Ransac.h"


namespace RVIO
{

class Tracker
{
public:

    Tracker(const cv::FileStorage& fsSettings);

    ~Tracker();

    void track(const cv::Mat& im, std::list<ImuData*>& plImuData);

    void UndistortAndNormalize(const int N, std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst);

    void DisplayTrack(const cv::Mat& imIn, std::vector<cv::Point2f>& vPoints1, std::vector<cv::Point2f>& vPoints2,
                      std::vector<unsigned char>& vInlierFlag, cv_bridge::CvImage& imOut);

    void DisplayNewer(const cv::Mat& imIn, std::vector<cv::Point2f>& vFeats, std::vector<cv::Point2f>& vNewFeats,
                      cv_bridge::CvImage& imOut);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

    // Feature types for update
    // '1': lose track
    // '2': reach the max. tracking length
    std::vector<unsigned char> mvFeatTypesForUpdate;

    // Feature measurements for update
    // Each list corresponds to a feature with its size the tracking length.
    std::vector<std::list<cv::Point2f> > mvlFeatMeasForUpdate;

private:

    int mnMaxFeatsPerImage;
    int mnMaxFeatsForUpdate;

    int mnMaxTrackingLength;
    int mnMinTrackingLength;

    bool mbIsRGB;
    bool mbIsFisheye;
    bool mbIsTheFirstImage;

    bool mbEnableEqualizer;

    // Intrinsics
    cv::Mat mK;
    cv::Mat mDistCoef;

    /**
     * Feature tracking history
     *
     * @each list number is a reusable feature index.
     * @each list corresponds to a feature with its size the tracking length.
     * @only store the undistorted and normalized coordinates, (u'/f,v'/f).
     */
    std::vector<std::list<cv::Point2f> > mvlTrackingHistory;

    // Indices of inliers
    std::vector<int> mvInlierIndices;

    // Indices available
    std::list<int> mlFreeIndices;

    // Features to track
    std::vector<cv::Point2f> mvFeatsToTrack;
    int mnFeatsToTrack;

    // For RANSAC
    Eigen::MatrixXd mPoints1ForRansac;
    Eigen::MatrixXd mPoints2ForRansac;

    // Last image
    cv::Mat mLastImage;

    // Handlers
    CornerDetector* mpCornerDetector;
    CornerCluster* mpCornerCluster;
    Ransac* mpRansac;

    // Interact with rviz
    ros::NodeHandle mTrackerNode;
    ros::Publisher mTrackPub;
    ros::Publisher mNewerPub;
};

} // namespace RVIO

#endif
