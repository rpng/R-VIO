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

#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <vector>
#include <deque>

#include <opencv2/core/core.hpp>


namespace RVIO
{

class FeatureDetector
{
public:

    FeatureDetector(const cv::FileStorage& fsSettings);

    /**
     * Corner detector
     *
     * @detect corners on the input image @p im and return the number of corners.
     * @realized using the OpenCV functions:
     *  cv::goodFeaturesToTrack(): determines strong corners in the image, and
     *  cv::cornerSubPix(): refines the corners' locations.
     * @extract the number of corners needed @p nCorners,
     * @store the corners into @p vCorners for the visual tracking.
     */
    int DetectWithSubPix(const cv::Mat& im, const int nCorners, const int s, std::vector<cv::Point2f>& vCorners);

    int FindNewer(const std::vector<cv::Point2f>& vCorners, const std::vector<cv::Point2f>& vRefCorners, std::deque<cv::Point2f>& qNewCorners);

private:

    void ChessGrid(const std::vector<cv::Point2f>& vCorners);

private:

    int mnImageCols;
    int mnImageRows;

    float mnMinDistance;
    float mnQualityLevel;

    int mnGridCols;
    int mnGridRows;

    // Top-left corner of chess grid
    int mnOffsetX;
    int mnOffsetY;

    float mnBlockSizeX;
    float mnBlockSizeY;

    int mnBlocks;

    int mnMaxFeatsPerBlock;

    /**
     * Corner distribution
     *
     * @each vector contains all the corners within the block.
     * @used for finding new features for the visual tracking.
     */
    std::vector<std::vector<cv::Point2f> > mvvGrid;
};

} // namespace RVIO

#endif
