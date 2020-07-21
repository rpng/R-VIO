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

#ifndef CORNER_DETECTOR_H
#define CORNER_DETECTOR_H

#include <vector>

#include <opencv2/core/core.hpp>


namespace RVIO
{

class CornerDetector
{
public:

    CornerDetector(const cv::FileStorage& fsSettings);

    /**
     * Corner detector
     *
     * @detect corners on the input image @p im and return the number of corners.
     * @realized using the OpenCV functions:
     *  cv::goodFeaturesToTrack(): determines strong corners in the image, and
     *  cv::cornerSubPix(): refines the corners' locations.
     * @extract the maximum number of corners @p nMaxCorners,
     * @store the corners into @p vCorners for the visual tracking.
     */
    int DetectWithSubPix(const int nMaxCorners, const cv::Mat& im, std::vector<cv::Point2f>& vCorners);

private:

    int mnImageRows;
    int mnImageCols;

    double mnQualityLevel;

    int mnMinDistance;
};

} // namespace RVIO

#endif
