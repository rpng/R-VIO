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

#include <opencv2/imgproc/imgproc.hpp>

#include "rvio/CornerDetector.h"


namespace RVIO
{

CornerDetector::CornerDetector(const cv::FileStorage& fsSettings)
{
    mnQualityLevel = fsSettings["Tracker.nQualLvl"];
    mnMinDistance = fsSettings["Tracker.nMinDist"];
}


int CornerDetector::DetectWithSubPix(const int nMaxCorners,
                                     const cv::Mat& im,
                                     std::vector<cv::Point2f>& vCorners)
{
    vCorners.clear();

    cv::goodFeaturesToTrack(im, vCorners, nMaxCorners, mnQualityLevel, mnMinDistance);

    if (vCorners.size()!=0)
    {
        // Refine the locations
        cv::Size subPixWinSize(10,10);
        cv::Size subPixZeroZone(-1,-1);
        cv::TermCriteria subPixCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 20, 0.01);
        cv::cornerSubPix(im, vCorners, subPixWinSize, subPixZeroZone, subPixCriteria);
    }

    return vCorners.size();
}

} // namespace RVIO
