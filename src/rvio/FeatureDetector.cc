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

#include <opencv2/opencv.hpp>

#include "FeatureDetector.h"


namespace RVIO
{

FeatureDetector::FeatureDetector(const cv::FileStorage& fsSettings)
{
    mnMinDistance = fsSettings["Tracker.nMinDist"];
    mnQualityLevel = fsSettings["Tracker.nQualLvl"];

    mnBlockSizeX = fsSettings["Tracker.nBlockSizeX"];
    mnBlockSizeY = fsSettings["Tracker.nBlockSizeY"];

    mnImageCols = fsSettings["Camera.width"];
    mnImageRows = fsSettings["Camera.height"];

    mnGridCols = std::floor(mnImageCols/mnBlockSizeX);
    mnGridRows = std::floor(mnImageRows/mnBlockSizeY);

    mnBlocks = mnGridCols*mnGridRows;

    mnOffsetX = .5*(mnImageCols-mnGridCols*mnBlockSizeX);
    mnOffsetY = .5*(mnImageRows-mnGridRows*mnBlockSizeY);

    int nMaxFeatsPerImage = fsSettings["Tracker.nFeatures"];
    mnMaxFeatsPerBlock = (float)nMaxFeatsPerImage/mnBlocks;

    mvvGrid.resize(mnBlocks);
}


int FeatureDetector::DetectWithSubPix(const cv::Mat& im, 
                                      const int nCorners, 
                                      const int s, 
                                      std::vector<cv::Point2f>& vCorners)
{
    vCorners.clear();
    vCorners.reserve(nCorners);

    cv::goodFeaturesToTrack(im, vCorners, nCorners, mnQualityLevel, s*mnMinDistance);

    if (!vCorners.empty())
    {
        // Refine the location
        cv::Size subPixWinSize(std::floor(.5*mnMinDistance), std::floor(.5*mnMinDistance));
        cv::Size subPixZeroZone(-1,-1);
        cv::TermCriteria subPixCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-2);
        cv::cornerSubPix(im, vCorners, subPixWinSize, subPixZeroZone, subPixCriteria);
    }

    return (int)vCorners.size();
}


void FeatureDetector::ChessGrid(const std::vector<cv::Point2f>& vCorners)
{
    mvvGrid.clear();
    mvvGrid.resize(mnBlocks);

    for (const cv::Point2f &pt : vCorners)
    {
        if (pt.x<=mnOffsetX || pt.y<=mnOffsetY || pt.x>=(mnImageCols-mnOffsetX) || pt.y>=(mnImageRows-mnOffsetY))
            continue;

        int col = std::floor((pt.x-mnOffsetX)/mnBlockSizeX);
        int row = std::floor((pt.y-mnOffsetY)/mnBlockSizeY);

        int nBlockIdx = row*mnGridCols+col;
        mvvGrid.at(nBlockIdx).emplace_back(pt);
    }
}


int FeatureDetector::FindNewer(const std::vector<cv::Point2f>& vCorners, 
                               const std::vector<cv::Point2f>& vRefCorners, 
                               std::deque<cv::Point2f>& qNewCorners)
{
    ChessGrid(vRefCorners);

    for (const cv::Point2f &pt : vCorners)
    {
        if (pt.x<=mnOffsetX || pt.y<=mnOffsetY || pt.x>=(mnImageCols-mnOffsetX) || pt.y>=(mnImageRows-mnOffsetY))
            continue;

        int col = std::floor((pt.x-mnOffsetX)/mnBlockSizeX);
        int row = std::floor((pt.y-mnOffsetY)/mnBlockSizeY);

        float xl = col*mnBlockSizeX+mnOffsetX;
        float xr = xl+mnBlockSizeX;
        float yt = row*mnBlockSizeY+mnOffsetY;
        float yb = yt+mnBlockSizeY;

        if (fabs(pt.x-xl)<mnMinDistance || fabs(pt.x-xr)<mnMinDistance || fabs(pt.y-yt)<mnMinDistance || fabs(pt.y-yb)<mnMinDistance)
            continue;

        int nBlockIdx = row*mnGridCols+col;

        if ((float)mvvGrid.at(nBlockIdx).size()<.75*mnMaxFeatsPerBlock)
        {
            if (!mvvGrid.at(nBlockIdx).empty())
            {
                int cnt = 0;

                for (const cv::Point2f &bpt : mvvGrid.at(nBlockIdx))
                {
                    if (cv::norm(pt-bpt)>1*mnMinDistance)
                        cnt++;
                    else
                        break;
                }

                if (cnt==(int)mvvGrid.at(nBlockIdx).size())
                {
                    qNewCorners.push_back(pt);
                    mvvGrid.at(nBlockIdx).push_back(pt);
                }
            }
            else
            {
                qNewCorners.push_back(pt);
                mvvGrid.at(nBlockIdx).push_back(pt);
            }
        }
    }

    return (int)qNewCorners.size();
}

} // namespace RVIO
