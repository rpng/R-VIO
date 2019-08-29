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

#include "rvio/CornerCluster.h"


namespace RVIO
{

CornerCluster::CornerCluster(const cv::FileStorage& fsSettings)
{
    const int nImageRows = fsSettings["Camera.height"];
    const int nImageCols = fsSettings["Camera.width"];

    mnBlockSize = fsSettings["Tracker.nGridSize"];

    mnGridRows = floor(nImageRows/mnBlockSize)+1;
    mnGridCols = floor(nImageCols/mnBlockSize)+1;

    mnBlocks = mnGridCols*mnGridRows;

    mvvGrid.resize(mnBlocks);
}


void CornerCluster::ChessGrid(std::vector<cv::Point2f>& vCorners)
{
    mvvGrid.clear();
    mvvGrid.resize(mnBlocks);

    for (std::vector<cv::Point2f>::const_iterator vit=vCorners.begin();
         vit!=vCorners.end(); ++vit)
    {
        int col = floor((*vit).x/mnBlockSize);
        int row = floor((*vit).y/mnBlockSize);

        if ((col>-1 && col<mnGridCols) && (row>-1 && row<mnGridRows))
        {
            int nBlockIdx = row*mnGridCols+col;
            mvvGrid.at(nBlockIdx).push_back(*vit);
        }
    }
}


int CornerCluster::FindNew(std::vector<cv::Point2f>& vRefCorners,
                           std::vector<cv::Point2f>& vNewCorners,
                           const int nMinDistance)
{
    for (std::vector<cv::Point2f>::const_iterator vit1=vRefCorners.begin();
         vit1!=vRefCorners.end(); ++vit1)
    {
        int col = floor((*vit1).x/mnBlockSize);
        int row = floor((*vit1).y/mnBlockSize);

        if ((col>-1 && col<mnGridCols) && (row>-1 && row<mnGridRows))
        {
            int nBlockIdx = row*mnGridCols+col;

            if (!mvvGrid.at(nBlockIdx).empty())
            {
                int count = 0;

                for (std::vector<cv::Point2f>::const_iterator vit2=mvvGrid.at(nBlockIdx).begin();
                     vit2!=mvvGrid.at(nBlockIdx).end(); ++vit2)
                {
                    // Check the distance in-between
                    cv::Point2f e = (*vit1)-(*vit2);
                    if (cv::norm(e)>nMinDistance)
                        count++;
                    else
                        break;
                }

                if (count==(int)mvvGrid.at(nBlockIdx).size())
                    vNewCorners.push_back(*vit1);
            }
            else
                vNewCorners.push_back(*vit1);
        }
    }

    return vNewCorners.size();
}

} // namespace RVIO
