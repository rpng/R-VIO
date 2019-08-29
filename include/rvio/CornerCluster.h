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

#ifndef CORNER_CLUSTER_H
#define CORNER_CLUSTER_H

#include <list>
#include <vector>

#include <opencv2/core/core.hpp>


namespace RVIO
{

class CornerCluster
{
public:

    CornerCluster(const cv::FileStorage& fsSettings);

    void ChessGrid(std::vector<cv::Point2f>& vCorners);

    int FindNew(std::vector<cv::Point2f>& vRefCorners, std::vector<cv::Point2f>& vNewCorners, const int nMinDistance=10);

private:

    int mnGridCols;
    int mnGridRows;

    int mnBlocks;

    double mnBlockSize;

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
