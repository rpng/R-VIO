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

#ifndef SENSORDATABASE_H
#define SENSORDATABASE_H

#include <list>
#include <mutex>

#include <Eigen/Core>


namespace RVIO
{

struct ImuData
{
    Eigen::Vector3d AngularVel;
    Eigen::Vector3d LinearAccel;
    double Timestamp;
    double TimeInterval;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuData() :
    AngularVel(Eigen::Vector3d::Zero()),
    LinearAccel(Eigen::Vector3d::Zero()),
    Timestamp(0),
    TimeInterval(0) {}
};


class SensorDatabase
{
public:

    SensorDatabase();

    void PushImuData(ImuData* pInData);
    bool PopImuData(ImuData* pOutData);

    int GetImuDataByTimestamp(const double nTimestamp, std::list<ImuData*>& lOutData);

protected:

    std::list<ImuData*> mlImuFIFO;

    std::mutex mMutexImu;
};

} // namespace RVIO

#endif
