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

#include "rvio/SensorDatabase.h"


namespace RVIO
{

SensorDatabase::SensorDatabase() {}


void SensorDatabase::PushImuData(ImuData* pInData)
{
    std::unique_lock<std::mutex> lock(mMutexImu);

    mlImuFIFO.push_back(pInData);

    if (mlImuFIFO.size()>1)
        mlImuFIFO.sort([](const ImuData* a, const ImuData* b) {return a->Timestamp<b->Timestamp;});
}


bool SensorDatabase::PopImuData(ImuData* pOutData)
{
    std::unique_lock<std::mutex> lock(mMutexImu);

    if (!mlImuFIFO.empty())
    {
        pOutData = mlImuFIFO.front();
        mlImuFIFO.pop_front();
        return true;
    }
    else
        return false;
}


int SensorDatabase::GetImuDataByTimestamp(const double nTimestamp,
                                          std::list<ImuData*>& lOutData)
{
    std::unique_lock<std::mutex> lock(mMutexImu);

    if (mlImuFIFO.empty())
        return 0;

    lOutData.clear();

    int nData = 0;
    while (!mlImuFIFO.empty())
    {
        if (mlImuFIFO.front()->Timestamp<nTimestamp)
        {
            lOutData.push_back(mlImuFIFO.front());

            mlImuFIFO.pop_front();
            nData++;
        }
        else
            break;
    }

    return nData;
}

} // namespace RVIO
