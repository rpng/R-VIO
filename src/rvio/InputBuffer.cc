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

#include "InputBuffer.h"
#include "../util/Numerics.h"


namespace RVIO
{

InputBuffer::InputBuffer() {}


void InputBuffer::PushImuData(ImuData* pData)
{
    std::unique_lock<std::mutex> lock(mMutex);

    mlImuFIFO.push_back(pData);

    if (!mlImuFIFO.empty())
        mlImuFIFO.sort([](const ImuData* a, const ImuData* b){return a->Timestamp<b->Timestamp;});
}


void InputBuffer::PushImageData(ImageData* pData)
{
    std::unique_lock<std::mutex> lock(mMutex);

    mlImageFIFO.push_back(pData);

    if (!mlImageFIFO.empty())
        mlImageFIFO.sort([](const ImageData* a, const ImageData* b){return a->Timestamp<b->Timestamp;});
}


bool InputBuffer::GetMeasurements(double nTimeOffset, std::pair<ImageData*, std::list<ImuData*> >& pMeasurements)
{
    if (mlImuFIFO.empty() || mlImageFIFO.empty())
        return false;

    // Make sure we have enough IMU measurements for an image to process!
    if (mlImuFIFO.back()->Timestamp<mlImageFIFO.front()->Timestamp+nTimeOffset)
        return false;

    std::unique_lock<std::mutex> lock(mMutex);

    ImageData* image = mlImageFIFO.front();
    mlImageFIFO.pop_front();

    std::list<ImuData*> imus;

    while (!mlImuFIFO.empty() && mlImuFIFO.front()->Timestamp<=image->Timestamp+nTimeOffset)
    {
        imus.push_back(mlImuFIFO.front());
        mlImuFIFO.pop_front();
    }

    if (imus.size()<2)
        return false;

    pMeasurements = {image, imus};

    return true;
}

} // namespace RVIO
