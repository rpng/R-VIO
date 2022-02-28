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
