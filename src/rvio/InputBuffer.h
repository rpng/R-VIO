#ifndef INPUTBUFFER_H
#define INPUTBUFFER_H

#include <list>
#include <mutex>

#include <Eigen/Core>

#include <opencv2/core/core.hpp>


namespace RVIO
{

struct ImuData
{
    Eigen::Vector3d AngularVel;
    Eigen::Vector3d LinearAccel;
    double Timestamp;
    double TimeInterval;

    ImuData()
    {
        AngularVel.setZero();
        LinearAccel.setZero();
        Timestamp = 0;
        TimeInterval = 0;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImageData
{
    cv::Mat Image;
    double Timestamp;

    ImageData()
    {
        Image = cv::Mat();
        Timestamp = 0;
    }
};


class InputBuffer
{
public:

    InputBuffer();

    void PushImuData(ImuData* pData);
    void PushImageData(ImageData* pData);

    bool GetMeasurements(double nTimeOffset, std::pair<ImageData*, std::list<ImuData*> >& pMeasurements);

protected:

    std::list<ImuData*> mlImuFIFO;
    std::list<ImageData*> mlImageFIFO;

    std::mutex mMutex;
};

} // namespace RVIO

#endif
