#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <vector>
#include <deque>

#include <opencv2/core/core.hpp>


namespace RVIO
{

class FeatureDetector
{
public:

    FeatureDetector(const cv::FileStorage& fsSettings);

    /**
     * Corner detector
     *
     * @detect corners on the input image @p im and return the number of corners.
     * @realized using the OpenCV functions:
     *  cv::goodFeaturesToTrack(): determines strong corners in the image, and
     *  cv::cornerSubPix(): refines the corners' locations.
     * @extract the number of corners needed @p nCorners,
     * @store the corners into @p vCorners for the visual tracking.
     */
    int DetectWithSubPix(const cv::Mat& im, const int nCorners, const int s, std::vector<cv::Point2f>& vCorners);

    int FindNewer(const std::vector<cv::Point2f>& vCorners, const std::vector<cv::Point2f>& vRefCorners, std::deque<cv::Point2f>& qNewCorners);

private:

    void ChessGrid(const std::vector<cv::Point2f>& vCorners);

private:

    int mnImageCols;
    int mnImageRows;

    float mnMinDistance;
    float mnQualityLevel;

    int mnGridCols;
    int mnGridRows;

    // Top-left corner of chess grid
    int mnOffsetX;
    int mnOffsetY;

    float mnBlockSizeX;
    float mnBlockSizeY;

    int mnBlocks;

    int mnMaxFeatsPerBlock;

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
