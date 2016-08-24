#ifndef TRACKING_H
#define TRACKING_H

#include "Pose.h"
#include "PointCloud.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <limits>

namespace omar {

class Tracking {

private:
    PointCloud pointCloud;

    Pose iterateTracking(cv::Mat jacobian, Pose poseHypothesis, cv::Mat intensityImage, double fLength);
    cv::Mat computeResiduals(Pose poseHypothesis, cv::Mat trackableImage, double fLength);
    cv::Mat computeWeights(cv::Mat residuals);
    cv::Mat computeJacobian(double fLength, int height, int width);

public:
    Tracking();
    Tracking(omar::PointCloud _pointCloud);

    // todo: dox
    Pose testIterationOfTracking(cv::Mat intensityImage, double fLength);
    Pose track(cv::Mat intensityImage, double fLength);

};

}

#endif