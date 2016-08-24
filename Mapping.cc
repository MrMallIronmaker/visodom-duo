//
// Created by mark on 8/24/16.
//

#include "Mapping.h"

omar::Mapping::Mapping(double thresholdVariance_, double positionalVariance_, double intensityVariance_) {

}
cv::Mat omar::Mapping::estimateUncertainty(Pose estimate, cv::Mat newImage) {
  return cv::Mat();
}
PointCloud omar::Mapping::map(PointCloud oldWorld, Pose estimate, cv::Mat newImage) {
  return omar::PointCloud();
}





