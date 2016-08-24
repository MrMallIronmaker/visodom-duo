//
// Created by mark on 8/24/16.
//

#ifndef VISODOM_MAPPING_H
#define VISODOM_MAPPING_H

#include "Pose.h"
#include "PointCloud.h"
namespace omar {

class Mapping {
 private:
  double thresholdVariance; // sigma^2_{d, obs} in the paper
  double positionalVariance; // sigma^2_{lambda(xi, pi)} in the paper
  double intensityVariance; // sigma^2_{lambda(I)} in the paper

  /**
   * Initialize a new mapper with the given parameters.
   */
  Mapping(double thresholdVariance_, double positionalVariance_,
          double intensityVariance_);

  /**
   * Returns a boolean matrix that represents whether that particular pixel
   * has a small enough uncertainty for it to be worth calculating depth
   * See Sec 2.1.3
   */
  cv::Mat estimateUncertainty(Pose estimate, cv::Mat newImage);

  /**
   * Returns the updated point cloud after processing the new image
   * See Sec 2, all of it
   * TODO: do I copy or overwrite? I assume copy is good right now.
   */
  PointCloud map(PointCloud oldWorld, Pose estimate, cv::Mat newImage);

  /**
   * Given the location of a pixel, find the most useful reference frame.
   * See Sec 2.1.1
   */
  cv::Mat getReferenceFrame(int row, int col);

};

} // end namespace


#endif //VISODOM_MAPPING_H
