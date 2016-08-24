//
// Created by mark on 4/15/16.
//

#ifndef OMAR_MATUTILS_H
#define OMAR_MATUTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace omar {

/**
 * TODO: dox
 */
void assertMatEquals(cv::Mat mat1, cv::Mat mat2);

void assertMatEquals(cv::Mat mat1, cv::Mat mat2, double eps);

void assertMatEquals(std::string tag, cv::Mat mat1, cv::Mat mat2);

void assertMatEquals(std::string tag, cv::Mat mat1, cv::Mat mat2, double eps);

cv::Mat imreadAsGrayDouble(std::string name);

std::string displayForDebugging(cv::Mat mat);

cv::Mat imreadAsMsrcKinectDepth(std::string name);

}

#endif //VISODOM_MATUTILS_H
