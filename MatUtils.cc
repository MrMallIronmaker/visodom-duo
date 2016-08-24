#include "MatUtils.h"
#include "gtest/gtest.h"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/highgui.hpp>


namespace omar {

namespace matutils {

const double defaultEpsilon = 2 * DBL_EPSILON;
const std::string defaultTag("No tag");
const int maxSizeToPrint = 32;

} // end namespace matutils

void assertMatEquals(cv::Mat mat1, cv::Mat mat2) {
    assertMatEquals(matutils::defaultTag, mat1, mat2, matutils::defaultEpsilon);
}

void assertMatEquals(cv::Mat mat1, cv::Mat mat2, double eps) {
    assertMatEquals(matutils::defaultTag, mat1, mat2, eps);
}

void assertMatEquals(std::string tag, cv::Mat mat1, cv::Mat mat2) {
    assertMatEquals(tag, mat1, mat2, matutils::defaultEpsilon);
}

void assertMatEquals(std::string tag, cv::Mat mat1, cv::Mat mat2, double eps) {
    /* OpenCV has the convention that (-1, -1) is the size of a multidimensional image
     * todo: I'm not supporting functionality for that yet.*/
    double what = mat2.at<double>(0, 3);
    ASSERT_NE(-1, mat1.size().height);

    ASSERT_EQ(mat2.rows, mat1.rows);
    ASSERT_EQ(mat2.cols, mat1.cols);
    cv::Mat diff = abs(mat1 - mat2) > 2 * eps;
    what = mat2.at<double>(0, 3);

    if (mat2.rows < matutils::maxSizeToPrint && mat2.cols < matutils::maxSizeToPrint) {
        EXPECT_EQ(0, cv::countNonZero(diff))
                                    << "Tag: " << tag << std::endl
                                    << "mat1: " << std::endl << mat1 << std::endl
                                    << "mat2: " << std::endl << mat2 << std::endl;
    } else {
        int whatwhat = cv::countNonZero(diff);

        EXPECT_EQ(0, cv::countNonZero(diff))
                                    << "Tag: " << tag << std::endl
                                    << "mat1: " << displayForDebugging(mat1) << std::endl
                                    << "mat2: " << displayForDebugging(mat2) << std::endl;
    }
}

std::string displayForDebugging(cv::Mat mat) {
    cv::imshow("Display Image", mat);
    cv::waitKey();
    return "Displayed.";
}

cv::Mat imreadAsGrayDouble(std::string name) {
    cv::Mat image = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
    image.convertTo(image, CV_64FC1, 1.0/255);
    return image;
}

cv::Mat imreadAsMsrcKinectDepth(std::string name) {
    cv::Mat image = cv::imread(name, CV_LOAD_IMAGE_ANYDEPTH);
    image.convertTo(image, CV_64FC1, 1.0/900);
    // factor from millimeters to meters.
    return image;
}

}