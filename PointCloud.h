//
// Created by mark on 4/15/16.
//

#ifndef OMAR_POINTCLOUD_H_
#define OMAR_POINTCLOUD_H_

#include <opencv2/core/mat.hpp>
#include "Pose.h"

namespace omar {

class PointCloud {
    /**
     * todo: long discussion on axis conventions
     *
     * tldr; +Z is out, +X is left, +Y is up.
     */
private:
    cv::Mat pointsAndIntensities;

    void initialize(cv::Mat image, cv::Mat depth, double fLengthInPixels);

public:
    /**
     * TODO: dox
     */
    PointCloud();

    PointCloud(const PointCloud& other);

    PointCloud(cv::Mat image, double fLengthInPixels);

    PointCloud(cv::Mat image, cv::Mat depth, double fLengthInPixels);

    cv::Mat renderToImage(double fLengthInPixels, int height, int width);

    cv::Mat renderToIndexes(double fLengthInPixels, int height, int width);

    // a two channel mat, image and depth
    cv::Mat renderToImageAndDepth(double fLengthInPixels, int height, int width);

    // todo: add zbuff
    cv::Mat renderToPositions(double fLengthInPixels, int height, int width);

    PointCloud viewPoints(omar::Pose otherCameraPose);

    void transform(omar::Pose otherCameraPose);

    void untransform(omar::Pose otherCameraPose);

    cv::Mat getPoint(int pointIndex);

    void addPoints(PointCloud);
};

}
#endif //OMAR_POINTCLOUD_H_
