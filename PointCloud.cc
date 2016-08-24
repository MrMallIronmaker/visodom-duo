//
// Created by mark on 4/15/16.
//

#include <iostream>
#include <limits>
#include "Pose.h"
#include "PointCloud.h"

namespace omar {

PointCloud::PointCloud() {
    pointsAndIntensities = cv::Mat(4, 0, CV_64FC1);
}

PointCloud::PointCloud(const PointCloud& other) {
    pointsAndIntensities = other.pointsAndIntensities.clone();
}

PointCloud::PointCloud(cv::Mat image, double fLengthInPixels) {
    cv::Mat depth(image.size(), CV_64FC1, cv::Scalar(1.0));
    initialize(image, depth, fLengthInPixels);
}

PointCloud::PointCloud(cv::Mat image, cv::Mat depth, double fLengthInPixels) {
    initialize(image, depth, fLengthInPixels);
}

void PointCloud::initialize(cv::Mat image, cv::Mat depth, double fLengthInPixels) {
    // right now, only accept intensity mats
    assert(image.channels() == 1);

    // calculate number of items
    int items = image.size().area();
    int height = image.rows;
    int width = image.cols;

    pointsAndIntensities = cv::Mat(4, items, CV_64FC1);
    int currentItemIndex = 0;
    for (int pixelY = 0; pixelY < height; pixelY++) {
        for (int pixelX = 0; pixelX < width; pixelX++) {
            // todo: formula here
            double zCoord = depth.at<double>(pixelY, pixelX);
            double xCoord = ((width - 1.0) / 2 - pixelX) / fLengthInPixels * zCoord;
            double yCoord = ((height - 1.0) / 2 - pixelY) / fLengthInPixels * zCoord;

            pointsAndIntensities.at<double>(0, currentItemIndex) = xCoord;
            pointsAndIntensities.at<double>(1, currentItemIndex) = yCoord;
            pointsAndIntensities.at<double>(2, currentItemIndex) = zCoord;
            pointsAndIntensities.at<double>(3, currentItemIndex) = image.at<double>(pixelY, pixelX);
            currentItemIndex++;
        }
    }
}

// todo: zbuf
cv::Mat PointCloud::renderToIndexes(double fLengthInPixels, int height, int width) {
    cv::Mat matOfIndexes(height, width, CV_32SC1, cv::Scalar(-1));
    cv::Mat positions = renderToPositions(fLengthInPixels, height, width);

    // for each point
    for (int i = 0; i < positions.cols; i++) {
        int pixelY = positions.at<int>(0, i); // the row
        int pixelX = positions.at<int>(1, i);
        // if it's within the pixel range:
        if (pixelX != -1) {
            // dump & increment
            matOfIndexes.at<int>(pixelY, pixelX) = i;
        }
    }
    return matOfIndexes;
}

cv::Mat PointCloud::renderToImage(double fLengthInPixels, int height, int width) {
    // initialize matrix
    cv::Mat accumulators(height, width, CV_64FC1, 0.0);
    cv::Mat counts(height, width, CV_8UC1, cv::Scalar(0));
    cv::Mat positions = renderToPositions(fLengthInPixels, height, width);

    // for each point
    for (int i = 0; i < positions.cols; i++) {
        int pixelY = positions.at<int>(0, i); // the row
        int pixelX = positions.at<int>(1, i);
        // if it's within the pixel range:
        if (pixelX != -1) {
            // dump & increment
            double val = pointsAndIntensities.at<double>(3, i);
            accumulators.at<double>(pixelY, pixelX) += val;
            counts.at<uchar>(pixelY, pixelX)++;
        }
    }

    cv::Mat intensities(height, width, CV_64FC1);
    cv::Mat countsAsDoubles;
    counts.convertTo(countsAsDoubles, CV_64FC1);
    cv::divide(accumulators, countsAsDoubles, intensities);
    return intensities;
}

cv::Mat PointCloud::renderToPositions(double fLengthInPixels, int height, int width) {
    cv::Mat positions(2, pointsAndIntensities.cols, CV_32SC1);
    for (int i = 0; i < pointsAndIntensities.cols; i++) {
        double x = pointsAndIntensities.at<double>(0, i);
        double y = pointsAndIntensities.at<double>(1, i);
        double z = pointsAndIntensities.at<double>(2, i);
        // calculate x
        int pixelX = int(double(width) / 2 - x/z * fLengthInPixels);
        // calculate y
        int pixelY = int(double(height) / 2 - y/z * fLengthInPixels);

            // if it's within the pixel range:
        if (z > 0 && 0 <= pixelX && pixelX < width && 0 <= pixelY && pixelY < height) {
            positions.at<int>(0, i) = pixelY; // the row
            positions.at<int>(1, i) = pixelX; // the col
        } else {
            positions.at<int>(0, i) = -1; // the row
            positions.at<int>(1, i) = -1; // the col
        }
    }
    return positions;
}

cv::Mat PointCloud::renderToImageAndDepth(double fLengthInPixels, int height, int width) {
    // todo: I COPY PASTED CODE. I FEE LGUILDTy
    // initialize matrix
    cv::Scalar defaultEntry(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity());
    cv::Mat intensities(height, width, CV_64FC2, defaultEntry);
    cv::Mat positions = renderToPositions(fLengthInPixels, height, width);

    // for each point
    for (int i = 0; i < positions.cols; i++) {
        int pixelY = positions.at<int>(0, i); // the row
        int pixelX = positions.at<int>(1, i);
        // if it's within the pixel range:
        if (pixelX != -1) {
            if (intensities.at<cv::Vec2d>(pixelY, pixelX)[1] > pointsAndIntensities.at<double>(2, i)) {
                intensities.at<cv::Vec2d>(pixelY, pixelX)[1] = pointsAndIntensities.at<double>(2, i);
                intensities.at<cv::Vec2d>(pixelY, pixelX)[0] = pointsAndIntensities.at<double>(3, i);
            }
        }
    }
    return intensities;

}

PointCloud PointCloud::viewPoints(omar::Pose otherCameraPose) {
    PointCloud otherPointCloud = PointCloud(*this);
    otherPointCloud.transform(otherCameraPose);
    return otherPointCloud;
}

void PointCloud::transform(omar::Pose otherCameraPose) {
    cv::Mat transformedPoints = otherCameraPose.world2camera(pointsAndIntensities(cv::Range(0, 3), cv::Range::all()));
    cv::Mat replaceablePoints(pointsAndIntensities, cv::Range(0, 3));
    transformedPoints.copyTo(replaceablePoints);
}

void PointCloud::untransform(omar::Pose otherCameraPose) {
    cv::Mat transformedPoints = otherCameraPose.camera2world(pointsAndIntensities(cv::Range(0, 3), cv::Range::all()));
    cv::Mat replaceablePoints(pointsAndIntensities, cv::Range(0, 3));
    transformedPoints.copyTo(replaceablePoints);
}

cv::Mat PointCloud::getPoint(int pointIndex) {
    return pointsAndIntensities.col(pointIndex);
}

void PointCloud::addPoints(PointCloud other) {
    hconcat(pointsAndIntensities, other.pointsAndIntensities, pointsAndIntensities);
}

}

