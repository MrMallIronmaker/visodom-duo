#include "Tracking.h"
#include <opencv2/highgui/highgui.hpp>
#include <limits>
#include <iostream>
#include <string>

namespace omar {

Tracking::Tracking() {
  pointCloud = omar::PointCloud();
}

Tracking::Tracking(omar::PointCloud _pointCloud) {
  pointCloud = _pointCloud;
}

cv::Mat Tracking::computeJacobian(double fLength, int height, int width) {

  double dNaN = std::numeric_limits<double>::quiet_NaN();

  cv::Mat imageAndDepth = pointCloud.renderToImageAndDepth(fLength, height, width);
  cv::Mat jacobian = cv::Mat(imageAndDepth.rows * imageAndDepth.cols, 6, CV_64FC1);
  cv::Mat xGradient(height, width, CV_64FC1);

  int jacobianEntry = 0;
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int x0 = MAX(0, MIN(col - 1, width - 1));
      int x1 = MAX(0, MIN(col + 1, width - 1));
      int y0 = MAX(0, MIN(row - 1, height - 1));
      int y1 = MAX(0, MIN(row + 1, height - 1));

      if (!std::isnan(imageAndDepth.at<cv::Vec2d>(row, x1)[0] +
          imageAndDepth.at<cv::Vec2d>(row, x0)[0] +
          imageAndDepth.at<cv::Vec2d>(y1, col)[0] +
          imageAndDepth.at<cv::Vec2d>(y0, col)[0] +
          imageAndDepth.at<cv::Vec2d>(row, col)[0])) {

        // calculate gradient
        double x_grad = 0.5 * fLength * (imageAndDepth.at<cv::Vec2d>(row, x1)[0]
            - imageAndDepth.at<cv::Vec2d>(row, x0)[0]);
        double y_grad = 0.5 * fLength * (imageAndDepth.at<cv::Vec2d>(y1, col)[0]
            - imageAndDepth.at<cv::Vec2d>(y0, col)[0]);
        cv::Matx12d pixel_gradient(x_grad, y_grad);
        xGradient.at<double>(row, col) = x_grad;

        // calculate position related derivative.
        double u_f = (col - (width / 2.0)) / fLength;
        double v_f = (row - (height / 2.0)) / fLength;
        double d = 1.0 / imageAndDepth.at<cv::Vec2d>(row, col)[1];
        double u_f2 = u_f * u_f;
        double v_f2 = v_f * v_f;
        double ufvf = u_f * v_f;
        cv::Matx<double, 2, 6> pixel_jacobian(
            -d, 0, -u_f * d, ufvf, -1 - u_f2, -v_f,
            0, -d, -v_f * d, 1 + v_f2, -ufvf, u_f);

        cv::Matx16d pixel_pose_shift = pixel_gradient * pixel_jacobian;

        for (int j = 0; j < 6; j++) {
          jacobian.at<double>(jacobianEntry, j) = pixel_pose_shift(j);
        }
      } else {
        for (int j = 0; j < 6; j++) {
          jacobian.at<double>(jacobianEntry, j) = 0;
        }
      }
      jacobianEntry++;
    }
  }

  //cv::imshow("xGrad", xGradient / fLength + 0.5);

  return jacobian;
}

/**
 * Given a pose, compute the error
 */
cv::Mat Tracking::computeResiduals(Pose poseHypothesis, cv::Mat trackableImage, double fLength) {
  int height = trackableImage.rows;
  int width = trackableImage.cols;
  double dNaN = std::numeric_limits<double>::quiet_NaN();
  cv::Mat expectedSource = pointCloud.renderToImage(fLength, height, width);

  // 1) Transform old points into new coordinates

  PointCloud transformedPointCloud(pointCloud);
  transformedPointCloud.untransform(poseHypothesis);

  // 2) Reach into the new image and grab the value at the new estimated positions.
  // A) for each point, find the pixel it's at [map pixels to points, this time the index]
  cv::Mat matOfIndexes = pointCloud.renderToIndexes(fLength, height, width);
  cv::Mat newImage(trackableImage.size(), trackableImage.type());
  // B) For each point, find its transformed version. (done in part 1)
  // C) For each transformed point, find its x/y in the new image.
  cv::Mat matOfPositions = transformedPointCloud.renderToPositions(fLength, height, width);
  // D) Steal its value
  for (int y = 0; y < matOfIndexes.rows; y++) {
    for (int x = 0; x < matOfIndexes.cols; x++) {
      int index = matOfIndexes.at<int>(y, x);
      if (index != -1) {
        int pixelY = matOfPositions.at<int>(0, index); // the row
        int pixelX = matOfPositions.at<int>(1, index);
        newImage.at<double>(y, x) = trackableImage.at<double>(pixelY, pixelX);
      } else {
        newImage.at<double>(y, x) = dNaN;
      }
    }
  }

  // 3) Compare new values at estimated positions with old image.
  cv::Mat diff = newImage - expectedSource;
  return diff;
}

/**
 * todo: dox
 */
cv::Mat Tracking::computeWeights(cv::Mat residuals) {
  int degreesOfFreedom = 5;
  cv::Mat weights;
  int oldRowCount = residuals.rows;
  residuals = residuals.reshape(0, 1);

  // A) Calculate initial variance
  cv::Mat squaredResiduals = residuals.mul(residuals);
  double dNaN = std::numeric_limits<double>::quiet_NaN();
  cv::Mat nanMask = (residuals != dNaN);
  double newVariance = cv::mean(squaredResiduals, nanMask)[0];

  if (newVariance == 0.0) { // that is, everything is the same already.
    weights = cv::Mat(squaredResiduals.size(), squaredResiduals.type(), dNaN);
    weights.setTo(1.0, nanMask);
    return weights;
  }
  double oldVariance;
  do {
    oldVariance = newVariance;
    // B) Calculate weights vector
    weights = (degreesOfFreedom + 1)
        / (degreesOfFreedom + squaredResiduals / oldVariance);
    // C) Calculate new variance
    newVariance = mean(weights.mul(squaredResiduals), nanMask)[0];
  }
    // D) test if the variance has converged. (delta < 0.5%)
  while (abs(newVariance - oldVariance) > 0.005 * newVariance);

  return weights;
}

// The old lk_track functionality
Pose Tracking::iterateTracking(cv::Mat jacobian, Pose poseHypothesis, cv::Mat intensityImage, double fLength) {
  // 1) Compute residuals
  cv::Mat residualsImage = computeResiduals(poseHypothesis, intensityImage, fLength);
  cv::namedWindow("residuals", CV_WINDOW_AUTOSIZE);
  cv::imshow("residuals", residualsImage + 0.5);
  //cv::Mat residualsT = residuals.t();
  //residuals = residualsT.reshape(1, residuals.cols*residuals.rows); // refactor this into the method
  cv::Mat residuals = residualsImage.reshape(1, residualsImage.cols * residualsImage.rows);
  // if error is small enough, return.
  // TODO

  // 2) Compute the matrix of weights
  cv::Mat weights = computeWeights(residuals);
  weights = weights.reshape(1, weights.cols * weights.rows);

  // 3) Solve the matrix equation.
  cv::Mat pose_values;
  {
    // A) Compute J.T * W (it's used twice)
    cv::Mat jtw = jacobian.t().mul(repeat(weights.t(), 6, 1));
    //cv::Mat jtw = jacobian.t();

    // B) Compute the Gauss-Newton approximate Hessian, J.T * W * J
    cv::Mat approximateHessian = jtw * jacobian;
    // C) Compute the c [constant factor]
    cv::Mat jtwr = jtw * residuals;
    std::cout << "jtwr:\n" << jtwr << std::endl;
    double c = jtwr.dot(jtwr) / jtwr.dot(approximateHessian * jtwr);
    std::cout << "c:" << c << std::endl;
    if (std::isnan(c)) {
      c = 0.0;
    }
    pose_values = - c * jtwr * 4;
    std::cout << "pose_values:\n" << pose_values << std::endl;

    // show the x values of J.T .* r
    cv::Mat bJacobian(jacobian.rows, 1, CV_64FC1);
    jacobian(cv::Range::all(), cv::Range(4, 5)).copyTo(bJacobian);
    bJacobian = bJacobian.reshape(1, residualsImage.rows);
    //cv::imshow("bJacobian times residuals", bJacobian.mul(residualsImage) + 0.5);
    cv::waitKey();
  }

  return Pose(pose_values);
}

Pose Tracking::testIterationOfTracking(cv::Mat intensityImage, double fLength) {
  Pose poseHypothesis(0, 0, 0, 0, 0, 0);
  cv::Mat jacobian = computeJacobian(fLength, intensityImage.rows, intensityImage.cols);
  Pose measuredPose = iterateTracking(jacobian, poseHypothesis, intensityImage, fLength);
  measuredPose.print();
  return measuredPose;
}

Pose Tracking::track(cv::Mat intensityImage, double fLength) {
  Pose poseHypothesis;
  //assert(intensityImage.rows % 4 == 0 && intensityImage.cols % 4 == 0); // gotta be whole pixels.
  // todo: just take off the last 1-3 pixels if not a multiple of 4.

  int downsampleFactors[] = {4, 2, 1};
  int iterationsPerSample[] = {8, 3, 3};
  int sampleRateCount = sizeof(downsampleFactors) / sizeof(downsampleFactors[0]);
  std::cout << sampleRateCount;
  assert(sampleRateCount == sizeof(iterationsPerSample) / sizeof(iterationsPerSample[0]));

  for (int i = 0; i < sampleRateCount; i++) {
    cv::Mat downsampled;
    cv::resize(intensityImage, downsampled, cv::Size(), 1.0/downsampleFactors[i], 1.0/downsampleFactors[i]);
    cv::Mat jacobian = computeJacobian(fLength / downsampleFactors[i], downsampled.rows, downsampled.cols);
    for (int j = 0; j < iterationsPerSample[i]; j++) {
      Pose poseDelta = iterateTracking(jacobian, poseHypothesis, downsampled, fLength / downsampleFactors[i]);
      poseHypothesis = poseHypothesis.lkUpdate(poseDelta); // this is still mostly garbage now
      poseHypothesis.print();
    }
  }
  return poseHypothesis;
}

}