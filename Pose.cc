#include <math.h>
#include <iostream>
#include "Pose.h"

namespace omar {

Pose::Pose() {
    t = cv::Mat(3, 1, CV_64FC1, cv::Scalar(0.0));
    R = cv::Mat::eye(3, 3, CV_64FC1);
}

Pose::Pose(cv::Mat pose_values) {
    set_from_twist(pose_values.at<double>(0),
                   pose_values.at<double>(1),
                   pose_values.at<double>(2),
                   pose_values.at<double>(3),
                   pose_values.at<double>(4),
                   pose_values.at<double>(5));
}

Pose::Pose(double _x, double _y, double _z, double _a, double _b, double _g) {
    set_from_twist(_x, _y, _z, _a, _b, _g);
}

Pose::Pose(cv::Matx33d _R, cv::Matx31d _t) {
    t = cv::Mat(_t);
    R = cv::Mat(_R);
}

void Pose::set_from_twist(double _x, double _y, double _z, double _a, double _b, double _g) {
    t = (cv::Mat_<double>(3, 1) << _x, _y, _z);
    double rx_input_data[3][3] = {
            {1,        0,         0},
            {0,  cos(_a),  -sin(_a)},
            {0,  sin(_a),   cos(_a)}
    };
    cv::Mat RX(3, 3, CV_64FC1, rx_input_data);

    double ry_input_data[3][3] = {
            { cos(_b), 0, sin(_b)},
            {       0, 1,       0},
            {-sin(_b), 0, cos(_b)}
    };
    cv::Mat RY(3, 3, CV_64FC1, ry_input_data);

    double rz_input_data[3][3] = {
            {cos(_g), -sin(_g), 0},
            {sin(_g),  cos(_g), 0},
            {      0,        0, 1}
    };
    cv::Mat RZ(3, 3, CV_64FC1, rz_input_data);

    R = RZ * RY * RX; // note that RX is applied first when some point or transform comes through.
}

cv::Matx16d Pose::getTwist() {
    double a = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    double b = atan2(-R.at<double>(2, 0),
                     sqrt(pow(R.at<double>(2, 1), 2) + pow(R.at<double>(2, 2), 2)));
    double g = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    cv::Matx16d twist(t.at<double>(0), t.at<double>(1), t.at<double>(2), a, b, g);
    return twist;
}

void Pose::print() {
    // print in twist form.
    cv::Matx16d twist = getTwist();

    std::cout << "\nX translation (right): " << twist(0)
    << "\nY translation (down): " << twist(1)
    << "\nZ translation (forward): " << twist(2)
    << "\nAlpha rotation (pitch up): " << twist(3)
    << "\nBeta rotation (yaw right): " << twist(4)
    << "\nGamma rotation (roll clockwise): " << twist(5)
    << "\n";
}

cv::Mat Pose::world2camera(cv::Mat world_points) {
    // Returns the position of each point relative to this camera pose.
    // first translate, then rotate.
    // TODO: ensure points is a 3xn matrix
    cv::Mat translate = repeat(t, 1, world_points.cols);
    return R * world_points + translate;
}

cv::Mat Pose::camera2world(cv::Mat camera_points) {
    // Calculates the points from the camera's frame of reference to the world frame of reference
    // TODO: ensure points is a 3xn matrix
    cv::Mat translate = repeat(t, 1, camera_points.cols);
    cv::Mat R_T;
    transpose(R, R_T);
    return R_T * (camera_points - translate);
}

bool Pose::approximatelyEquals(Pose other) {
    // todo: have threshold as an argument
    double threshold = 0.00001;
    cv::Mat diffR = abs(R - other.R) > 2 * threshold;
    cv::Mat diffT = abs(t - other.t) > 2 * threshold;

    return cv::countNonZero(diffR) == 0 && cv::countNonZero(diffT) == 0;
}

Pose Pose::lkUpdate(Pose delta) {
    // template [i.e. point cloud image]
    // image [i.e. input trackable]
    // T(delta * pts) = I (thisPose * pts)
    // Pose gets applied to the trackable image, so
    // T (pts) = I (newPose * pts)
    // therefore: newPose = thisPose * inverse(delta)
    // R_n x + t_n = R_o( R_d^T (x - t_d)) + t_o
    // [linear algebra]
    // R_n = R_o * R_d^T, t_n = -R_o * R_d^T * t_d + t_o

    cv::Mat rNew = R * delta.R.t();
    cv::Mat tNew = -R * delta.R.t() * delta.t + t;
    return Pose(rNew, tNew);

}

} // namespace omar