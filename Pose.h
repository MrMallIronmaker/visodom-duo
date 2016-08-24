#ifndef OMAR_POSE_H_
#define OMAR_POSE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace omar {

class Pose {
    /**
     * A pose is a combination of a 3D rotation and 3D translation.
     */

private:
    /**
     * Translations and Rotations in OpenCV matrix form.
     */
    cv::Mat t;
    cv::Mat R;


public:
    /**
     * Converts the points from one camera to another.
     */
    //static Mat camera2camera(Pose in_camera, Mat points, Pose outCamera);

    /**
     * creates a new Pose object with the identity transformation.
     */
    Pose();

    /**
     * creates a new Pose object with translation / Euler angle rotations.
     * Variables x, y, and z, refer to translations.
     * Variables a, b, and g are short for alpha, beta, and gamma, and refer to
     * rotations on the x, y, and z axes in units of radians.
     * NOTE THE AXES OF ROTATION DO NOT ROTATE.
     */
    Pose(double _x, double _y, double _z, double _a, double _b, double _g);

    /**
     * Todo
     */
    Pose(cv::Mat pose_values);

    Pose(cv::Matx33d _R, cv::Matx31d _t);

    void set_from_twist(double _x, double _y, double _z, double _a, double _b, double _g);

    /**
     * Transforms a set of openCV points (represented as a Mat)
     * Generally, the representaiton that we use is camera = T*world,
     * where camera and world are 3n matricies and T is the transformation
     */
    cv::Mat world2camera(cv::Mat points);

    /**
     * Transforms a set of openCV points (represented as a Mat)
     */
    cv::Mat camera2world(cv::Mat points);

    /**
     * TODO
     */
    void print();
    cv::Matx16d getTwist();
    Pose lkUpdate(Pose other);

    bool approximatelyEquals(Pose other);
};

} // end namespace omar

#endif // OMAR_POSE_H_