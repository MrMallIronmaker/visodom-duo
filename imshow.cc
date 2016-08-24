#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(int argc, char **argv) {
    char* filename = "/home/mark/cs543/visodom/block.png";
    if (argc == 2) {
        std::cout << " Usage: display_image ImageToLoadAndDisplay" << std::endl;
        std::cout << " Using " << filename << " by default." << std::endl;
        filename = argv[1];
    }

    cv::Mat image;
    image = cv::imread(filename, CV_LOAD_IMAGE_COLOR);   // Read the file

    if (!image.data)                              // Check for invalid input
    {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
    imshow("Display window", image);                   // Show our image inside it.

    cv::waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}