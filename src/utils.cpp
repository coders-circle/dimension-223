#include <stdinc.h>


void blur(std::vector<glm::vec3>& points, int width, int height)
{
    cv::Mat image(height, width, CV_32FC3, &points[0][0]);
    // cv::medianBlur(image, image, 5);
    cv::blur(image, image, cv::Size(9,9), cv::Point(-1,-1));
    // cv::GaussianBlur(image, image, cv::Size(9,9), 0);
    // cv::bilateralFilter(output, image, 9, 75, 75);

    for (int i=0; i<height; ++i) {
        for (int j=0; j<width; ++j) {
            cv::Vec3f pixel = image.at<cv::Vec3f>(cv::Point(j, i));
            points[i*width+j] = glm::vec3(pixel[0], pixel[1], pixel[2]);
        }
    }
}
