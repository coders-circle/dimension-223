#include <stdinc.h>


void blur(std::vector<glm::vec3>& points, int width, int height)
{
    cv::Mat image(height, width, CV_32FC3, &points[0][0]);
    cv::Mat output = image.clone();
    cv::GaussianBlur(image, output, cv::Size(9,9), 0);

    for (int i=0; i<height; ++i) {
        for (int j=0; j<width; ++j) {
            cv::Vec3f pixel = output.at<cv::Vec3f>(cv::Point(j, i));
            points[i*width+j] = glm::vec3(pixel[0], pixel[1], pixel[2]);
        }
    }
}
