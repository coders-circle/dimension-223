#include <stdinc.h>
#include <backend/PointsMatcher.h>


PointsMatcher::PointsMatcher(PointCloud& pointCloud1, PointCloud& pointCloud2)
    : mPointCloud1(pointCloud1), mPointCloud2(pointCloud2)
{}


std::pair<index_list, index_list> PointsMatcher::getMatchingPoints() {

    // LensBlurImage& limage1 = mPointCloud1.getImage();
    // LensBlurImage& limage2 = mPointCloud2.getImage();
    //
    // cv::Mat& image1 = limage1.getImage();
    // cv::Mat& image2 = limage2.getImage();

    // Assume 1 is left and 2 is right image.
    // Start searching points from right of 1 and left of 2.
    //
    // For each point from right of 1.
    // Find min SAD point from left of 2.
    // If min is greater than MIN, reject this point pair.
    // Else add in the list.
    //
    // F*** ! This won't work. What have I been thinking?
    // 

    // for (int i=0; i<image1.rows; ++i) {
    //     for (int j=image1.cols-1; j>=0; --j) {
    //
    //     }
    // }

    return std::pair<index_list, index_list>(index_list(), index_list());
}

float PointsMatcher::calculateSAD(int i1, int j1, int i2, int j2, int window) {
    return 0;
}
