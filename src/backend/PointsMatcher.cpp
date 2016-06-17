#include <stdinc.h>
#include <backend/PointsMatcher.h>


PointsMatcher::PointsMatcher(PointCloud& pointCloud1, PointCloud& pointCloud2)
    : mPointCloud1(pointCloud1), mPointCloud2(pointCloud2)
{}


std::pair<index_list, index_list> PointsMatcher::getMatchingPoints() {
    return std::pair<index_list, index_list>(index_list(), index_list());
}
