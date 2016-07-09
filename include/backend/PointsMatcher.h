#pragma once
#include <graphics/PointCloud.h>

typedef std::vector<unsigned int> index_list;


// TODO.
class PointsMatcher {
public:
    PointsMatcher(PointCloud& pointCloud1, PointCloud& pointCloud2);

    std::pair<index_list, index_list> getMatchingPoints();

    PointCloud& getPointCloud1() { return mPointCloud1; }
    PointCloud& getPointCloud2() { return mPointCloud2; }

private:
    PointCloud& mPointCloud1;
    PointCloud& mPointCloud2;

};
