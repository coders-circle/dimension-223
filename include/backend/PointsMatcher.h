#pragma once
#include <graphics/PointCloud.h>


class PointsMatcher {
public:
    PointsMatcher(PointCloud& pointCloud1, PointCloud& pointCloud2,
                  const Area& area1, const Area& area2);

    PointCloud& getPointCloud1() { return mPointCloud1; }
    PointCloud& getPointCloud2() { return mPointCloud2; }

    const std::vector<glm::vec3>& getPoints1() const { return mPoints1; }
    const std::vector<glm::vec3>& getPoints2() const { return mPoints2; }

    int getWidth2() const {
        return mWidth2;
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr getPclCloud1() {
    //     return mPclCloud1;
    // }
    //
    // pcl::PointCloud<pcl::PointXYZ>::Ptr getPclCloud2() {
    //     return mPclCloud2;
    // }

private:
    PointCloud& mPointCloud1;
    PointCloud& mPointCloud2;

    // Area mArea1;
    // Area mArea2;

    std::vector<glm::vec3> mPoints1;
    std::vector<glm::vec3> mPoints2;
    //
    // pcl::PointCloud<pcl::PointXYZ>::Ptr mPclCloud1;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr mPclCloud2;

    int mWidth1, mWidth2;
};
