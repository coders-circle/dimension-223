#include <stdinc.h>
#include <backend/PointsMatcher.h>


PointsMatcher::PointsMatcher(PointCloud& pointCloud1, PointCloud& pointCloud2,
                             const Area& area1, const Area& area2)
    : mPointCloud1(pointCloud1), mPointCloud2(pointCloud2),
      mPclCloud1(new pcl::PointCloud<pcl::PointXYZ>),
      mPclCloud2(new pcl::PointCloud<pcl::PointXYZ>)
    //   mArea1(area1), mArea2(area2)
{

    const std::vector<glm::vec3>& points1 = pointCloud1.getPoints();
    const std::vector<glm::ivec2>& iPoints1 = pointCloud1.getImageCoordinates();
    const std::vector<glm::vec3>& points2 = pointCloud2.getPoints();
    const std::vector<glm::ivec2>& iPoints2 = pointCloud2.getImageCoordinates();

    int cnt = 0;
    bool yFirst = true;
    float lastX = -9999999;
    int index = 0;
    for (size_t i=0; i<iPoints1.size(); ++i) {
        if (area1.contains(iPoints1[i].x, iPoints1[i].y)) {
            if (yFirst) {
                if (iPoints1[i].x >= lastX) {
                    lastX = iPoints1[i].x;
                    cnt++;
                }
                else
                    yFirst  = false;
            }
            mPoints1.push_back(points1[i]);

            mPclCloud1->push_back(pcl::PointXYZ(
                points1[i].x, points1[i].y, points1[i].z
            ));
            index++;
        }
    }
    mWidth1 = cnt;


    cnt = 0;
    yFirst = true;
    lastX = -9999999;
    for (size_t i=0; i<iPoints2.size(); ++i) {
        if (area2.contains(iPoints2[i].x, iPoints2[i].y)) {
            if (yFirst) {   // First row, then width+=1.
                if (iPoints1[i].x >= lastX) {
                    lastX = iPoints1[i].x;
                    cnt++;
                }
                else
                    yFirst  = false;
            }
            mPoints2.push_back(points2[i]);

            mPclCloud2->push_back(pcl::PointXYZ(
                points2[i].x, points2[i].y, points2[i].z
            ));
        }
    }
    mWidth2 = cnt;

    mPclCloud1->is_dense = false;
    mPclCloud2->is_dense = false;

    mPclCloud1->height = mPclCloud2->height = 1;
    mPclCloud1->width = mPclCloud1->points.size();
    mPclCloud2->width = mPclCloud2->points.size();
}
