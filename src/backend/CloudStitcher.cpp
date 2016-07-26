#include <stdinc.h>
#include <backend/CloudStitcher.h>


CloudStitcher::CloudStitcher(PointCloud& pointCloud1, PointCloud& pointCloud2,
                             const Area& area1, const Area& area2)
    : mMatcher(pointCloud1, pointCloud2, area1, area2),
      mRotation(glm::mat3()), mTranslation(0)
{}


void CloudStitcher::stitch() {

    /*
    const std::vector<glm::vec3>& pointCloud1 = mMatcher.getPoints1();
    const std::vector<glm::vec3>& pointCloud2 = mMatcher.getPoints2();

    if (pointCloud1.size() == 0 || pointCloud2.size() == 0) {
        std::cout << "No intersection area" << std::endl;
        return;
    }

    std::vector<glm::vec3> transformedPointCloud2(pointCloud2.size());
    std::vector<unsigned int> temp(pointCloud1.size());

    for (unsigned int i=0; i<pointCloud2.size(); ++i) {
        transformedPointCloud2[i] = pointCloud2[i];
    }

    // while (true) {
    for (unsigned int iter=0; iter<15; ++iter) {
        std::cout << "Iter #" << iter << std::endl;

        // Step 1: Initialization
        glm::mat4 transformation = glm::translate(glm::mat4(), mTranslation)
            * glm::mat4(mRotation);

        // Step 2: Matching
        for (unsigned int i=0; i<transformedPointCloud2.size(); ++i) {
            transformedPointCloud2[i] = glm::vec3(transformation *
                glm::vec4(transformedPointCloud2[i], 1.0f));
        }

#define pc2 transformedPointCloud2
#define pc1 pointCloud1

        // Match first point
        unsigned int error;
        unsigned int temp_temp = temp[0];
        temp[0] = getClosest(pc1[0], pc2);
        error = glm::abs(temp[0] - temp_temp);

        // Match the rest considering the neighbourhood of previous match.
        int width = mMatcher.getWidth2();
#define WINDOW_SIZE 10

        for (unsigned int i=1; i<pc1.size(); ++i) {
            temp_temp = temp[i];
            int startIndex = temp[i-1] - width*WINDOW_SIZE/2 - WINDOW_SIZE/2 + 1;
            temp[i] = getClosest(pc1[i], pc2, startIndex, WINDOW_SIZE,
                WINDOW_SIZE, width);
            error += glm::abs(temp[i] - temp_temp);
        }

        // if (error <= 5) {
        //     break;
        // }

        // Step 3: Transformation / Error Minimization
        glm::vec3 a_bar = getCentroid(pointCloud1);
        glm::vec3 b_bar = getCentroid(transformedPointCloud2);

        std::vector<glm::vec3> a_dash(pointCloud1.size());
        std::vector<glm::vec3> b_dash(pointCloud1.size());
        for (unsigned int i=0; i<pointCloud1.size(); i+=1) {
            a_dash[i] = pointCloud1[i] - a_bar;
            b_dash[i] = transformedPointCloud2[temp[i]] - b_bar;
        }

        // Calculate matrix N and its SVD.
        glm::mat3 mat_n = glm::mat3(0);
        for (unsigned int i=0; i<pointCloud1.size(); i+=1) {
            mat_n += outerProduct(a_dash[i], b_dash[i]);
        }

        glm::mat3 u, e, v;
        svd(mat_n, u, e, v);

        // Update rotation and translation matrices.
        mRotation = v*glm::transpose(u);
        if (glm::determinant(mRotation) < 0) {
            v[0][2] *= -1;
            v[1][2] *= -1;
            v[2][2] *= -1;
            mRotation = v*glm::transpose(u);
        }
        // std::cout << glm::to_string(mat_n) << std::endl;
        // std::cout << glm::to_string(u) << std::endl;
        // std::cout << glm::to_string(v) << std::endl;
        // std::cout << glm::to_string(mRotation) << std::endl;

        mTranslation = -mRotation * b_bar + a_bar;
    }
    // std::cout << glm::to_string(mTranslation) << std::endl;
    // std::cout << glm::to_string(mRotation) << std::endl;
    */

    auto pc1 = mMatcher.getPclCloud1();
    auto pc2 = mMatcher.getPclCloud2();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pc2);
    icp.setInputTarget(pc1);
    
    pcl::PointCloud<pcl::PointXYZ> pc3;
    icp.align(pc3);
    std::cout << icp.getFitnessScore() << std::endl;

    mTransformation = glm::make_mat4(icp.getFinalTransformation().data());
}

unsigned int CloudStitcher::getClosest(const glm::vec3& v,
                                       const std::vector<glm::vec3>& points) {
    float minDist = 99999;
    unsigned int minId = 0;

    for (unsigned int i=0; i<points.size(); ++i) {
        float dist = glm::length2(points[i]- v);
        if (dist < minDist) {
            minDist = dist;
            minId = i;
        }
    }
    return minId;
}

unsigned int CloudStitcher::getClosest(const glm::vec3& v,
                                       const std::vector<glm::vec3>& points,
                                       int startIndex, int cols, int rows,
                                       int skip) {
    float minDist = 99999;
    int minId = startIndex;

    for (int i=0; i<rows; ++i) {
        for (int j=0; j<cols; ++j) {
            int index = startIndex + (i*skip + j);
            if (index < 0)
                continue;
            if (index >= (int)points.size())
                break;
            float dist = glm::length2(points[index] - v);
            if (dist < minDist) {
                minDist = dist;
                minId = index;
            }
        }
    }
    return minId;
}
