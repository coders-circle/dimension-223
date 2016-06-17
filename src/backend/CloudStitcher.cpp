#include <stdinc.h>
#include <backend/CloudStitcher.h>


CloudStitcher::CloudStitcher(PointCloud& pointCloud1, PointCloud& pointCloud2)
    : mMatcher(pointCloud1, pointCloud2)
{}


void CloudStitcher::stitch() {
    std::vector<glm::vec3>& pointCloud1 = mMatcher.getPointCloud1().getPoints();
    std::vector<glm::vec3>& pointCloud2 = mMatcher.getPointCloud2().getPoints();
    std::vector<glm::vec3> transformedPointCloud2(pointCloud2.size());
    std::vector<unsigned int> temp(pointCloud1.size());

    for (unsigned int i=0; i<pointCloud2.size(); ++i) {
        transformedPointCloud2[i] = pointCloud2[i];
    }

    while (true) {

        // Step 1: Initialization
        glm::mat4 transformation = glm::mat4(mRotation) *
            glm::translate(glm::mat4(), mTranslation);

        // Step 2: Matching
        for (unsigned int i=0; i<transformedPointCloud2.size(); ++i) {
            transformedPointCloud2[i] = glm::vec3(transformation *
                glm::vec4(transformedPointCloud2[i], 1.0f));
        }

        unsigned int error;
        for (unsigned int i=0; i<pointCloud1.size(); ++i) {
            unsigned int x = temp[i];
            temp[i] = getClosest(pointCloud1[i], transformedPointCloud2);
            error += glm::abs(temp[i]-x);
        }

        // Termination condition
        if (error <= 50) {
            break;
        }

        // Step 3: Transformation / Error Minimization
        glm::vec3 a_bar = getCentroid(pointCloud1);
        glm::vec3 b_bar = getCentroid(transformedPointCloud2);

        std::vector<glm::vec3> a_dash(pointCloud1.size());
        std::vector<glm::vec3> b_dash(pointCloud1.size());
        for (unsigned int i=0; i<pointCloud1.size(); ++i) {
            a_dash[i]= pointCloud1[i]- a_bar;
            b_dash[i]= transformedPointCloud2[temp[i]] - b_bar;
        }

        // Calculate matrix N and its SVD.
        glm::mat3 mat_n = glm::mat3(0);
        for (unsigned int i=0; i<pointCloud1.size(); ++i) {
            mat_n += outerProduct(a_dash[i], b_dash[i]);
        }

        glm::mat3 u, v, e;
        svd(mat_n, u, v, e);

        // Update rotation and translation matrices.
        mRotation = v * glm::transpose(u);
        mTranslation = mRotation * a_bar - b_bar;

    }
}

unsigned int CloudStitcher::getClosest(glm::vec3& v, std::vector<glm::vec3>& points) {
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
