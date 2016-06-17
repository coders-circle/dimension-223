#pragma once
#include <backend/PointsMatcher.h>
#include <graphics/PointCloud.h>


class CloudStitcher {
public:
    CloudStitcher(PointCloud& pointCloud1, PointCloud& pointCloud2);

    void setTransformation(const glm::mat3& rotation,
            const glm::vec3& translation) {
        mRotation = rotation;
        mTranslation = translation;
    }

    void stitch();

private:
    PointsMatcher mMatcher;

    glm::mat3 mRotation;
    glm::vec3 mTranslation;

    unsigned int getClosest(glm::vec3& v, std::vector<glm::vec3>& points);
};
