#pragma once
#include <backend/PointsMatcher.h>
#include <graphics/PointCloud.h>


/**
 * Point Clouds Stitcher that combines two clouds using ICP.
 */
class CloudStitcher {
public:
    CloudStitcher(PointCloud& pointCloud1, PointCloud& pointCloud2,
                  const Area& area1, const Area& area2);

    void setTransformation(const glm::mat3& rotation,
                           const glm::vec3& translation) {
        mRotation = rotation;
        mTranslation = translation;
    }

    glm::mat4 getTransformation() {
        // return glm::translate(glm::mat4(), mTranslation) * glm::mat4(mRotation);
        return mTransformation;
    }

    void stitch();

private:
    PointsMatcher mMatcher;

    glm::mat3 mRotation;
    glm::vec3 mTranslation;

    glm::mat4 mTransformation;

    unsigned int getClosest(const glm::vec3& v,
                            const std::vector<glm::vec3>& points);
    unsigned int getClosest(const glm::vec3& v,
                            const std::vector<glm::vec3>& points,
                            int startIndex, int cols, int rows, int skip);
};
