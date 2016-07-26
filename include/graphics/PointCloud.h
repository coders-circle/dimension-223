#pragma once
#include <backend/LensBlurImage.h>
#include <graphics/Mesh.h>
#include <graphics/Transformation.h>
#include <physics/Object.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

/**
 * A collection of vertices or points in space like a mesh, but untriangulated.
 * It consists of additional functions exclusively to operate on point clouds.
 */
class PointCloud {
public:
    PointCloud(const std::string& path,
               const std::vector<glm::ivec2>& floorPixels);

    std::vector<glm::vec3>& getPoints() { return mPoints; }

    const std::vector<glm::ivec2>& getImageCoordinates() const {
        return mImageCoordinates;
    }

    void destroy() {
        glDeleteBuffers(1, &mVbo);
        glDeleteBuffers(1, &mIbo);
        glDeleteVertexArrays(1, &mVao);
    }

    void draw(const Program& program, const glm::mat4& viewProjection);

    int getWidth() { return mLensBlurImage.getImage().cols; }
    int getHeight() { return mLensBlurImage.getImage().rows; }
    int getDepthWidth() { return mLensBlurImage.getDepthMap().cols; }
    int getDepthHeight() { return mLensBlurImage.getDepthMap().rows; }

    Transformation transformation;

    std::string getPath() const {
        return mLensBlurImage.getFilename();
    }

    Object* getObject() const {
        return mObject;
    }

    LensBlurImage& getImage() {
        return mLensBlurImage;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPclCloud() {
        return mPclCloud;
    }

private:
    LensBlurImage mLensBlurImage;
    std::vector<glm::vec3> mPoints;
    std::vector<glm::ivec2> mImageCoordinates;
    // std::vector<glm::ivec2> mFloorPixels;

    btTriangleMesh* mTriangleMesh;
    btCollisionShape* mTriangleMeshShape;
    Object* mObject;

    std::vector<float> mVertices;
    std::vector<GLuint> mIndices;
    GLuint mVbo, mIbo, mVao;
    Texture mTexture;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mPclCloud;
};
