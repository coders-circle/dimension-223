#pragma once
#include <backend/InputData.h>
#include <backend/Surface.h>

#include <graphics/Mesh.h>
#include <graphics/Transformation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


class PointCloud {
public:
    PointCloud(const InputData& inputData);

    void setSurfaces(const std::vector<Surface>& surfaces) {
        mSurfaces = surfaces;
    }

    std::vector<Surface>& getSurfaces() {
        return mSurfaces;
    }

    void reconstruct();

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

    int getWidth() { return mInputData.getImage().cols; }
    int getHeight() { return mInputData.getImage().rows; }
    int getDepthWidth() { return mInputData.getDepthMap().cols; }
    int getDepthHeight() { return mInputData.getDepthMap().rows; }

    InputData& getInputData() { return mInputData; }

    Transformation transformation;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getPclCloud() {
        return mPclCloud;
    }

private:
    InputData mInputData;
    bool mConstructed;

    std::vector<glm::vec3> mPoints;
    std::vector<glm::ivec2> mImageCoordinates;

    std::vector<float> mVertices;
    std::vector<GLuint> mIndices;
    GLuint mVbo, mIbo, mVao;
    Texture mTexture;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mPclCloud;

    std::vector<Surface> mSurfaces;
};
