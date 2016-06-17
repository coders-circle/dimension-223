#pragma once
#include <graphics/Mesh.h>
#include <graphics/Texture.h>


/**
 * A collection of vertices or points in space like a mesh, but untriangulated.
 * It consists of additional functions exclusively to operate on point clouds.
 */
class PointCloud {
public:
    PointCloud(cv::Mat& image, cv::Mat& depthMap);
    std::vector<glm::vec3>& getPoints() { return mPoints; }

    ~PointCloud() {
        glDeleteBuffers(1, &mVbo);
        glDeleteBuffers(1, &mIbo);
        glDeleteVertexArrays(1, &mVao);
    }

    void draw(const Program& program, const glm::mat4& model,
        const glm::mat4& viewProjection);

private:
    cv::Mat& mImage;
    cv::Mat& mDepthMap;
    std::vector<glm::vec3> mPoints;

    std::vector<float> mVertices;
    GLuint mVbo, mIbo, mVao;
    Texture mTexture;
};
