#pragma once
#include <graphics/Mesh.h>


/**
 * A collection of vertices or points in space like a mesh, but untriangulated.
 * It consists of additional functions exclusively to operate on point clouds.
 */
class PointCloud {
public:
    PointCloud(cv::Mat& image, cv::Mat& depthMap);
    std::vector<glm::vec3>& getPoints() { return mPoints; }

    void destroy() {
        glDeleteBuffers(1, &mVbo);
        glDeleteBuffers(1, &mIbo);
        glDeleteVertexArrays(1, &mVao);
    }

    void draw(const Program& program, const glm::mat4& model,
              const glm::mat4& viewProjection);

    int getWidth() const { return mImage.cols; }
    int getHeight() const { return mImage.rows; }

private:
    cv::Mat& mImage;
    cv::Mat& mDepthMap;
    std::vector<glm::vec3> mPoints;

    std::vector<float> mVertices;
    std::vector<GLuint> mIndices;
    GLuint mVbo, mIbo, mVao;
    Texture mTexture;
};
