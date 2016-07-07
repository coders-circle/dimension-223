#pragma once
#include <backend/LensBlurImage.h>
#include <graphics/Mesh.h>
#include <graphics/Transformation.h>


/**
 * A collection of vertices or points in space like a mesh, but untriangulated.
 * It consists of additional functions exclusively to operate on point clouds.
 */
class PointCloud {
public:
    PointCloud(const std::string& path);
    std::vector<glm::vec3>& getPoints() { return mPoints; }

    void destroy() {
        glDeleteBuffers(1, &mVbo);
        glDeleteBuffers(1, &mIbo);
        glDeleteVertexArrays(1, &mVao);
    }

    void draw(const Program& program, const glm::mat4& viewProjection);

    int getWidth() { return mLensBlurImage.getImage().cols; }
    int getHeight() { return mLensBlurImage.getImage().rows; }

    Transformation transformation;

private:
    LensBlurImage mLensBlurImage;
    std::vector<glm::vec3> mPoints;

    std::vector<float> mVertices;
    std::vector<GLuint> mIndices;
    GLuint mVbo, mIbo, mVao;
    Texture mTexture;
};
