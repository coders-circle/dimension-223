#pragma once

#include <graphics/Program.h>
#include <graphics/Texture.h>


/**
 * A 3D vertex with position, normal and texture coordinates.
 */
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoords;
};


/**
 * An OpenGL mesh made up of vertices and indices.
 */
class Mesh {
public:
    Mesh(const std::vector<Vertex>& vertices,
         const std::vector<GLuint>& indices);

    void draw(const Program& program, const glm::mat4& model,
              const glm::mat4& viewProjection, Texture& texture);

    void destroy() {
        glDeleteBuffers(1, &mVbo);
        glDeleteBuffers(1, &mEbo);
        glDeleteVertexArrays(1, &mVao);
    }

private:
    GLuint mVao, mVbo, mEbo;
    size_t mNumIndices;
};
