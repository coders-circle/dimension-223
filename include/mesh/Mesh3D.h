#pragma once
#include <shaders/Material.h>


/**
 * Vertex structure that stores position, normal and texture coordinates
 * of a 3D vertex.
 */
struct Vertex3D {
    /// 3D position vector of the vertex.
    glm::vec3 position;
    /// 3D normal vector of the vertex.
    glm::vec3 normal;
    /// 2D texture coordinates of the vertex.
    glm::vec2 texCoords;

    /**
     * Construct a vertex from given position, normal and texture coordinates.
     * @param position  3D position vector.
     * @param normal    3D normal vector.
     * @param texCoords 2D texture coordinates.
     */
    Vertex3D(const glm::vec3& position,
        const glm::vec3& normal, const glm::vec2& texCoords)
        : position(position), normal(normal), texCoords(texCoords)
    {}

    /**
     * Construct a vertex from given position.
     *
     * Normal and texture coordinates are set to zero vectors.
     * @param position  3D position vector.
     */
    Vertex3D(const glm::vec3& position)
        : position(position) {}

    /**
     * Construct a zero vertex.
     *
     * Each vector (position, normal and texCoords) are set as zero vectors.
     */
    Vertex3D() {}
};


/**
 * A 3D Mesh that can be rendered with given material.
 */
class Mesh3D
{
public:
    /**
     * Construct a mesh with given list of vertices and indices.
     * @param vertices  Array of mesh vertices.
     * @param indices   Array of indices to the vertices to create triangles.
     */
    Mesh3D(const std::vector<Vertex3D>& vertices,
        const std::vector<GLuint>& indices);

    /**
     * Construct a cuboidal mesh.
     * @param extents   Length of cuboid in each dimension.
     * @param offset    Offset of the center of the mesh.
     */
    Mesh3D(const glm::vec3& extents,
        const glm::vec3& offset=glm::vec3(0));

     /// Destroy the OpenGL objects representing this mesh.
    ~Mesh3D()
    {
        glDeleteVertexArrays(1, &m_vao);
        glDeleteBuffers(1, &m_vbo);
        glDeleteBuffers(1, &m_ebo);
    }

    /**
     * Render the mesh with given material and transformations.
     * @param material Pointer to Material to render this mesh with.
     * @param model    World transformation matrix.
     * @param vp       View-Projection transformation matrix.
     */
    void Render(Material* material, const glm::mat4& model,
        const glm::mat4& vp);

    /**
     * Get number of triangles contained in this mesh.
     * @return Total number of triangles in this mesh.
     */
    size_t GetNumTriangles() const { return m_numElements/3; }

private:
    void Build(const std::vector<Vertex3D>& vertices,
        const std::vector<GLuint>& indices);

    GLuint m_vao, m_vbo, m_ebo, m_numElements;
};
