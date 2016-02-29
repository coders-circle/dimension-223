#include <stdinc.h>
#include <mesh/Mesh3D.h>


Mesh3D::Mesh3D(const std::vector<Vertex3D>& vertices,
    const std::vector<GLuint>& indices)
{
    Build(vertices, indices);
}

void Mesh3D::Build(const std::vector<Vertex3D>& vertices,
    const std::vector<GLuint>& indices)
{
    // Generate buffers and arrays
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    glGenVertexArrays(1, &m_vao);


    glBindVertexArray(m_vao);

    // Set vertices data to VBO
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex3D),
        &vertices[0], GL_STATIC_DRAW);

    // Set indices data to EBO
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()
        * sizeof(GLuint), &indices[0], GL_STATIC_DRAW);

    // Vertex Attributes
    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
        (GLvoid*)offsetof(Vertex3D, position));
    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
        (GLvoid*)offsetof(Vertex3D, normal));
    // Texture coordinates
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex3D),
        (GLvoid*)offsetof(Vertex3D, texCoords));
    // ...

    glBindVertexArray(0);

    m_numElements = indices.size();
}


void Mesh3D::Render(Material* material, const glm::mat4& model,
    const glm::mat4& vp)
{
    if (material)
    {
        material->Use();

        Program& p = material->GetProgram();
        if (p.UsesStdTransforms())
        {
            p.SetUniform("mvpMatrix", vp * model);
            p.SetUniform("modelMatrix", model);
        }
    }

    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_numElements, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}


Mesh3D::Mesh3D(const glm::vec3& extents, const glm::vec3 &offset)
{
    using namespace glm;
    float halfSizeX = extents[0] / 2.0f;
    float halfSizeY = extents[1] / 2.0f;
    float halfSizeZ = extents[2] / 2.0f;

    std::vector<Vertex3D> vertices = {
        // front
        { vec3(-halfSizeX, -halfSizeY, halfSizeZ), vec3(0.0, 0.0, 1.0), vec2(0.0, 0.0) },
        { vec3(halfSizeX, -halfSizeY, halfSizeZ), vec3(0.0, 0.0, 1.0), vec2(1.0, 0.0) },
        { vec3(halfSizeX, halfSizeY, halfSizeZ), vec3(0.0, 0.0, 1.0), vec2(1.0, 1.0) },
        { vec3(-halfSizeX, halfSizeY, halfSizeZ), vec3(0.0, 0.0, 1.0), vec2(0.0, 1.0) },
        // top
        { vec3(-halfSizeX, halfSizeY, halfSizeZ), vec3(0.0, 1.0, 0.0), vec2(0.0, 0.0) },
        { vec3(halfSizeX, halfSizeY, halfSizeZ), vec3(0.0, 1.0, 0.0), vec2(1.0, 0.0) },
        { vec3(halfSizeX, halfSizeY, -halfSizeZ), vec3(0.0, 1.0, 0.0), vec2(1.0, 1.0) },
        { vec3(-halfSizeX, halfSizeY, -halfSizeZ), vec3(0.0, 1.0, 0.0), vec2(0.0, 1.0) },
        // back
        { vec3(halfSizeX, -halfSizeY, -halfSizeZ), vec3(0.0, 0.0, -1.0), vec2(0.0, 0.0) },
        { vec3(-halfSizeX, -halfSizeY, -halfSizeZ), vec3(0.0, 0.0, -1.0), vec2(1.0, 0.0) },
        { vec3(-halfSizeX, halfSizeY, -halfSizeZ), vec3(0.0, 0.0, -1.0), vec2(1.0, 1.0) },
        { vec3(halfSizeX, halfSizeY, -halfSizeZ), vec3(0.0, 0.0, -1.0), vec2(0.0, 1.0) },
        // bottom
        { vec3(-halfSizeX, -halfSizeY, -halfSizeZ), vec3(0.0, -1.0, 0.0), vec2(0.0, 0.0) },
        { vec3(halfSizeX, -halfSizeY, -halfSizeZ), vec3(0.0, -1.0, 0.0), vec2(1.0, 0.0) },
        { vec3(halfSizeX, -halfSizeY, halfSizeZ), vec3(0.0, -1.0, 0.0), vec2(1.0, 1.0) },
        { vec3(-halfSizeX, -halfSizeY, halfSizeZ), vec3(0.0, -1.0, 0.0), vec2(0.0, 1.0) },
        // left
        { vec3(-halfSizeX, -halfSizeY, -halfSizeZ), vec3(-1.0, 0.0, 0.0), vec2(0.0, 0.0) },
        { vec3(-halfSizeX, -halfSizeY, halfSizeZ), vec3(-1.0, 0.0, 0.0), vec2(1.0, 0.0) },
        { vec3(-halfSizeX, halfSizeY, halfSizeZ), vec3(-1.0, 0.0, 0.0), vec2(1.0, 1.0) },
        { vec3(-halfSizeX, halfSizeY, -halfSizeZ), vec3(-1.0, 0.0, 0.0), vec2(0.0, 1.0) },
        // right
        { vec3(halfSizeX, -halfSizeY, halfSizeZ), vec3(1.0, 0.0, 0.0), vec2(0.0, 0.0) },
        { vec3(halfSizeX, -halfSizeY, -halfSizeZ), vec3(1.0, 0.0, 0.0), vec2(1.0, 0.0) },
        { vec3(halfSizeX, halfSizeY, -halfSizeZ), vec3(1.0, 0.0, 0.0), vec2(1.0, 1.0) },
        { vec3(halfSizeX, halfSizeY, halfSizeZ), vec3(1.0, 0.0, 0.0), vec2(0.0, 1.0) },
    };

    if (offset != glm::vec3(0.0f))
        for (unsigned i = 0; i < vertices.size(); ++i)
            vertices[i].position += offset;

    std::vector<GLuint> indices = {
        0,  1,  2,  0,  2,  3,
        4,  5,  6,  4,  6,  7,
        8,  9,  10, 8,  10, 11,
        12, 13, 14, 12, 14, 15,
        16, 17, 18, 16, 18, 19,
        20, 21, 22, 20, 22, 23,
    };

    Build(vertices, indices);
}

