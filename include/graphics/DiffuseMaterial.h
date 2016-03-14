#pragma once
#include <graphics/Material.h>
#include <graphics/Texture.h>

inline std::vector<Shader*> GetDiffuseShaders()
{
    static Shader diffuseVSShader("shaders/vs_diffuse.glsl", GL_VERTEX_SHADER);
    static Shader diffuseFSShader("shaders/fs_diffuse.glsl", GL_FRAGMENT_SHADER);
    return std::vector<Shader*> {
        &diffuseVSShader, &diffuseFSShader
    };
}

/**
 * Material to render meshes with diffuse shading.
 *
 * It stores a color and a texture to render the mesh with diffuse lighting.
 */
class DiffuseMaterial : public Material
{
public:
    /**
     * Construct new DiffuseMaterial.
     * @param color RGBA diffuse color to blend with the texture.
     * @param texture Texture object to use with the mesh.
     */
    DiffuseMaterial(const glm::vec4& color=glm::vec4(1),
        Texture* texture=0)
        : Material(GetDiffuseShaders()), color(color),
          texture(texture)
    {
        m_program.AddUniform("diffuseColor");
        m_program.AddUniform("diffuseTexture");
    }

    /// Diffuse color in RGBA format.
    glm::vec4 color;
    /// Diffuse texture to wrap around the mesh.
    Texture* texture;

    /// Set values of the uniforms of the GLSL program.
    void SetUniforms()
    {
        // TODO: Set to white texture by default

        m_program.SetUniform("diffuseColor", color);
        glActiveTexture(GL_TEXTURE0);
        if (texture)
            glBindTexture(GL_TEXTURE_2D, texture->GetTexture());
        m_program.SetUniform("diffuseTexture", (int)0);
    }
};
