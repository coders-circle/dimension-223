#pragma once
#include <graphics/Material.h>
#include <graphics/Texture.h>

inline std::vector<Shader*> GetDepthShaders()
{
    static Shader depthVSShader("shaders/vs_depth.glsl", GL_VERTEX_SHADER);
    static Shader depthFSShader("shaders/fs_depth.glsl", GL_FRAGMENT_SHADER);
    return std::vector<Shader*> {
        &depthVSShader, &depthFSShader
    };
}

/**
 * Material to render meshes with depth map and parallax effect.
 */
class DepthMaterial : public Material
{
public:
    /**
     * Construct new DepthMaterial.
     */
    DepthMaterial()
        : Material(GetDepthShaders())
    {
    }

    /// Set values of the uniforms of the GLSL program.
    void SetUniforms()
    {
    }
};
