#pragma once
#include <shaders/Program.h>

/**
 * Abstract base class for materials.
 *
 * This class handles GLSL program and its uniforms through object mapping.
 */
class Material
{
public:
    /**
     * Construct Material from shaders.
     *
     * The shaders are linked to a program which is stored internally by this material.
     * @param shaders List of shaders to link.
     * @param useStdTransforms Whether or not the shaders contain the standard transformation uniform variables.
     */
    Material(const std::vector<Shader*>& shaders,
        bool useStdTransforms = true)
        : m_program(shaders, useStdTransforms)
    {}

    virtual ~Material() {};

    /// Use the material by setting the program as current OpenGL program and
    /// setting the uniform values to material current values.
    virtual void Use()
    {
        m_program.Use();
        SetUniforms();
    }

    virtual void SetUniforms() = 0;

    /**
     * Get program that this material uses.
     * @return Program object that this material uses.
     */
    Program& GetProgram() { return m_program; }

protected:
    Program m_program;
};
