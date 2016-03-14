#pragma once


/// Exception representing shader compilation error.
class ShaderError : public Exception
{
public:
    /// Shader error produced by given file.
    ShaderError(const std::string& filename,
        const std::string& error)
        : Exception("\tError while compiling shader"
            + std::string("\n\tFilename: ") + filename
            + std::string("\n\tError:\n\t\t") + error)
        {}
};


/**
 * Class to handle compilation of shaders.
 */
class Shader
{
public:
    /**
     * Construct and compile OpenGL shader object.
     * @param filename File path for the shader source.
     * @param shaderType OpenGL shader type.
     */
    Shader(const std::string& filename, GLenum shaderType);

    /// Destroy the OpenGL shader object.
    ~Shader()
    {
        glDeleteShader(m_shader);
    }

    /**
     * Get OpenGL shader id represented by this object.
     * @return Id of the OpenGL shader object.
     */
    GLuint GetShaderObject() const { return m_shader; }

    /**
     * Attach the shader to a GLSL program.
     * @param program Id of the OpenGL program object.
     */
    void AttachTo(GLuint program) const
    {
        glAttachShader(program, m_shader);
    }

    /**
     * Detach the shader from a GLSL program it was previously attached with.
     * @param program Id of the OpenGL program object.
     */
    void DetachFrom(GLuint program) const
    {
        glDetachShader(program, m_shader);
    }


private:
    // The shader object.
    GLuint m_shader;
};
