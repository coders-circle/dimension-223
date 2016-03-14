#pragma once
#include <graphics/Shader.h>


/// Exception representing program linking error.
class ProgramError : public Exception
{
public:
    /// Program linkage error.
    ProgramError(const std::string& error)
        : Exception("\tError while linking shaders"
            + std::string("\n\tError:\n\t\t") + error)
        {}
};


/**
 * Class to handle linkage of shaders and to store their uniforms.
 */
class Program
{
public:
    /**
     * Link shaders to a program.
     * @param shaders List of shaders to link.
     * @param useStdTransforms Whether or not the shaders contain the standard transformation uniform variables.
     */
    Program(const std::vector<Shader*>& shaders,
        bool useStdTransforms=true);

    /// Destroy the OpenGL program object.
    ~Program()
    {
        glDeleteProgram(m_program);
    }

    /**
     * Get the represented OpenGL program object.
     * @return Id of the OpenGL program.
     */
    GLuint GetProgramObject() const { return m_program; }

    /**
     * Add uniform from the shaders.
     *
     * Effectively, this gets and save the uniform location for later purposes.
     * @param uniform_name Name of the uniform variable.
     * @return The uniform location.
     */
    GLint AddUniform(const std::string& uniform_name)
    {
        GLint uniform = glGetUniformLocation(m_program,
            uniform_name.c_str());
        m_uniforms[uniform_name] = uniform;
        return uniform;
    }

    /**
     * Get previously added uniform.
     * @param uniform_name Name of the uniform variable.
     * @return The uniform location.
     */
    GLint GetUniform(const std::string& uniform_name)
    {
        return m_uniforms[uniform_name];
    }


    /// Set uniform value.
    void SetUniform(const std::string& uniform, bool boolean)
    { glUniform1i(m_uniforms[uniform], boolean); }
    /// Set uniform value.
    void SetUniform(const std::string& uniform, GLint integer)
    { glUniform1i(m_uniforms[uniform], integer); }
    /// Set uniform value.
    void SetUniform(const std::string& uniform, GLfloat number)
    { glUniform1f(m_uniforms[uniform], number); }
    /// Set uniform value.
    void SetUniform(const std::string& uniform, const glm::mat4& matrix)
    { glUniformMatrix4fv(m_uniforms[uniform], 1, GL_FALSE, glm::value_ptr(matrix)); }
    /// Set uniform value.
    void SetUniform(const std::string& uniform, const glm::vec4& vector)
    { glUniform4fv(m_uniforms[uniform], 1, glm::value_ptr(vector)); }
    /// Set uniform value.
    void SetUniform(const std::string& uniform, const glm::vec3& vector)
    { glUniform3fv(m_uniforms[uniform], 1, glm::value_ptr(vector)); }


    /// Use the OpenGL program object.
    void Use() const
    {
        glUseProgram(m_program);
    }

    /**
     * Get whether or not the program uses standard transformation uniform variables.
     * @return True only if the program uses standard transformation uniforms.
     */
    bool UsesStdTransforms() const { return m_useStdTransforms; }

private:
    // The GLSL program object
    GLuint m_program;

    // The uniforms
    std::map<std::string, GLint> m_uniforms;

    // Uses standard transformation uniform matrices
    bool m_useStdTransforms;

    void AddStdTransforms()
    {
        AddUniform("mvpMatrix");
        AddUniform("modelMatrix");
    }


};
