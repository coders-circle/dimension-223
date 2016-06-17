#pragma once
#include <graphics/Shader.h>


/**
 * OpenGL shader program linked from a vertex and a fragment shader.
 */
class Program {
public:
    Program(const Shader& vertexShader, const Shader& fragmentShader) {
        GLuint program = glCreateProgram();
        glAttachShader(program, vertexShader.getId());
        glAttachShader(program, fragmentShader.getId());
        glLinkProgram(program);

        GLint success;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (!success) {
            GLchar infoLog[512];
            glGetProgramInfoLog(program, 512, 0, infoLog);

            std::stringstream msg;
            msg << "Linking error: \t" << infoLog << std::endl;

            throw Exception(msg.str());
        }
        mProgramId = program;
    }

    ~Program() {
        glDeleteProgram(mProgramId);
    }

    GLint getUniformLocation(const std::string& name) const {
        return glGetUniformLocation(mProgramId, name.c_str());
    }

    void use() const {
        glUseProgram(mProgramId);
    }

private:
    GLuint mProgramId;

};
