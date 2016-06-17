#pragma once


/**
 * OpenGL vertex or fragment shader compiled from a GLSL script.
 */
class Shader {
public:
    Shader(const std::string& filename, GLenum type) {
        std::string str = readFile(filename);
        const char* text = str.c_str();

        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &text, 0);
        glCompileShader(shader);

        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);

        if (!success) {
            GLchar infoLog[512];
            glGetShaderInfoLog(shader, 512, 0, infoLog);

            std::stringstream msg;
            msg << "Shader compilation error: \t" << filename
                << "\n\n" << infoLog << std::endl;

            throw Exception(msg.str());
        }
        mShaderId = shader;
    }

    ~Shader() {
        glDeleteShader(mShaderId);
    }

    GLuint getId() const { return mShaderId; }

private:
    GLuint mShaderId;

};
