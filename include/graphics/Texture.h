#pragma once
#include <SOIL/SOIL.h>


/**
 * OpenGL texture that can be used by meshes and point clouds.
 */
class Texture {
public:
    Texture() : mTextureId(0) {}

    Texture(const std::string& filename) {
        load(filename);
    }

    Texture(const cv::Mat& image) {
        cv::imwrite("tmp.png", image);
        load("tmp.png");

    }

    void bind() const {
        glBindTexture(GL_TEXTURE_2D, mTextureId);
    }

    void destroy() {
        glDeleteTextures(1, &mTextureId);
    }

private:
    GLuint mTextureId;

    void load(const std::string& filename) {
        glGenTextures(1, &mTextureId);
        glBindTexture(GL_TEXTURE_2D, mTextureId);

        int w, h;
        unsigned char* image = SOIL_load_image(filename.c_str(), &w, &h,
            0, SOIL_LOAD_RGB);

        if (!image) {
            throw Exception(std::string("Couldn't load image.\n")
                +SOIL_last_result()+"\n");
        }

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
            GL_UNSIGNED_BYTE, image);
        glGenerateMipmap(GL_TEXTURE_2D);
        SOIL_free_image_data(image);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};
