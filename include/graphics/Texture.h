#pragma once
#include <SOIL/SOIL.h>


/**
 * OpenGL texture that can be used by meshes and point clouds.
 */
class Texture {
public:
    Texture() : mTextureId(getWhiteTexture()) {}

    static GLuint getWhiteTexture() {
        static GLuint texId = 0;
        if (texId == 0) {
            glGenTextures(1, &texId);
            glBindTexture(GL_TEXTURE_2D, texId);
            std::vector<unsigned char> wh;
            for (int i=0; i<4*4; ++i)
                wh.push_back(0xFF);
            createTexture(&wh[0], 1, 1);
        }
        return texId;
    }

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
        if (mTextureId != getWhiteTexture())
            glDeleteTextures(1, &mTextureId);
    }

private:
    GLuint mTextureId;

    void load(const std::string& filename) {
        int w, h;
        unsigned char* image = SOIL_load_image(filename.c_str(), &w, &h,
            0, SOIL_LOAD_RGB);

        if (!image) {
            throw Exception(std::string("Couldn't load image: ")
                + filename + "\n"
                + SOIL_last_result() + "\n");
        }

        glGenTextures(1, &mTextureId);
        glBindTexture(GL_TEXTURE_2D, mTextureId);
        createTexture(image, w, h);
        SOIL_free_image_data(image);
    }

    static void createTexture(unsigned char* image, int w, int h) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB,
            GL_UNSIGNED_BYTE, image);
        glGenerateMipmap(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};
