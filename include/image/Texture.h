#pragma once
#include <image/Image.h>


/**
 * Class representing OpenGL texture object.
 *
 * Texture is wrapper for OpenGL texture object. It stores the texture-id
 * and size of the texture and can be constructed by directly loading the image
 * from file and through an Image object.
 */
class Texture
{
public:
    /**
     * Construct texture from an image file.
     * @param path  Image file path to load image from.
     */
    Texture(const std::string& path)
        : Texture(Image<>(path)) {}

    /**
     * Construct texture from an Image object.
     * @param image Image object to create Texture from.
     */
    Texture(const Image<>& image);

    /// Delete the internally stored OpenGL texture object.
    ~Texture()
    {
        glDeleteTextures(1, &m_texture);
    }

    /**
     * Get width of the texture image.
     * @return  Width of the texture in pixels.
     */
    int GetWidth() const { return m_width; }
    /**
     * Get height of the texture image.
     * @return  Height of the texture in pixels.
     */
    int GetHeight() const { return m_height; }

    /**
     * Get OpenGL texture object reprsented by this object.
     * @return  OpenGL texture id.
     */
    GLuint GetTexture() const { return m_texture; }

private:
    int m_width, m_height;
    GLuint m_texture;
};
