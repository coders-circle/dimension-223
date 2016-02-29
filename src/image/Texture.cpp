#include <stdinc.h>
#include <image/Texture.h>


Texture::Texture(const Image<>& image)
{
    m_width = image.GetWidth();
    m_height = image.GetHeight();

    // Create texture object
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);

    // Set texture data to image
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, m_width, m_height,
        0, GL_RGBA, GL_FLOAT, image.GetPixels());
    glGenerateMipmap(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, 0);
}
