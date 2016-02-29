#pragma once
#include <Magick++.h>
#include <image/utils.h>


/**
 * Class to store and access an array of pixels.
 *
 * Image<T> stores image pixels and size. Pixels are stored internally as
 * RGBA floating point values but can be accessed as type T.
 *
 * T can be glm::vec4 to access RGBA color and can be float to access only R.
 * Effectively, Image<glm::vec4> represents color image and Image<float>
 * represents grayscale image.
 */
template <class T=glm::vec4>
class Image
{
public:

    /**
     * Load image from a file.
     * @param filename Name of file to load image from.
     */
    Image(const std::string& filename)
    {
        // Read in the image file
        Magick::Image image;
        image.read(filename);
        Construct(image);
    }

    /**
     * Load image from Magick::Image object.
     * @param image ImageMagick image to use to create this Image object.
     */
    Image(Magick::Image image)
    {
        Construct(image);
    }

    /**
     * Save image to a file.
     * @param filename Name of file to save image to.
     */
    void Save(const std::string& filename)
    {
        CreateMagickImage().write(filename);
    }

    /**
     * Get color of a pixel.
     * @param  row    Row index of image pixel.
     * @param  column Column index of image pixel.
     * @return        Color of the pixel returned as type T.
     */
    T Get(int row, int column) const
    { return VecToColor<T>(m_pixels[row*m_width + column]); }

    /**
     * Set color of a pixel.
     * @param row    Row index of image pixel.
     * @param column Column index of image pixel.
     * @param color  Color of the pixel provided as type T.
     */
    void Set(int row, int column, const T& color)
    { m_pixels[row*m_width + column] = ColorToVec<T>(color); }

    /**
     * Get array of pixels each of RGBA format.
     * @return Pointer to interal representation of pixels in RGBA format returned as pointer to float.
     */
    const float* GetPixels() const
    { return &m_pixels[0][0]; }

    /**
     * Create ImageMagick image from this image object.
     * @return New ImageMagick::Image object created from stored pixels.
     */
    Magick::Image CreateMagickImage() const
    {
        Magick::Image image;
        image.read(m_width, m_height, "RGBA", Magick::FloatPixel,
            &m_pixels[0]);
        return image;
    }

    /**
     * Get width of image in pixels.
     * @return Width of the image in pixels.
     */
    int GetWidth() const { return m_width; }

    /**
     * Set height of image in pixels.
     * @return Height of the image in pixels.
     */
    int GetHeight() const { return m_height; }

private:
    std::vector<glm::vec4> m_pixels;
    int m_width, m_height;

    void Construct(Magick::Image image)
    {
        if (IsGrayscale<T>())
            image.type(Magick::GrayscaleType);

        m_width = image.columns();
        m_height = image.rows();

        // Get the pixels
        m_pixels.resize(m_width * m_height);
        image.write(0, 0, m_width, m_height, "RGBA",
            Magick::FloatPixel, &m_pixels[0]);
    }
};
