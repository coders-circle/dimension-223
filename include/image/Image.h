#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
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
        int flags = CV_LOAD_IMAGE_COLOR;
        cv::Mat image = cv::imread(filename, flags);
        Construct(image);
    }

    /**
     * Load image from cv::Mat object.
     * @param image OpenCV image to use to create this Image object.
     */
    Image(const cv::Mat& image)
    {
        Construct(image);
    }

    /**
     * Create image of given size, all pixels filled with given value.
     * @param width Width of the image.
     * @param height Height of the image.
     * @param value Initial intensity for all pixels.
     */
    Image(int width, int height, const T& value)
    {
        m_width = width;
        m_height = height;
        m_pixels.resize(width*height, ColorToVec(value));
    }

    /**
     * Save image to a file.
     * @param filename Name of file to save image to.
     */
    void Save(const std::string& filename)
    {
        // CreateMagickImage().write(filename);
    }

    /**
     * Get color of a pixel.
     * @param x Column index of image pixel.
     * @param y Row index of image pixel.
     * @return Color of the pixel returned as type T.
     */
    T Get(int x, int y) const
    { return VecToColor<T>(m_pixels[y*m_width + x]); }

    /**
     * Set color of a pixel.
     * @param x Column index of image pixel.
     * @param y Row index of image pixel.
     * @param color Color of the pixel provided as type T.
     */
    void Set(int x, int y, const T& color)
    { m_pixels[y*m_width + x] = ColorToVec<T>(color); }

    /**
     * Get array of pixels each of RGBA format.
     * @return Pointer to interal representation of pixels in RGBA format returned as pointer to float.
     */
    const float* GetPixels() const
    { return &m_pixels[0][0]; }

    /**
     * Create cv::Mat image from this image object.
     * @return New cv::Mat object created from stored pixels.
     */
    cv::Mat CreateCVImage()
    {
        cv::Mat image(m_height, m_width, CV_32FC3);
        cv::Mat source(image.size(), CV_32FC4, &m_pixels[0][0]);
        cv::cvtColor(source, image, CV_RGBA2BGRA, 4);
        return image;
    }

    /**
     * Display the image using OpenCV.
     */
    void Show()
    {
        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display window", CreateCVImage());
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

    void Construct(const cv::Mat& image)
    {
        m_width = image.cols;
        m_height = image.rows;

        // Create copy of image in floating point BGR space.
        cv::Mat copy;

        if (IsGrayscale<T>())
        {
            // Grayscale image is stored as RGBA image but with same RGB values.
            // Single channel is not used since RGBA texture is used by OpenGL.
            cv::Mat gray, rgb;
            cv::cvtColor(image, gray, CV_BGR2GRAY);
            cv::cvtColor(gray, rgb, CV_GRAY2BGR);
            rgb.convertTo(copy, CV_32FC3);
        }
        else
            image.convertTo(copy, CV_32FC3);

        copy /= 255.0f;

        // Get the pixels.
        m_pixels.resize(m_width*m_height);
        cv::Mat destination(copy.size(), CV_32FC4, &m_pixels[0][0]);
        cv::cvtColor(copy, destination, CV_BGR2RGBA, 4);
    }
};
