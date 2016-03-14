#pragma once
#include <image/Image.h>

/**
 * Disparity Map generator that for any two given images
 * calculate disparity for each point based on least SAD or SSD values
 * and generate a map of disparities.
 *
 * The input images are RGBA images and the output image is a grayscale
 * image, where the gray intensity gives the total disparity as normalized
 * value. To get actual disparity in terms of pixels, this disparity should
 * be multiplied by the maximum disparity.
 */
class DisparityMap
{
public:
    /**
     * Construct the generator with two input images.
     * @param image1 First input image.
     * @param image2 Second input image.
     * @param dMax Maximum disparity in pixels. This value is used to keep the
     *             disparity calculation iteration in check and to normalize
     *             the obtained disparity.
     */
    DisparityMap(Image<glm::vec4>& image1, Image<glm::vec4>& image2, int dMax);

    /**
     * Generate a disparity map out of given input images.
     * @return A grayscale image whose any pixel represents the disparity of
     *         corresponding pixels in two input images.
     */
    Image<float> Generate();

private:
    // Compute Sum of Absolute Differences.
    float ComputeSAD(int x, int y, int d);
    // Compute Sum of Squared Differences.
    float ComputeSSD(int x, int y, int d);
    // Get disparity for image pixel at (x, y).
    float GetDisparity(int x, int y);

    Image<glm::vec4>& m_image1;
    Image<glm::vec4>& m_image2;
    int m_dMax;

};
