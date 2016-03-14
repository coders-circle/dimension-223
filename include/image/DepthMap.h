#pragma once
#include <image/Image.h>

/**
 * Depth Map generator that for a given disparity map generates
 * the depth values based on disparity.
 *
 * The inputs are a grayscale disparity map, camera distance for two images
 * and focal length of the camera. The output is a depth map which is again
 * a grayscale image, whose intensities correspond to depth values.
 */
class DepthMap
{
public:
    /**
     * Construct the generator for given disparity map.
     * @param disparityMap Grayscale image whose pixels represent disparities.
     * @param camDistance Distance of camera positions between two images.
     * @param focalLength Focal length of the camera.
     */
    DepthMap(Image<float>& disparityMap, float camDistance, float focalLength);

    /**
     * Generate the depth map.
     * @return A grayscale image whose any pixel represent the depth value
     *         for that point in original image.
     */
    Image<float> Generate();

private:
    // Calculate depth value for a pixel.
    float GetDepth(int x, int y);

    Image<float>& m_disparityMap;
    float m_camDistance;
    float m_focalLength;
};
