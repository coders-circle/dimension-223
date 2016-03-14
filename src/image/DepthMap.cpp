#include <stdinc.h>
#include <image/DepthMap.h>


DepthMap::DepthMap(Image<float>& disparityMap, float camDistance,
        float focalLength)
    : m_disparityMap(disparityMap), m_camDistance(camDistance),
      m_focalLength(focalLength)
{
}


Image<float> DepthMap::Generate()
{
    int width = m_disparityMap.GetWidth();
    int height = m_disparityMap.GetHeight();
    Image<float> depthMap(width, height, 0.0f);

    for (int x=0; x<width; ++x)
        for (int y=0; y<height; ++y)
            depthMap.Set(x, y, GetDepth(x, y));

    return depthMap;
}


float DepthMap::GetDepth(int x, int y)
{
    float disparity = m_disparityMap.Get(x, y);
    return m_focalLength * m_camDistance / disparity;
}
