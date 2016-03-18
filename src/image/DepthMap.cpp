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

    std::cout << "Generating depth map:\n0%\t";
    for (int x=0; x<width; ++x)
    {
        std::cout << "\r" << int(x/(float)width*100) << "%\t";
        std::cout.flush();
        for (int y=0; y<height; ++y)
            depthMap.Set(x, y, GetDepth(x, y));
    }

    std::cout << "\r100%\tDone" << std::endl;

    return depthMap;
}


float DepthMap::GetDepth(int x, int y)
{
    float disparity = m_disparityMap.Get(x, y);
    // return m_focalLength * m_camDistance / disparity;
    return disparity;
}
