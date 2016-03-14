#include <stdinc.h>
#include <image/DisparityMap.h>


DisparityMap::DisparityMap(Image<glm::vec4>& image1, Image<glm::vec4>& image2,
        int dMax)
    : m_image1(image1), m_image2(image2), m_dMax(dMax)
{
}


Image<float> DisparityMap::Generate()
{
    int width = m_image1.GetWidth();
    int height = m_image1.GetHeight();
    Image<float> disparityMap(width, height, 0.0f);

    for (int x=0; x<width; ++x)
        for (int y=0; y<height; ++y)
            disparityMap.Set(x, y, GetDisparity(x, y));

    return disparityMap;
}


float DisparityMap::ComputeSAD(int x, int y, int d)
{
    //TODO
    return 0;
}


float DisparityMap::ComputeSSD(int x, int y, int d)
{
    //TODO
    return 0;
}


float DisparityMap::GetDisparity(int x, int y)
{
    float minSad = 9999999;
    int disparity = 0;

    for (int d=1; d<=m_dMax; ++d)
    {
        float sad = ComputeSAD(x, y, d);
        if (sad < minSad)
        {
            minSad = sad;
            disparity = d;
        }
    }

    // Return normalized value.
    return static_cast<float>(disparity) /  m_dMax;
}
