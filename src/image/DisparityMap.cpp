#include <stdinc.h>
#include <image/DisparityMap.h>


DisparityMap::DisparityMap(Image<glm::vec4>& image1, Image<glm::vec4>& image2,
        int dMax, int windowSize)
    : m_image1(image1), m_image2(image2),
      m_dMax(dMax), m_windowSize(windowSize)
{
}


Image<float> DisparityMap::Generate()
{
    int width = m_image1.GetWidth();
    int height = m_image1.GetHeight();
    Image<float> disparityMap(width, height, 0.0f);

    std::cout << "Generating disparity map:\n0%\t";
    for (int x=0; x<width; ++x)
    {
        std::cout << "\r" << int(x/(float)width*100) << "%\t";
        std::cout.flush();
        for (int y=0; y<height; ++y) {
            disparityMap.Set(x, y, GetDisparity(x, y));
        }
    }

    std::cout << "\r100%\tDone !" << std::endl;

    return disparityMap;
}


float DisparityMap::ComputeSAD(int px, int py, int d)
{
    // Get window boundaries limited by size of first image.
    int leftX = m_image1.GetBoundaryX(px - m_windowSize/2);
    int rightX = m_image1.GetBoundaryX(px + m_windowSize/2);
    int topY = m_image1.GetBoundaryY(py - m_windowSize/2);
    int bottomY = m_image1.GetBoundaryY(py + m_windowSize/2);

    // Calculate the sum.
    float sad = 0;
    for (int x=leftX; x!=rightX; ++x)
    {
        for (int y=topY; y!=bottomY; ++y)
        {
            float xr = x - d;
            float yr = y;

            // Check boundary condition for second image.
            if (xr < 0 || xr >= m_image2.GetWidth())
                continue;
            if (yr < 0 || yr >= m_image2.GetHeight())
                continue;

            // Get absolute intensity differences.
            float diff = glm::abs(m_image1.GetIntensity(x, y) -
                m_image2.GetIntensity(xr, yr));

            sad += diff;
        }
    }

    return sad;
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

    for (int d=0; d<=m_dMax; ++d)
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
