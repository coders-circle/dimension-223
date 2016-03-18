#include <stdinc.h>
#include <Application.h>
#include <image/DisparityMap.h>
#include <image/DepthMap.h>

int main(int argc, char* argv[])
{
    try
    {
        // Image<> left("images/left.jpg");
        // Image<> right("images/right.jpg");
        //
        // DisparityMap dispGen(left, right, 8, 20);
        // Image<float> disparityMap = dispGen.Generate();
        //
        // DepthMap depthGen(disparityMap, 1, 1);
        // Image<float> depthMap = depthGen.Generate();
        // depthMap.Show();
        //
        // cv::waitKey(0);

        // Run the application
        Application application(argc, argv);
        application.Run();
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
}
