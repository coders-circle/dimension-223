#include <stdinc.h>
#include <Application.h>

int main(int argc, char* argv[])
{
    try
    {
        // Run the application
        Application application(argc, argv);
        application.Run();
        return application.GetReturnCode();
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
}
