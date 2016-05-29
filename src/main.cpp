#include <stdinc.h>
#include <Application.h>


int main(int argc, char* argv[]) {
    try {
        Application application(argc, argv);
        return application.run();
    }
    catch (const std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}
