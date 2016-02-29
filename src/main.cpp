#include <stdinc.h>
#include <ui/MainWindow.h>
#include <image/Image.h>

void OnDraw();
void OnResize(int, int);


int main(int argc, char* argv[])
{
    try
    {
        Magick::InitializeMagick(*argv);

        // float == grayscale
        Image<float> image("images/test.jpg");

        // Create stripes in the middle portion
        int cw = image.GetWidth()/2;
        int ch = image.GetHeight()/2;

        for (int i=ch-20; i<ch+20; ++i)
            for (int j=cw-20; j<cw+20; ++j)
                // white == 1, black == 0
                image.Set(i, j, j%2==0);

        // Display the image
        image.CreateMagickImage().display();


        // Create the main window with Gtk

        auto app = Gtk::Application::create(argc, argv,
            "org.toggle.dimension");
        MainWindow window(800, 600);
        return window.Run(app);
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return -1;
    }
}
