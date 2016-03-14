#pragma once
#include <gtkmm.h>
#include <ui/Renderer.h>


/**
 * Main GTK+ window which contains all the user interface.
 */
class MainWindow
{
public:
    /**
     * Construct the window.
     * @param width Initial width of the window.
     * @param height Initial height of the window.
     */
    MainWindow(int width, int height);

    /**
     * Display the window and starts its main loop.
     * @param application Application that contains this window.
     * @return Value returned by the application that runs this window.
     */
    int Run(Glib::RefPtr<Gtk::Application> application)
    {
        return application->run(m_window);
    }

private:
    Gtk::Window m_window;
    Renderer m_renderer;
};
