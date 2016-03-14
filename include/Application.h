#pragma once
#include <Project.h>
#include <ui/MainWindow.h>


/**
 * Main application that manages objects of every other class
 * and acts as central coordinator between them.
 */
class Application
{
public:
    /**
     * Construct the application along with the main window that acts
     * as its GUI front end and the image processors and generators that act
     * its back end.
     * @param argc Command line arguments count.
     * @param argv Comman line arguments array.
     */
    Application(int argc, char* argv[]);

    /**
     * Run the application, starting by opening the main window.
     */
    void Run();

    /**
     * Get code returned by the application on exiting.
     * @return The return code.
     */
    int GetReturnCode() const { return m_returnCode; }

private:
    Glib::RefPtr<Gtk::Application> m_app;
    MainWindow m_mainWindow;
    int m_returnCode;
};
