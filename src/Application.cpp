#include <stdinc.h>
#include <Application.h>


Application::Application(int argc, char* argv[])
    : m_app(Gtk::Application::create(argc, argv,
        "org.toggle.dimension")),
      m_mainWindow(800, 600)
{}


void Application::Run()
{
    m_returnCode = m_mainWindow.Run(m_app);
}
