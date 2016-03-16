#include <stdinc.h>
#include <Application.h>


Application::Application(int argc, char* argv[])
    : m_gtkMain(argc, argv),
      m_mainWindow(800, 600)
{}


void Application::Run()
{
    m_mainWindow.Run();
}
