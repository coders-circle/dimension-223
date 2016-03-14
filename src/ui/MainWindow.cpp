#include <stdinc.h>
#include <ui/MainWindow.h>


MainWindow::MainWindow(int width, int height)
    : m_renderer(m_window, width, height)
{
}
