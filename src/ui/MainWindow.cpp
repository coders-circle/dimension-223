#include <stdinc.h>
#include <ui/MainWindow.h>


const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

MainWindow::MainWindow() :
    mViewport(WINDOW_WIDTH, WINDOW_HEIGHT)
{
    mWindow.set_default_size(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Pass viewport event handlers to the renderer.
    mViewport.setResizeCallback([this](int w, int h) {
        mRenderer.resize(w, h);
    });
    mViewport.setInitCallback([this]() {
        mRenderer.init();
    });
    mViewport.setDrawCallback([this]() {
        mRenderer.draw();
        mViewport.invalidate();
    });

    // Show and add the viewport to the window.
    mViewport.show();
    mWindow.add(mViewport);

    glewInit();

    mWindow.maximize();
}
