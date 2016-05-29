#pragma once
#include <gtkmm.h>
#include <ui/SFMLWidget.h>
#include <graphics/Renderer.h>

/**
 * The main Gtk+ window. This handles creation of all widgets and toolbars
 * inside the window.
 */
class MainWindow {
public:
    MainWindow();

    Gtk::Window& getGtkWindow() {
        return mWindow;
    }

private:
    Gtk::Window mWindow;
    SFMLWidget mViewport;
    Renderer mRenderer;

};
