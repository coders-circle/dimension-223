#pragma once
#include <gtkmm.h>
#include <ui/SFMLWidget.h>
#include <graphics/Renderer.h>

/**
 * The main Gtk+ window. This handles creation of all widgets and toolbars
 * inside the window.
 */
class MainWindow : public Gtk::Window {
public:
    MainWindow();

private:
    SFMLWidget mViewport;
    Renderer mRenderer;

    void addMenuItems(Gtk::MenuBar* menuBar);
    void addToolItems(Gtk::Toolbar* toolbar);

    Gtk::MenuItem* createMenuItem(const std::string& itemName,
                                  std::function<void()> handler = nullptr) {
        Gtk::MenuItem* item = Gtk::manage(new Gtk::MenuItem(itemName, true));
        if (handler) {
            item->signal_activate().connect(handler);
        }
        return item;
    }
};
