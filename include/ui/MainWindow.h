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
    MainWindow(Project& project);

private:
    Project& mProject;

    SFMLWidget mViewport;
    Renderer mRenderer;

    void addMenuItems(Gtk::MenuBar* menuBar);
    void addToolItems(Gtk::Toolbar* toolbar);

    Gtk::SpinButton* mTranslations[3], *mScales[3], *mRotations[3];

    size_t mSelection;
    void changeSelection(size_t selection);
    void updateSelection();
    bool mChanging;

    void addModel();
    void addPointCloud();
    void loadProject();
    void saveProject();

    typedef std::pair<std::string, std::string> filter;

    std::string openFileDialog(
        const std::string& prompt = "Open",
        const std::vector<filter> filters = std::vector<filter>()
    );

    std::string saveFileDialog(
        const std::string& prompt = "Save",
        const std::vector<filter> filters = std::vector<filter>()
    );
};
