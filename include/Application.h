#pragma once
#include <ui/MainWindow.h>


/**
 * Central application handling the coordination between all other major
 * objects including the main window, the renderer and the project.
 */
class Application {
public:
    Application(int argc, char* argv[]) {
        mApp = Gtk::Application::create(argc, argv, "org.toggle.d223");
    }

    int run() {
        Project project;
        MainWindow window(project);
        return mApp->run(window);
    }

private:
    Glib::RefPtr<Gtk::Application> mApp;
};
