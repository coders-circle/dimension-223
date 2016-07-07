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

    Gtk::MenuItem* createMenuItem(const std::string& itemName,
                                  std::function<void()> handler = nullptr)
    {
        Gtk::MenuItem* item = Gtk::manage(new Gtk::MenuItem(itemName, true));
        if (handler) {
            item->signal_activate().connect(handler);
        }
        return item;
    }

    void addModel();
    void addPointCloud();
    void loadProject();
    void saveProject();

    typedef std::pair<std::string, std::string> filter;

    std::string openFileDialog(
        const std::string& prompt = "Open",
        const std::vector<filter> filters = std::vector<filter>()
    ) {
        Gtk::FileChooserDialog dialog(prompt, Gtk::FILE_CHOOSER_ACTION_OPEN);
        dialog.set_transient_for(*this);

        // Add response buttons the the dialog.
        dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
        dialog.add_button("_Open", Gtk::RESPONSE_OK);

        // Add filters.
        if (filters.size() == 0) {
            Glib::RefPtr<Gtk::FileFilter> filter_any =
                Gtk::FileFilter::create();
            filter_any->set_name("Any files");
            filter_any->add_pattern("*");
            dialog.add_filter(filter_any);
        }
        else {
            // TODO
        }

        int result = dialog.run();

        switch(result) {
        case(Gtk::RESPONSE_OK):
            return dialog.get_filename();
        case(Gtk::RESPONSE_CANCEL):
        default:
            return "";

        }
    }

    std::string saveFileDialog(
        const std::string& prompt = "Save",
        const std::vector<filter> filters = std::vector<filter>()
    ) {
        Gtk::FileChooserDialog dialog(prompt, Gtk::FILE_CHOOSER_ACTION_SAVE);
        dialog.set_transient_for(*this);

        // Add response buttons the the dialog.
        dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
        dialog.add_button("_Save", Gtk::RESPONSE_OK);

        // Add filters.
        if (filters.size() == 0) {
            Glib::RefPtr<Gtk::FileFilter> filter_any =
                Gtk::FileFilter::create();
            filter_any->set_name("Any files");
            filter_any->add_pattern("*");
            dialog.add_filter(filter_any);
        }
        else {
            // TODO
        }

        int result = dialog.run();

        switch(result) {
        case(Gtk::RESPONSE_OK):
            return dialog.get_filename();
        case(Gtk::RESPONSE_CANCEL):
        default:
            return "";

        }
    }
};
