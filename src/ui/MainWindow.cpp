#include <stdinc.h>
#include <ui/MainWindow.h>


const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

MainWindow::MainWindow() :
    mViewport(WINDOW_WIDTH, WINDOW_HEIGHT)
{
    set_default_size(WINDOW_WIDTH, WINDOW_HEIGHT);

    // Global box to contain all widgets.
    Gtk::Box* vbox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_VERTICAL, 0));
    add(*vbox);

    // Menubar.
    Gtk::MenuBar* menuBar = Gtk::manage(new Gtk::MenuBar());
    vbox->pack_start(*menuBar, Gtk::PACK_SHRINK, 0);
    addMenuItems(menuBar);

    // The horizontal box with toolbar and viewport.
    Gtk::Box* hbox = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 0));
    hbox->set_border_width(5);
    vbox->add(*hbox);

    Gtk::Toolbar* toolbar = Gtk::manage(new Gtk::Toolbar());
    toolbar->set_property("orientation", Gtk::ORIENTATION_VERTICAL);
    toolbar->set_property("toolbar-style", Gtk::TOOLBAR_ICONS);
    addToolItems(toolbar);
    // hbox->pack_start(*toolbar, Gtk::PACK_SHRINK, 0); // TODO: Show toolbar.

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
    // mViewport.set_border_width(15);
    mViewport.set_hexpand(true);
    mViewport.set_vexpand(true);
    hbox->add(mViewport);

    vbox->show_all();

    glewInit();

    maximize();
}


void MainWindow::addMenuItems(Gtk::MenuBar* menuBar) {

    // File Menu.

    Gtk::MenuItem* menuitem_file = createMenuItem("_File");
    menuBar->append(*menuitem_file);
    Gtk::Menu* filemenu = Gtk::manage(new Gtk::Menu());
    menuitem_file->set_submenu(*filemenu);

    filemenu->append(*createMenuItem("_Quit", [this]() {
        hide();
    }));

    // Edit Menu.

    Gtk::MenuItem* menuitem_edit = createMenuItem("_Edit");
    menuBar->append(*menuitem_edit);
    Gtk::Menu* editmenu = Gtk::manage(new Gtk::Menu());
    menuitem_edit->set_submenu(*editmenu);
}


void MainWindow::addToolItems(Gtk::Toolbar* toolbar) {

}
