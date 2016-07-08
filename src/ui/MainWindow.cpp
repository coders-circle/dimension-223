#include <stdinc.h>
#include <ui/MainWindow.h>


const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

MainWindow::MainWindow(Project& project) :
    mProject(project),
    mViewport(WINDOW_WIDTH, WINDOW_HEIGHT),
    mRenderer(mViewport, project)
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
    Gtk::Menu* fileMenu = Gtk::manage(new Gtk::Menu());
    menuitem_file->set_submenu(*fileMenu);

    fileMenu->append(*createMenuItem("_Load Project", [this]() {
        loadProject();
    }));
    fileMenu->append(*createMenuItem("_Save Project", [this]() {
        saveProject();
    }));
    fileMenu->append(*createMenuItem("_Quit", [this]() {
        hide();
    }));

    // Edit Menu.

    Gtk::MenuItem* menuitem_edit = createMenuItem("_Edit");
    menuBar->append(*menuitem_edit);
    Gtk::Menu* editMenu = Gtk::manage(new Gtk::Menu());
    menuitem_edit->set_submenu(*editMenu);

    editMenu->append(*createMenuItem("_Add Model", [this]() {
        addModel();
    }));
    editMenu->append(*createMenuItem("_Add Lensblur Model", [this]() {
        addPointCloud();
    }));
}


void MainWindow::addToolItems(Gtk::Toolbar* toolbar) {

}


void MainWindow::addModel() {
    std::string path = openFileDialog("Load Model");
    if (path != "") {
        mProject.addModel(path);
    }
}

void MainWindow::addPointCloud() {
    std::string path = openFileDialog("Load Lens Blur Image");
    if (path != "") {
        mProject.addPointCloud(path);
    }
}


void MainWindow::loadProject() {
    mProject.clear();
    std::string path = saveFileDialog("Load project");
    if (path != "") {
        std::ifstream file(path);

        size_t numModels;
        file >> numModels;
        for (size_t i=0; i<numModels; ++i) {
            std::string path;
            std::getline(file, path);
            size_t index = mProject.addModel(path);

            float x, y, z, rx, ry, rz, rw;
            file >> x >> y >> z >> rx >> ry >> rz >> rw;
            Model& model = mProject.getModel(index);
            model.transformation.position = glm::vec3(x, y, z);
            model.transformation.rotation = glm::quat(rw, rx, ry, rz);
            model.transform();
        }

        size_t numPointclouds;
        file >> numPointclouds;
        for (size_t i=0; i<numPointclouds; ++i) {
            std::string path;
            std::getline(file, path);
            mProject.addPointCloud(path);
        }
    }
}

void MainWindow::saveProject() {
    std::string path = saveFileDialog("Save project");
    if (path != "") {
        std::ofstream file(path);
        file << mProject.getNumModels();
        for (size_t i=0; i<mProject.getNumModels(); ++i) {
            Model& model = mProject.getModel(i);
            file << model.getPath() << std::endl;
            file << model.transformation.position.x
                << " " << model.transformation.position.y
                << " " << model.transformation.position.z
                << " " << model.transformation.rotation.x
                << " " << model.transformation.rotation.y
                << " " << model.transformation.rotation.z
                << " " << model.transformation.rotation.w;
        }

        file << mProject.getNumPointClouds();
        for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
            file << mProject.getPointCloud(i).getPath() << std::endl;
        }
    }
}
