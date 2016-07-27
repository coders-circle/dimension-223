#include <stdinc.h>
#include <ui/MainWindow.h>
#include <ui/widget_creator.h>
#include <ui/SurfaceEditor.h>
#include <ui/CloudPairEditor.h>
#include <ui/InputDialog.h>


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

    // Toolbar.
    Gtk::Toolbar* toolbar = Gtk::manage(new Gtk::Toolbar());
    toolbar->set_property("orientation", Gtk::ORIENTATION_HORIZONTAL);
    toolbar->set_property("toolbar-style", Gtk::TOOLBAR_ICONS);
    addToolItems(toolbar);

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
    mRenderer.setSelectionChangedCallback([this](size_t selection)
    {
        changeSelection(selection);
    });

    // Show and add the viewport to the window.
    mViewport.show();
    mViewport.set_hexpand(true);
    mViewport.set_vexpand(true);
    vbox->add(mViewport);
    vbox->add(*toolbar);

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

    fileMenu->append(*createMenuItem("_New Project", [this]() {
        mProject.clear();
    }, Accelerator('N', Gdk::ModifierType::CONTROL_MASK, get_accel_group())));

    fileMenu->append(*createMenuItem("_Load Project", [this]() {
        loadProject();
    }, Accelerator('O', Gdk::ModifierType::CONTROL_MASK, get_accel_group())));

    fileMenu->append(*createMenuItem("_Save Project", [this]() {
        saveProject();
    }, Accelerator('S', Gdk::ModifierType::CONTROL_MASK, get_accel_group())));

    fileMenu->append(*createMenuItem("_Quit", [this]() {
        hide();
    }, Accelerator('Q', Gdk::ModifierType::CONTROL_MASK, get_accel_group())));

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
    // Gtk::ToolButton *TB1 = new Gtk::ToolButton(Gtk::Stock::GO_BACK);
    // toolbar->append(*TB1);

    toolbar->append(*createToolItem(
        Gtk::manage(new Gtk::Label("Translation: ")))
    );
    for (int i=0; i<3; ++i) {
        mTranslations[i] = createSpinEntry();
        mTranslations[i]->signal_value_changed().connect([this](){
            updateSelection();
        });
        toolbar->append(*createToolItem(mTranslations[i]));
    }
    toolbar->append(*Gtk::manage(new Gtk::SeparatorToolItem()));

    toolbar->append(*createToolItem(
        Gtk::manage(new Gtk::Label("Rotation: ")))
    );
    for (int i=0; i<3; ++i) {
        mRotations[i] = createSpinEntry(1, 1);
        mRotations[i]->signal_value_changed().connect([this](){
            updateSelection();
        });
        toolbar->append(*createToolItem(mRotations[i]));
    }
    toolbar->append(*Gtk::manage(new Gtk::SeparatorToolItem()));

    toolbar->append(*createToolItem(
        Gtk::manage(new Gtk::Label("Scale: ")))
    );
    for (int i=0; i<3; ++i) {
        mScales[i] = createSpinEntry(0.0001, 4, 10);
        mScales[i]->signal_value_changed().connect([this](){
            updateSelection();
        });
        toolbar->append(*createToolItem(mScales[i]));
    }
    toolbar->append(*Gtk::manage(new Gtk::SeparatorToolItem()));

    // TODO: Use ToolButton.
    mDynamicButton = Gtk::manage(
        new Gtk::ToggleButton("Dynamic"));
    mDynamicButton->signal_toggled().connect([this]() {
        updateSelection();
    });
    toolbar->append(*createToolItem(mDynamicButton));
}


void MainWindow::changeSelection(size_t selection) {
    mChanging = true;
    mSelection = selection;
    Model& model = mProject.getModel(selection);
    Transformation& t = model.transformation;

    for (int i=0; i<3; ++i)
        mTranslations[i]->set_value(t.position[i]);

    glm::vec3 angles = glm::eulerAngles(t.rotation);
    for (int i=0; i<3; ++i)
        mRotations[i]->set_value(angles[i]);

    for (int i=0; i<3; ++i)
        mScales[i]->set_value(t.scale[i]);
    mChanging = false;
}

void MainWindow::updateSelection() {
    if (!mChanging && mSelection < mProject.getNumModels()) {
        Model& model = mProject.getModel(mSelection);
        Transformation& t = model.transformation;

        for (int i=0; i<3; ++i) {
            float val = mTranslations[i]->get_value();
            t.position[i] = val;
        }

        glm::vec3 angles;
        for (int i=0; i<3; ++i) {
            float val = mRotations[i]->get_value();
            angles[i] = val;
        }
        t.rotation = glm::quat(glm::radians(angles));

        for (int i=0; i<3; ++i) {
            float val = mScales[i]->get_value();
            t.scale[i] = val;
        }

        model.transform();

        auto object = model.getObject();
        bool state = mDynamicButton->get_active();
        if (object->isDynamic() != state) {
            mProject.getPhysicsWorld().remove(*object);
            model.getObject()->setDynamic(state);
            mProject.getPhysicsWorld().add(*object);
        }
    }
}



void MainWindow::addModel() {
    std::string path = openFileDialog(*this, "Load Model");
    if (path != "") {
        mProject.addModel(path);
    }
}


void MainWindow::addPointCloud() {

    InputDialog inputDialog(
        [this] (const InputData& inputData) {
            std::vector<glm::ivec2> points;
            mProject.addPointCloud(inputData, points);
        }
    );

    inputDialog.set_transient_for(*this);
    inputDialog.run();
    inputDialog.hide();

    // std::string path = openFileDialog(*this, "Load Lens Blur Image");
    // if (path != "") {
    //     // First open the image in separate window for surface drawing.
    //     SurfaceEditor* editor = new SurfaceEditor(path);
    //     editor->set_transient_for(*this);
    //     editor->run();
    //     editor->hide();

    //     // Next add as the point cloud.
    //     LensBlurImage lbi(path);
    //     mProject.addPointCloud(
    //         InputData(lbi.getImage(), lbi.getDepthMap()),
    //         editor->getPoints()
    //     );
    //     delete editor;

    //     // Next if this is second point cloud, open two images as pairs
    //     // for intersection area selection.
    //     size_t num = mProject.getNumPointClouds();
    //     if (num > 1) {
    //         CloudPairEditor* editor2 = new CloudPairEditor(
    //             mProject.getPointCloud(num-2).getInputData().getImage(),
    //             cv::imread(path)
    //         );
    //         editor2->set_transient_for(*this);
    //         editor2->run();
    //         editor2->hide();
    //         mProject.addIntersection(num-2, editor2->getArea2(),
    //                                  editor2->getArea1());

    //         std::cout << "Stitching" << std::endl;
    //         CloudStitcher cs(mProject.getPointCloud(num-2),
    //                          mProject.getPointCloud(num-1),
    //                          editor2->getArea1(),
    //                          editor2->getArea2());
    //         cs.stitch();
    //         mProject.getPointCloud(num-1).transformation.setMatrix(
    //             cs.getTransformation()
    //         );
    //         std::cout << "Done" << std::endl;
    //         delete editor2;
    //     }
    // }
}


void MainWindow::loadProject() {
    mProject.clear();
    std::string path = saveFileDialog(*this, "Load project");
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
            // mProject.addPointCloud(path); TODO Save and load floor points.
        }
    }
}

void MainWindow::saveProject() {
    std::string path = saveFileDialog(*this, "Save project");
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

        // file << mProject.getNumPointClouds();
        // for (size_t i=0; i<mProject.getNumPointClouds(); ++i) {
        //     file << mProject.getPointCloud(i).getPath() << std::endl;
        // }
    }
}
