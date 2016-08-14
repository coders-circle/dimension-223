#pragma once
#include <gtkmm.h>
#include <graphics/Renderer.h>

/**
 * The main Gtk+ window. This handles creation of all widgets and toolbars
 * inside the window.
 */
class MainWindow : public Gtk::Window {
public:
    MainWindow(Project& project);
    enum SELECTION_TYPE {POINTCLOUD, MODEL};

private:
    Project& mProject;

    GlxWidget mViewport;
    Renderer mRenderer;

    void addMenuItems(Gtk::MenuBar* menuBar);
    void addToolItems(Gtk::Toolbar* toolbar);

    Gtk::SpinButton* mTranslations[3], *mScales[3], *mRotations[3];
    Gtk::ToggleButton* mDynamicButton;
    Gtk::ListViewText* mPointCloudList;

    SELECTION_TYPE mSelectionType;
    size_t mSelection;
    void changeSelection(SELECTION_TYPE type, size_t selection);
    void updateSelection();
    bool mChanging;

    void setTransformation(Transformation& t);
    void updateTransformation(Transformation& t);

    void addModel();
    void addPointCloud();
    void loadProject();
    void saveProject();

    void editPointCloud(size_t index);
    void stitch(size_t index0, size_t index1);

    void exportToVr();
};
