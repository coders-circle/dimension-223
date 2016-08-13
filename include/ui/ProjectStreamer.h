#pragma once
#include <gtkmm.h>
#include <Project.h>
#include <graphics/PointCloud.h>


class ProjectStreamer : public Gtk::Dialog {
public:
    ProjectStreamer(Gtk::Window* parent, Project& project);


private:
    Project& mProject;
    Gtk::Box mHBox;
    Gtk::Button mStartBtn;
    static bool mListening;
    int mSocket;

    void streamProject(int client);

};
