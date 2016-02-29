#pragma once
#include <gtkmm.h>
#include <ui/SFMLWidget.h>
#include <mesh/Mesh3D.h>
#include <shaders/DiffuseMaterial.h>


/**
 * Main GTK+ window which contains all the user interface.
 */
class MainWindow
{
public:
    /**
     * Construct the window.
     * @param width     Initial width of the window.
     * @param height    Initial height of the window.
     */
    MainWindow(int width, int height);

    /// Destroy the window and its widgets.
    ~MainWindow();

    /**
     * Display the window and starts its main loop.
     * @param  application Application that contains this window.
     * @return             Value returned by the application that runs this window.
     */
    int Run(Glib::RefPtr<Gtk::Application> application)
    {
        return application->run(m_window);
    }

private:
    Gtk::Window m_window;
    SFMLWidget m_widget;

    Mesh3D* m_mesh;
    DiffuseMaterial* m_material;
    Texture* m_texture;

    glm::mat4 m_viewProjection;

    void OnInit();
    void OnDraw();
    void OnResize(int width, int height);
};
