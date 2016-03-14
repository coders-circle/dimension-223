#pragma once
#include <gtkmm.h>
#include <ui/SFMLWidget.h>
#include <graphics/Mesh.h>

/**
 * A renderer class that handles the OpenGL widget and the user interaction
 * on it. It also keeps track of all the currently rendered models.
 */
class Renderer
{
public:
    /**
     * Construct a renderer of given size.
     * @param parent Parent Gtk window containing the renderer.
     * @param width Width of the renderer widget.
     * @param height Height of the renderer widget.
     */
    Renderer(Gtk::Window& parent, int width, int height);

private:
    SFMLWidget m_widget;

    void OnInit();
    void OnDraw();
    void OnResize(int width, int height);

};
