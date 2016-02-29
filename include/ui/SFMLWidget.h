#pragma once

// Include necessary GL, SFMl and Gtk headers
#define GLEW_STATIC
#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <gtkmm/widget.h>


/**
 * GTK wrapper for SFML render window.
 *
 * This widget can be added as normal GTK widget but can be used
 * like SFML window with OpenGL rendering capabilities.
 */
class SFMLWidget : public Gtk::Widget
{
public:
    /**
     * The internal SFML window where stuffs can be rendered.
     */
    sf::RenderWindow renderWindow;

    /**
     * Construct a new SFML widget with given size.
     * @param width  Initial width of render window.
     * @param height Initial height of render window.
     */
    SFMLWidget(int width, int height);

    /**
     * Destroy the internal SFML window along with the GTK widget.
     */
    virtual ~SFMLWidget();

    /**
     * Invalidate the widget so it will draw itself next when it gets a chance.
     */
    void Invalidate();
    /**
     * Display everything drawn on the backbuffer onto the widget.
     */
    void Display();

    /**
     * Set callback function to call when the widget needs to be drawn.
     * @param draw  The draw event handler.
     */
    void SetDrawCallback(const std::function<void()>& draw)
    { m_draw = draw; }

    /**
     * Set callback function to call when the widget is resized.
     * @param resize  The resize event handler.
     */
    void SetResizeCallback(const std::function<void(int, int)>& resize)
    { m_resize = resize; }

    /**
     * Set callback function to call when the widget is succesfully created.
     * This is the right place to perform all the opengl intialization stuffs.
     * @param draw  The initialization event handler.
     */
    void SetInitCallback(const std::function<void()>& init)
    { m_init = init; }

protected:

    virtual void on_size_allocate(Gtk::Allocation& allocation);
    virtual void on_realize();
    virtual void on_unrealize();
    virtual bool on_draw(const ::Cairo::RefPtr< ::Cairo::Context >& cr);

    Glib::RefPtr<Gdk::Window> m_refGdkWindow;

    // callbacks
    std::function<void()> m_draw;
    std::function<void(int, int)> m_resize;
    std::function<void()> m_init;
};
