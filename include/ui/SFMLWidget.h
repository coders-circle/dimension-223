#pragma once

#define GLEW_STATIC
#include <GL/glew.h>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <gtkmm/widget.h>


/**
 * A Gtk Widget that wraps an SFML renderer. It basically acts as
 * OpenGL widget for Gtk+.
 */
class SFMLWidget : public Gtk::Widget {
public:
    /**
     * The internal SFML window where stuffs can be rendered.
     */
    sf::RenderWindow renderWindow;

    /**
     * Construct a new SFML widget with given size.
     * @param width Initial width of render window.
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
    void invalidate();
    /**
     * Display everything drawn on the backbuffer onto the widget.
     */
    void display();

    /**
     * Set callback function to call when the widget needs to be drawn.
     * @param draw The draw event handler.
     */
    void setDrawCallback(const std::function<void()>& draw)
    { mDraw = draw; }

    /**
     * Set callback function to call when the widget is resized.
     * @param resize The resize event handler.
     */
    void setResizeCallback(const std::function<void(int, int)>& resize)
    { mResize = resize; }

    /**
     * Set callback function to call when the widget is succesfully created.
     * This is the right place to perform all the opengl intialization stuffs.
     * @param draw The initialization event handler.
     */
    void setInitCallback(const std::function<void()>& init)
    { mInit = init; }

protected:

    virtual void on_size_allocate(Gtk::Allocation& allocation);
    virtual void on_realize();
    virtual void on_unrealize();
    virtual bool on_draw(const ::Cairo::RefPtr< ::Cairo::Context >& cr);

    Glib::RefPtr<Gdk::Window> mRefGdkWindow;

    // callbacks
    std::function<void()> mDraw;
    std::function<void(int, int)> mResize;
    std::function<void()> mInit;
};
