#pragma once

#include <gtkmm/widget.h>
#include <gdk/gdkx.h>
#include <X11/Xlib.h>

#include <GL/glx.h>
#define GLEW_STATIC
#include <GL/glew.h>


/**
 * A Gtk Widget that wraps GLX. It basically acts as
 * OpenGL widget for Gtk+.
 */
class GlxWidget : public Gtk::Widget {
public:
    /**
     * Construct a new GlxWidget widget with given size.
     * @param width Initial width of render window.
     * @param height Initial height of render window.
     */
    GlxWidget(int width, int height);

    /**
     * Destroy the internal GlxWidget window along with the GTK widget.
     */
    virtual ~GlxWidget();

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

    bool isLeftMouseDown() const { return mLeftMouseDown; }
    bool isRightMouseDown() const { return mRightMouseDown; }
    bool isMiddleMouseDown() const { return mMiddleMouseDown; }
    unsigned int getMouseState() const { return mMouseState; }

    void setScrollCallback(const std::function<void(float, float)>& scroll) {
        mScroll = scroll;
    }

protected:

    bool mLeftMouseDown, mRightMouseDown, mMiddleMouseDown;
    unsigned int mMouseState;
    std::function<void(float, float)> mScroll;

    virtual void on_size_allocate(Gtk::Allocation& allocation);
    virtual void on_realize();
    virtual void on_unrealize();
    virtual bool on_draw(const ::Cairo::RefPtr< ::Cairo::Context >& cr);
    virtual bool on_button_press_event(GdkEventButton* button_event);
    virtual bool on_button_release_event(GdkEventButton* release_event);
    virtual bool on_scroll_event(GdkEventScroll* scroll_event);

    Glib::RefPtr<Gdk::Window> mRefGdkWindow;

    // callbacks
    std::function<void()> mDraw;
    std::function<void(int, int)> mResize;
    std::function<void()> mInit;

    // X11 dn GLX stuffs.
    Display *xd;
    XVisualInfo *xvi;
    GLXContext glxc;
};
