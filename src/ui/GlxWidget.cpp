#include <stdinc.h>
#include <ui/GlxWidget.h>


#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);


GlxWidget::GlxWidget(int width, int height)
    : mLeftMouseDown(false), mRightMouseDown(false), mMiddleMouseDown(false)
{
    set_size_request(width, height);
    set_has_window(false); // Makes this behave like an internal object rather then a parent window.
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK
        | Gdk::SCROLL_MASK);
}

GlxWidget::~GlxWidget()
{
}

void GlxWidget::on_size_allocate(Gtk::Allocation& allocation)
{
    // Do something with the space that we have actually been given:
    // (We will not be given heights or widths less than we have requested, though
    // we might get more)

    this->set_allocation(allocation);

    if(mRefGdkWindow)
    {
        mRefGdkWindow->move_resize(allocation.get_x(),
                                    allocation.get_y(),
                                    allocation.get_width(),
                                    allocation.get_height() );

        if (mResize)
            mResize(allocation.get_width(), allocation.get_height());
        if (mDraw)
            mDraw();
    }
}

bool GlxWidget::on_draw(const ::Cairo::RefPtr< ::Cairo::Context >& cr)
{
    if (mDraw)
        mDraw();
    display();
    return true;
}

void GlxWidget::on_realize()
{
    Gtk::Widget::on_realize();

    if(!mRefGdkWindow)
    {
        //Create the GdkWindow:
        GdkWindowAttr attributes;
        memset(&attributes, 0, sizeof(attributes));

        Gtk::Allocation allocation = get_allocation();

        //Set initial position and size of the Gdk::Window:
        attributes.x = allocation.get_x();
        attributes.y = allocation.get_y();
        attributes.width = allocation.get_width();
        attributes.height = allocation.get_height();

        attributes.event_mask = get_events () | Gdk::EXPOSURE_MASK;
        attributes.window_type = GDK_WINDOW_CHILD;
        attributes.wclass = GDK_INPUT_OUTPUT;

        mRefGdkWindow = Gdk::Window::create(get_window(), &attributes,
                GDK_WA_X | GDK_WA_Y);
        set_has_window(true);
        set_window(mRefGdkWindow);

        // transparent background
#if GTK_VERSION_GE(3, 0)
        this->unset_background_color();
#else
        this->get_window()->set_back_pixmap(Glib::RefPtr<Gdk::Pixmap>());
#endif

        this->set_double_buffered(false);

        //make the widget receive expose events
        mRefGdkWindow->set_user_data(gobj());

        xd = gdk_x11_display_get_xdisplay(get_display()->gobj());

        if(!glXQueryExtension(xd, 0, 0))
        {
            std::cout << "OpenGL not supported\n";
        }

        int attrlist [] =
        {
            GLX_RGBA,
            GLX_DEPTH_SIZE, 24,
            GLX_DOUBLEBUFFER,
            0
        };

        int fbcount;
        GLXFBConfig* fbc = glXChooseFBConfig(xd, DefaultScreen(xd), attrlist, &fbcount);
        if (!fbc)
        {
            std::cerr << "Failed to retrieve a framebuffer config\n";
        }
        else
            std::cout << "Found " << fbcount << " matching FB configs.\n";


        int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;
        for (int i=0; i<fbcount; ++i)
        {
            XVisualInfo *vi = glXGetVisualFromFBConfig(xd, fbc[i] );
            if (vi)
            {
                int samp_buf, samples;
                glXGetFBConfigAttrib(xd, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf);
                glXGetFBConfigAttrib(xd, fbc[i], GLX_SAMPLES, &samples);
                printf( "  Matching fbconfig %d, visual ID 0x%2x: SAMPLE_BUFFERS = %d,"
                    " SAMPLES = %d\n", 
                    i, (int)vi->visualid, samp_buf, samples );

                if ( best_fbc <= 0 || samp_buf && samples >= best_num_samp )
                    best_fbc = i, best_num_samp = samples;
                if ( worst_fbc < 0 || !samp_buf || samples < worst_num_samp )
                    worst_fbc = i, worst_num_samp = samples;
            }
            XFree( vi );
        }

        GLXFBConfig bestFbc = fbc[ best_fbc ];

        // Be sure to free the FBConfig list allocated by glXChooseFBConfig()
        XFree(fbc);

        // Get a visual
        xvi = glXGetVisualFromFBConfig( xd, bestFbc );
        printf( "Chosen visual ID = 0x%x\n", (int)xvi->visualid );


        glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
        glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
                     glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB" );

        int context_attribs[] =
        {
            GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
            GLX_CONTEXT_MINOR_VERSION_ARB, 3,
            //GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
            None
        };

        printf( "Creating context\n" );
        glxc = glXCreateContextAttribsARB(xd, bestFbc, 0, True, context_attribs);

        glXMakeCurrent(
            xd,
            gdk_x11_window_get_xid(get_window()->gobj()), glxc
        );

        glewExperimental=GL_TRUE;
        glewInit();
        if (mInit)
            mInit();
    }
}

void GlxWidget::on_unrealize()
{
    glXMakeCurrent(xd, 0, 0);
    glXDestroyContext(xd, glxc);
    mRefGdkWindow.clear();

    //Call base class:
    Gtk::Widget::on_unrealize();
}

void GlxWidget::display()
{
    if (mRefGdkWindow) {
        glXSwapBuffers (xd, gdk_x11_window_get_xid(get_window()->gobj()));
    }
}

void GlxWidget::invalidate()
{
    if(mRefGdkWindow)
        mRefGdkWindow->invalidate(true);
}

bool GlxWidget::on_button_press_event(GdkEventButton* button_event)
{
    if (button_event->button == 1)
        mLeftMouseDown = true;
    else if (button_event->button == 2)
        mMiddleMouseDown = true;
    else if (button_event->button == 3)
        mRightMouseDown = true;
    mMouseState = button_event->state;
    return false;
}

bool GlxWidget::on_button_release_event(GdkEventButton* release_event)
{
    if (release_event->button == 1)
        mLeftMouseDown = false;
    else if (release_event->button == 2)
        mMiddleMouseDown = false;
    else if (release_event->button == 3)
        mRightMouseDown = false;
    return false;
}

bool GlxWidget::on_scroll_event(GdkEventScroll* scroll_event) {
    if (mScroll) {
        mScroll(scroll_event->delta_x, scroll_event->delta_y);
    }
    return false;
}
