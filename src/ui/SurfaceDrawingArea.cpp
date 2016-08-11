#include <stdinc.h>
#include <ui/SurfaceDrawingArea.h>

#include <cairomm/context.h>
#include <giomm/resource.h>
#include <gdkmm/general.h>
#include <glibmm/fileutils.h>


SurfaceDrawingArea::SurfaceDrawingArea(
    const cv::Mat& image,
    int width, int height,
    std::vector<Surface>& surfaces
)
    : mSurfaces(surfaces),
      mWidth(width), mHeight(height),
      mLeftButton(false), mRightButton(false)
{
    cv::imwrite("tmp.jpg", image);
    mImage = Gdk::Pixbuf::create_from_file("tmp.jpg");

    set_size_request(width, height);
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK
               | Gdk::POINTER_MOTION_MASK);

    mImage = mImage->scale_simple(width, height, Gdk::INTERP_BILINEAR);
}

bool SurfaceDrawingArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    Gdk::Cairo::set_source_pixbuf(cr, mImage, 0, 0);
    cr->paint();

    cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);

    if (mSurfaces.size() > mSelection) {
        for (int i=0; i<mHeight; ++i) {
            for (int j=0; j<mWidth; ++j) {
                auto& surface = mSurfaces[mSelection];
                if (surface.marked[i*mWidth + j]) {
                    // cr->arc(j, i, 24.0, 0.0, 2 * M_PI);
                    cr->rectangle(j, i, 1, 1);
                    cr->fill();
                }
            }
        }
    }
    return false;
}

bool SurfaceDrawingArea::on_button_press_event(GdkEventButton* event) {
    if (event->button == 1)
        mLeftButton = true;
    else if (event->button == 3)
        mRightButton = true;
    return false;
}

bool SurfaceDrawingArea::on_button_release_event(GdkEventButton* event) {
    if (event->button == 1)
        mLeftButton = false;
    else if (event->button == 3)
        mRightButton = false;
    return false;
}

bool SurfaceDrawingArea::on_motion_notify_event(GdkEventMotion* event) {
    int ex = (int)event->x;
    int ey = (int)event->y;

    if (mLeftButton) {
        if (mSelection < mSurfaces.size()
            && ey >= 0 && ey < mHeight
            && ex >= 0 && ex < mWidth)
        {
            for (int i = glm::max(0, ey-24);
                 i < glm::min(mHeight, ey+24); ++i)
            {
                for (int j = glm::max(0, ex-24);
                     j < glm::min(mWidth, ex+24); ++j)
                {
                    mSurfaces[mSelection].marked[i*mWidth+j] = true;
                }
            }
            
        }
        queue_draw();
    }
    if (mRightButton) {
        if (mSelection < mSurfaces.size()
            && ey >=0 && ey < mHeight
            && ex >=0 && ex < mWidth)
        {
            for (int i = glm::max(0, ey-24);
                 i < glm::min(mHeight, ey+24); ++i)
            {
                for (int j = glm::max(0, ex-24);
                     j < glm::min(mWidth, ex+24); ++j)
                {
                    mSurfaces[mSelection].marked[i*mWidth+j] = false;
                }
            }
        }
        queue_draw();
    }
    return false;
}
