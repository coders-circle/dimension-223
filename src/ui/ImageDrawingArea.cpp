#include <stdinc.h>
#include <ui/ImageDrawingArea.h>

#include <cairomm/context.h>
#include <giomm/resource.h>
#include <gdkmm/general.h>
#include <glibmm/fileutils.h>


ImageDrawingArea::ImageDrawingArea(
    const std::string& imagePath,
    int width, int height
)
    : mImage(Gdk::Pixbuf::create_from_file(imagePath)),
      mWidth(width), mHeight(height),
      mLeftButton(false), mRightButton(false)
{
    set_size_request(width, height);
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK
               | Gdk::POINTER_MOTION_MASK);

    mImage = mImage->scale_simple(width, height, Gdk::INTERP_BILINEAR);
}

bool ImageDrawingArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    Gdk::Cairo::set_source_pixbuf(cr, mImage, 0, 0);
    cr->paint();

    cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);
    for (size_t i=0; i<mPoints.size(); ++i) {
      glm::ivec2& p = mPoints[i];
      cr->arc(p.x, p.y, 16.0, 0.0, 2 * M_PI);
      cr->fill();
    }
    return false;
}

bool ImageDrawingArea::on_button_press_event(GdkEventButton* event) {
    if (event->button == 1)
        mLeftButton = true;
    else if (event->button == 3)
        mRightButton = true;
    return false;
}

bool ImageDrawingArea::on_button_release_event(GdkEventButton* event) {
    if (event->button == 1)
        mLeftButton = false;
    else if (event->button == 3)
        mRightButton = false;
    return false;
}

bool ImageDrawingArea::on_motion_notify_event(GdkEventMotion* event) {
    if (mLeftButton) {
        mPoints.push_back(glm::ivec2(event->x, event->y));
        queue_draw();
    }
    if (mRightButton) {
        for (auto it=mPoints.begin(); it!=mPoints.end(); )
        {
            if(glm::abs(it->x - event->x) < 16 &&
               glm::abs(it->y - event->y) < 16)
            {
                it = mPoints.erase(it);
            }
            else {
                ++it;
            }
        }
        queue_draw();
    }
    return false;
}
