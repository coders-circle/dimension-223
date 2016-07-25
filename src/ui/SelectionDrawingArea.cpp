#include <stdinc.h>
#include <ui/SelectionDrawingArea.h>

#include <cairomm/context.h>
#include <giomm/resource.h>
#include <gdkmm/general.h>
#include <glibmm/fileutils.h>


SelectionDrawingArea::SelectionDrawingArea(
    const std::string& imagePath,
    int width, int height
)
    : mImage(Gdk::Pixbuf::create_from_file(imagePath)),
      mWidth(width), mHeight(height),
      mLeftButton(false)
{
    set_size_request(width, height);
    add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK
               | Gdk::POINTER_MOTION_MASK);

    mImage = mImage->scale_simple(width, height, Gdk::INTERP_BILINEAR);

    mArea.x1 = mArea.x2 = mArea.y1 = mArea.y2 = 0;
}

bool SelectionDrawingArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
    Gdk::Cairo::set_source_pixbuf(cr, mImage, 0, 0);
    cr->paint();

    if (mArea.x1 != mArea.x2 && mArea.y1 != mArea.y2) {
        cr->set_source_rgba(1, 1, 1, 0.5);
        cr->rectangle(mArea.x1, mArea.y1,
                      mArea.x2-mArea.x1, mArea.y2-mArea.y1);
        cr->fill();
    }

    return false;
}

bool SelectionDrawingArea::on_button_press_event(GdkEventButton* event) {
    if (event->button == 1) {
        mLeftButton = true;

        mArea.x1 = mArea.x2 = event->x;
        mArea.y1 = mArea.y2 = event->y;
    }
    return false;
}

bool SelectionDrawingArea::on_button_release_event(GdkEventButton* event) {
    if (event->button == 1) {
        mLeftButton = false;

        mArea.x2 = event->x;
        mArea.y2 = event->y;

        int x1 = glm::max(0, glm::min(mArea.x1, mArea.x2));
        int x2 = glm::min(mWidth, glm::max(mArea.x1, mArea.x2));
        int y1 = glm::max(0, glm::min(mArea.y1, mArea.y2));
        int y2 = glm::min(mHeight, glm::max(mArea.y1, mArea.y2));
        mArea.x1 = x1; mArea.x2 = x2; mArea.y1 = y1; mArea.y2 = y2;
    }
    return false;
}

bool SelectionDrawingArea::on_motion_notify_event(GdkEventMotion* event) {
    if (mLeftButton) {
        mArea.x2 = event->x;
        mArea.y2 = event->y;
        queue_draw();
    }
    return false;
}
