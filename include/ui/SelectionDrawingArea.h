#pragma once
#include <gtkmm.h>


class SelectionDrawingArea : public Gtk::DrawingArea
{
public:
    SelectionDrawingArea(const cv::Mat& image, int width, int height);

    const Area& getArea() const {
        return mArea;
    }

protected:
    Area mArea;

    Glib::RefPtr<Gdk::Pixbuf> mImage;
    int mWidth, mHeight;

    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    bool on_button_press_event(GdkEventButton* event) override;
    bool on_button_release_event(GdkEventButton* event) override;
    bool on_motion_notify_event(GdkEventMotion* event) override;

    bool mLeftButton;
};
