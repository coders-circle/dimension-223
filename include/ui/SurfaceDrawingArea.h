#pragma once

#include <gtkmm.h>

class SurfaceDrawingArea : public Gtk::DrawingArea
{
public:
    SurfaceDrawingArea(const std::string& mImagePath, int width, int height);
    const std::vector<glm::ivec2>& getPoints() const { return mPoints; }

protected:

    Glib::RefPtr<Gdk::Pixbuf> mImage;
    std::vector<glm::ivec2> mPoints;
    int mWidth, mHeight;

    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    bool on_button_press_event(GdkEventButton* event) override;
    bool on_button_release_event(GdkEventButton* event) override;
    bool on_motion_notify_event(GdkEventMotion* event) override;

    bool mLeftButton;
    bool mRightButton;
};
