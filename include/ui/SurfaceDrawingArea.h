#pragma once

#include <gtkmm.h>
#include <backend/Surface.h>


class SurfaceDrawingArea : public Gtk::DrawingArea
{
public:
    SurfaceDrawingArea(const cv::Mat& image, int width, int height,
                       std::vector<Surface>& surfaces);

    void selectSurface(size_t selection) {
        mSelection = selection;
        queue_draw();
    }

    size_t addSurface() {
        mSurfaces.push_back(Surface());
        mSurfaces[mSurfaces.size()-1].marked.resize(
            mWidth*mHeight, false
        );
        queue_draw();
    }

    void removeSurface() {
        if (mSelection < mSurfaces.size()) {
            mSurfaces.erase(mSurfaces.begin() + mSelection);
            queue_draw();
        }
    }


protected:
    std::vector<Surface>& mSurfaces;
    size_t mSelection;

    Glib::RefPtr<Gdk::Pixbuf> mImage;
    int mWidth, mHeight;

    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    bool on_button_press_event(GdkEventButton* event) override;
    bool on_button_release_event(GdkEventButton* event) override;
    bool on_motion_notify_event(GdkEventMotion* event) override;

    bool mLeftButton;
    bool mRightButton;
};
