#pragma once
#include <gtkmm.h>
#include <backend/InputData.h>


class InputDialog : public Gtk::Dialog {
public:
    InputDialog(
        Gtk::Window* parent,
        const std::function<void(const InputData& inputData)>& callback
    );

private:
    Gtk::Grid mGrid;

    Gtk::Image *mImageView, *mDepthView;

    cv::Mat mImage;
    cv::Mat mDepthMap;
};
