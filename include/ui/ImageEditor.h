#pragma once
#include <gtkmm.h>
#include <ui/ImageDrawingArea.h>


class ImageEditor : public Gtk::Dialog {
public:
    ImageEditor(const std::string& imagePath);
    const std::vector<glm::ivec2>& getPoints() const {
        return mArea.getPoints();
    }

private:
    cv::Mat mImage;
    ImageDrawingArea mArea;

};
