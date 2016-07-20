#pragma once
#include <gtkmm.h>
#include <ui/SurfaceDrawingArea.h>


class SurfaceEditor : public Gtk::Dialog {
public:
    SurfaceEditor(const std::string& imagePath)
        : mImage(cv::imread(imagePath)),
          mArea(imagePath, 550, 550*mImage.rows/mImage.cols)
    {
        get_content_area()->add(mArea);
        show_all();
    }

    const std::vector<glm::ivec2>& getPoints() const {
        return mArea.getPoints();
    }

private:
    cv::Mat mImage;
    SurfaceDrawingArea mArea;

};
