#pragma once
#include <gtkmm.h>
#include <ui/SelectionDrawingArea.h>


class CloudPairEditor : public Gtk::Dialog {
public:
    CloudPairEditor(const cv::Mat& image1, const cv::Mat& image2)
        : mImage1(image1), mImage2(image2),
          mArea1(image1, 550, 550*mImage1.rows/mImage1.cols),
          mArea2(image2, 550, 550*mImage2.rows/mImage2.cols),
          mBox(Gtk::ORIENTATION_HORIZONTAL, 5)
    {
        get_content_area()->add(mBox);
        mBox.add(mArea1);
        mBox.add(mArea2);
        show_all();
    }

    const Area& getArea1() const { return mArea1.getArea(); }
    const Area& getArea2() const { return mArea2.getArea(); }

private:
    cv::Mat mImage1, mImage2;
    SelectionDrawingArea mArea1, mArea2;
    Gtk::Box mBox;

    bool on_key_press_event(GdkEventKey* event) override {
        if (event->keyval == 65307) {
            hide();
        }
        return false;
    }
};
