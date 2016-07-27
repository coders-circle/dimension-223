#include <stdinc.h>
#include <ui/InputDialog.h>
#include <ui/widget_creator.h>
#include <backend/LensBlurImage.h>


inline void setViewImage(Gtk::Image* view, cv::Mat& image) {
    cv::imwrite("tmp.jpg", image);
    Glib::RefPtr<Gdk::Pixbuf> pixBuf =
        Gdk::Pixbuf::create_from_file("tmp.jpg");
    pixBuf = pixBuf->scale_simple(
        256*image.cols/image.rows, 256, Gdk::INTERP_BILINEAR
    );
    view->set(pixBuf);
}


InputDialog::InputDialog(
    const std::function<void(const InputData& inputData)>& callback
)
{
    get_content_area()->add(mGrid);

    mImageView = Gtk::manage(new Gtk::Image());
    mDepthView = Gtk::manage(new Gtk::Image());

    mGrid.attach(*mImageView, 0, 0, 4, 4);
    mGrid.attach(*mDepthView, 5, 0, 4, 4);

    Gtk::Button* imgBtn = Gtk::manage(new Gtk::Button("Browse"));
    Gtk::Button* depthBtn = Gtk::manage(new Gtk::Button("Browse"));

    imgBtn->signal_clicked().connect([this]() {
        std::string file = openFileDialog(*this, "Choose image");
        if (file != "") {
            mImage = cv::imread(file);
            setViewImage(mImageView, mImage);
        }
    });

    depthBtn->signal_clicked().connect([this]() {
        std::string file = openFileDialog(*this, "Choose depth map");
        if (file != "") {
            mDepthMap = cv::imread(file);
            setViewImage(mDepthView, mDepthMap);
        }
    });


    mGrid.attach(*imgBtn, 0, 5, 1, 1);
    mGrid.attach(*depthBtn, 5, 5, 1, 1);

    Gtk::Button* lbBtn = Gtk::manage(
        new Gtk::Button("Add from lens blur")
    );
    mGrid.attach(*lbBtn, 0, 6, 2, 2);

    lbBtn->signal_clicked().connect([this]() {
        std::string file = openFileDialog(*this,
            "Choose lens blur image");
        if (file != "") {
            LensBlurImage image(file);
            mImage = image.getImage();
            mDepthMap = image.getDepthMap();

            setViewImage(mImageView, mImage);
            setViewImage(mDepthView, mDepthMap);
        }
        
    });

    Gtk::Button* cancelBtn = Gtk::manage(new Gtk::Button("Cancel"));
    Gtk::Button* addBtn = Gtk::manage(new Gtk::Button("Add"));

    mGrid.attach(*cancelBtn, 6, 8, 1, 1);
    mGrid.attach(*addBtn, 7, 8, 1, 1);

    cancelBtn->signal_clicked().connect([this]() {
        close();
    });
    addBtn->signal_clicked().connect([this, callback]() {
        InputData inputData(mImage, mDepthMap);
        callback(inputData);
        close();
    });

    show_all();
}
