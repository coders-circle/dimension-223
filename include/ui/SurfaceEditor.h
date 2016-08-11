#pragma once
#include <gtkmm.h>
#include <ui/SurfaceDrawingArea.h>
#include <graphics/PointCloud.h>


class SurfaceEditor : public Gtk::Dialog {
public:
    SurfaceEditor(Gtk::Window* parent, PointCloud& pointCloud)
        : mPointCloud(pointCloud),
          mImage(pointCloud.getInputData().getImage()),

          mHBox(Gtk::ORIENTATION_HORIZONTAL),
          mPanel(Gtk::ORIENTATION_VERTICAL),
          mArea(mImage, 550, 550*mImage.rows/mImage.cols,
                pointCloud.getSurfaces()),
          mList(1)
    {
        set_transient_for(*parent);
        get_content_area()->add(mHBox);
        mHBox.add(mArea);
        mHBox.add(mPanel);

        mList.set_size_request(200, 300);
        mPanel.add(mList);
        mList.set_headers_visible(false);
        mList.set_activate_on_single_click(true);
        mList.signal_row_activated().connect(
            [this](const Gtk::TreeModel::Path& path,
                   Gtk::TreeViewColumn* column)
        {
            auto selected = mList.get_selected();
            if (selected.size() > 0) {
                // Select the surface selected[0].
                mArea.selectSurface(selected[0]);
            }
        });

        Gtk::Button* addBtn = Gtk::manage(new Gtk::Button("Add"));
        mPanel.add(*addBtn);
        addBtn->signal_clicked().connect([this](){
            // Add new surface.
            mArea.addSurface();
            refreshList();
        });
 
        Gtk::Button* delBtn = Gtk::manage(new Gtk::Button("Remove"));
        mPanel.add(*delBtn);
        delBtn->signal_clicked().connect([this](){
            mArea.removeSurface();
            refreshList();
        });

        show_all();
        refreshList();
    }

    void refreshList() {
        mList.clear_items();
        for (size_t i=0; i<mPointCloud.getSurfaces().size(); ++i)
            mList.append("Surface #" + std::to_string(i));
    }


private:
    PointCloud& mPointCloud;
    cv::Mat mImage;

    Gtk::Box mHBox;
    Gtk::Box mPanel;
    SurfaceDrawingArea mArea;

    Gtk::ListViewText mList;

    bool on_key_press_event(GdkEventKey* event) override {
        if (event->keyval == 65307) {
            hide();
        }
        return false;
    }
};
