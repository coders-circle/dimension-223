#include <stdinc.h>
#include <ui/ImageEditor.h>


ImageEditor::ImageEditor(const std::string& imagePath)
    : mImage(cv::imread(imagePath)),
      mArea(imagePath, 550, 550*mImage.rows/mImage.cols)
{
    get_content_area()->add(mArea);

    show_all();
}
