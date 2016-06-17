#include <stdinc.h>
#include <backend/base64.h>
#include <backend/LensBlurImage.h>


LensBlurImage::LensBlurImage(const std::string& filename) {
    std::ifstream inFile(filename);
    std::stringstream stream;
    stream << inFile.rdbuf();

    std::string src = stream.str();

    std::string depth = match("GDepth:Data", src);
    std::string image = match("GImage:Data", src);

    std::string dmime = match("GDepth:Mime", src);
    std::string imime = match("GImage:Mime", src);

    mFocalDistance = match("GFocus:FocalDistance", src);
    mDepthNear = match("GDepth:Near", src);
    mDepthFar = match("GDepth:Far", src);

    std::string depthFile = save("tmp_depth", depth, dmime);
    std::string imgFile = save("tmp_image", image, imime);

    mDepth = cv::imread(depthFile);
    mImage = cv::imread(imgFile);

    // Invert the depth map and resize it.
    cv::Mat temp = cv::Mat::zeros(mDepth.size(), mDepth.type());
    cv::Mat sub_mat = cv::Mat(mDepth.rows, mDepth.cols, mDepth.type(),
        cv::Scalar(1,1,1))*255;
    cv::subtract(sub_mat, mDepth, temp);
    cv::Size size(550, mDepth.cols/(float)mDepth.rows*550);
    cv::resize(temp, mDepth, size);
}


std::string LensBlurImage::save(const std::string& filename,
        const std::string& data, const std::string& mime) {

    std::string ext = "";
    if (mime == "image/jpeg")
        ext = ".jpg";
    else if (mime == "image/png")
        ext = ".png";

    else {
        // TODO: Exception
        std::cout << "Couldn't recognize mime: " << mime << std::endl;
        return "";
    }

    std::string temp = "";
    for (size_t i=0; i<data.size(); ++i) {
        if (data[i] == (char)0xff) {
            i += 79;
        }
        temp += data[i];
    }

    std::ofstream outFile(filename+ext, std::ios::binary);
    outFile << base64_decode(temp);

    return filename+ext;
}


std::string LensBlurImage::match(const std::string& tag, const std::string &src) {
    size_t pos = src.find(tag+"=\"");
    size_t tmp = std::string(tag+"=\"").size();

    if (pos != std::string::npos) {
        size_t epos = src.find("\"", pos+tmp);
        return src.substr(pos+tmp, epos-pos-tmp);
    }

    return "";
}
