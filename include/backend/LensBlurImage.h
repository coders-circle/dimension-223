#pragma once

class LensBlurImage {
public:
    LensBlurImage(const std::string& filename);

    cv::Mat& getImage() { return mImage; }
    cv::Mat& getDepthMap() { return mDepth; }

    std::string getFilename() const {
        return mFilename;
    }

private:
    std::string mFilename;

    cv::Mat mImage;
    cv::Mat mDepth;

    std::string mFocalDistance, mDepthNear, mDepthFar;

    std::string match(const std::string& tag, const std::string &src);

    std::string save(const std::string& filename, const std::string& data,
                     const std::string& mime);
};
