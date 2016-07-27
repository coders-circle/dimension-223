#pragma once


/**
 * A representation of the Google Lens Blur image that
 * stores both the original image and the depth map encoded in the file.
 */
class LensBlurImage {
public:
    LensBlurImage(const std::string& filename);

    cv::Mat& getImage() { return mImage; }
    cv::Mat& getDepthMap() { return mDepth; }

    std::string getFilename() const {
        return mFilename;
    }

    float getFocalDistance() const { return mFocalDistance; }
    float getDepthNear() const { return mDepthNear; }
    float getDepthFar() const { return mDepthFar; }

private:
    std::string mFilename;

    cv::Mat mImage;
    cv::Mat mDepth;

    float mFocalDistance, mDepthNear, mDepthFar;

    std::string match(const std::string& tag, const std::string &src);

    std::string save(const std::string& filename, const std::string& data,
                     const std::string& mime);
};
