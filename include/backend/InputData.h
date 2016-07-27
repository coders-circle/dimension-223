#pragma once


class InputData {
public:
    InputData(const std::string& imageFile,
              const std::string& depthFile,
              float depthScale=1.0f/300);
    InputData(cv::Mat& image, cv::Mat& depthMap,
              float depthScale=1.0f/300);


    cv::Mat& getImage() { return mImage; }
    cv::Mat& getDepthMap() { return mDepthMap; }
    float getDepthScale() { return mDepthScale; }


private:
    cv::Mat mImage;
    cv::Mat mDepthMap;
    float mDepthScale;

};
