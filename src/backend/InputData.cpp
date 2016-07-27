#pragma once

#include <stdinc.h>
#include <backend/InputData.h>


InputData::InputData(
    const std::string& imageFile,
    const std::string& depthFile,
    float depthScale
)
    : mImage(cv::imread(imageFile)),
      mDepthMap(cv::imread(depthFile)),
      mDepthScale(depthScale)
{
}


InputData::InputData(
    cv::Mat& image, cv::Mat& depthMap,
    float depthScale
)
    : mImage(image), mDepthMap(depthMap),
      mDepthScale(depthScale)
{
}
