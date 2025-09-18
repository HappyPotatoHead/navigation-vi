#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <string>
#include <vector>

#include <functional>

namespace NavigationVI {
    class QRReader{
        public:
            std::string reader(const cv::Mat& imageBGR);
        public:
            std::function<void(const std::string&)> onMessage{};
        private:
            cv::Mat toGray(const cv::Mat& image) const;
            cv::Mat perspectiveCorrection(const cv::Mat& image) const;
            std::string printResult(const std::vector<zbar::Symbol>& zbarResults) const;
            std::vector<zbar::Symbol> decodeWithZbar(const cv::Mat& image, zbar::ImageScanner& scanner) const;
            std::string decodeQR(const cv::Mat& image, 
                                zbar::ImageScanner& scanner, 
                                const std::string& stageName) const;
            cv::Mat applyUpscaling(const cv::Mat& image) const;
            cv::Mat thresholdImage(const cv::Mat& image) const;
    };
}