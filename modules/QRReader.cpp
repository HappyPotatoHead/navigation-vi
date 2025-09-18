#include "QRReader.h"
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>

namespace NavigationVI{
    static inline bool tooSmall(const cv::Mat& img, int minSide=8){
        return img.empty() || img.cols < minSide || img.rows < minSide;
    }

    cv::Mat QRReader::toGray(const cv::Mat& image) const{
        if(image.empty()) return {};
        if (image.channels() == 1) return image;
        cv::Mat gray{};
        if (image.channels() == 4) cv::cvtColor(image, gray, cv::COLOR_RGBA2GRAY);
        else cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        return gray;
    }

    cv::Mat QRReader::perspectiveCorrection(const cv::Mat& image) const{
        if (tooSmall(image, 12)) return image;

        cv::QRCodeDetector detector{};
        std::vector<cv::Point2f> points{};
        bool retval{ false };
        try{
            retval = detector.detect(image, points);
        } catch (const cv::Exception& e){
            std::cerr << "QRReader::perspectiveCorrection detect() error: " << e.what() << std::endl;
            return image;
        }

        if (retval && points.size() == 4){
            double side{ 0.0f }; 
            for (int i{ 0 }; i < 4; i++){
                double dist{ cv::norm(points[i] - points[(i + 1)%4]) };
                side = std::max(side, dist);
            }

            int outSide{ static_cast<int>(std::clamp(side, 16.0, 1024.0)) };
            if (outSide < 16) return image;

            std::vector<cv::Point2f> dstPoints{
                {0.0f, 0.0f},
                {static_cast<float>(outSide - 1), 0.0f},
                {static_cast<float>(outSide - 1), static_cast<float>(outSide - 1)},
                {0.0f, static_cast<float>(outSide - 1)},
            };
            // std::vector<cv::Point2f> dstPoints {
            //     {
            //         cv::Point2f(0, 0),
            //         cv::Point2f(side - 1, 0),
            //         cv::Point2f(side - 1, side - 1),
            //         cv::Point2f(0, side - 1),
            //     }
            // };
            cv::Mat M{};
            try{
                M = cv::getPerspectiveTransform(points, dstPoints);
            } catch (const cv::Exception& e){
                std::cerr << "QRReader::perspectiveCorrection getPerspectiveTransform error: " << e.what() << std::endl;
                return image;
            }

            // cv::Mat M{ cv::getPerspectiveTransform(points, dstPoints) };
            cv::Mat warped{};
            try{
                cv::warpPerspective(image, warped, M, cv::Size{outSide, outSide});
            } catch(const cv::Exception& e) {
                std::cerr << "QRReader::perspectiveCorrection warpPerspective error: " << e.what() << std::endl;
                return image;
            }
            return warped;
        }
        return image;
    }

    std::string QRReader::printResult(const std::vector<zbar::Symbol>& zbarResults) const{
        std::string data{};
        for(const auto& symbol : zbarResults){
            std::string msg = "QR detected: " + symbol.get_data();
            std::cout << msg << std::endl;
            // std::cout << "Data: " << symbol.get_data() << std::endl;
            // std::cout << "Type: " << symbol.get_type_name() << std::endl;
            if (onMessage) onMessage(msg);
            data = symbol.get_data();
        }
        return data;
    }

    std::vector<zbar::Symbol> QRReader::decodeWithZbar(const cv::Mat& image, zbar::ImageScanner& scanner) const {
        std::vector<zbar::Symbol> results{};
        if (tooSmall(image, 8)) return results;
        cv::Mat gray{};
        if(image.channels() == 1) gray = image;
        else cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        if(tooSmall(gray, 8)) return results;

        zbar::Image zbarImage(
            gray.cols, 
            gray.rows, 
            "Y800",
            static_cast<void*>(const_cast<uchar*>(gray.data)),
            gray.cols * gray.rows
        );

        int n{ scanner.scan(zbarImage) };
        if (n > 0) {
            for (zbar::Image::SymbolIterator symbol{zbarImage.symbol_begin()};
                symbol != zbarImage.symbol_end(); ++symbol)
            {
                results.push_back(*symbol);
            }
        }
        return results;
    }

    std::string QRReader::decodeQR(const cv::Mat& image, zbar::ImageScanner& scanner, const std::string& stageName) const{
        if (tooSmall(image, 8)) return "";
        std::vector<zbar::Symbol> results{ decodeWithZbar(image, scanner) };
        if (!results.empty()){
            std::cout << "\nZbar detected " << results.size()
                      << " Qr code(s) in " << stageName << std::endl;
            return printResult(results);
        }
        return "";
    }

    cv::Mat QRReader::applyUpscaling(const cv::Mat& image) const{
        if (tooSmall(image, 8)) return image;
        cv::Mat upscaled{};
        cv::resize(image, upscaled, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
        return upscaled;
    }

    cv::Mat QRReader::thresholdImage(const cv::Mat& image) const{
        if (tooSmall(image, 8)) return image;
        cv::Mat threshold{};
        cv::threshold(image, threshold, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        return threshold;
    }

    std::string QRReader::reader(const cv::Mat& imageBGR){
        if (tooSmall(imageBGR, 12)) {
            std::cout << "QRReader: empty or too small image, skipping." << std::endl;
            return "";
        }

        zbar::ImageScanner scanner{};
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
        
        {
            std::string result{ decodeQR(imageBGR, scanner, "original image") };
            if (!result.empty()) return result;
        }
        
        cv::Mat gray{ toGray(imageBGR) };
        if (tooSmall(gray, 12)) return "";

        {
            std::string result{ decodeQR(gray, scanner, "gray image") };
            if (!result.empty()) return result;
        }

        cv::Mat corrected{};
        try {
            corrected = perspectiveCorrection(gray);
        } catch(const cv::Exception& e) {
            std::cerr << "QRReader::reader perspectiveCorrection error: " << e.what() << std::endl;
            corrected = gray;
        }
        if (tooSmall(corrected, 12)) corrected = gray;

        cv::Mat upscaled{ applyUpscaling(corrected) };
        if(!tooSmall(upscaled, 8)) {
            std::string result{ decodeQR(upscaled, scanner, "upscaled image") };
            if(!result.empty()) return result;

            cv::Mat thresholded{ thresholdImage(upscaled) };
            if(!tooSmall(thresholded, 8)){
                result = decodeQR(thresholded, scanner, "thresholded image");
                if (!result.empty()) return result;
            }
        }

        std::cout << "\nZBar: No QR code detected" << std::endl;
        // if (onMessage) onMessage("No QR code detected");
        return "";
    }
}