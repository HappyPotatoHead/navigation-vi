#pragma once
#include <opencv2/opencv.hpp>
#include <string>

namespace NavigationVI{
    class UIManager{
    public:
        UIManager(
            const std::string& windowName = "Navigation View",
            bool fullscreen = true,
            int width = 1280, 
            int height = 720);

        cv::Mat makeComposite(
            const cv::Mat& mainFeed, 
            const cv::Mat& mask,
            const cv::Mat& qrROI) const;
        
        cv::Mat addTextPanel(const cv::Mat& composite,
                    const std::string& lastQR,
                    const std::string& destination,
                    const std::string& suggestion) const;

        // void drawOverlay(cv::Mat& composite,
        //             const std::string& lastQR,
        //             const std::string& destination,
        //             const std::string& suggestion) const;
    
        void showWindow(const cv::Mat& composite) const;
    
    private:
        std::string m_windowName{};
        bool m_fullscreen{};
        int m_width{};
        int m_height{};
    };
}