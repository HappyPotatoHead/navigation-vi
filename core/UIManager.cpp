#include "UIManager.h"

namespace NavigationVI{
    UIManager::UIManager(const std::string& windowName, bool fullscreen, int width, int height)
        : m_windowName(windowName), m_fullscreen(fullscreen), m_width(width), m_height(height)
    {
        cv::namedWindow(m_windowName, cv::WINDOW_NORMAL);
        if (m_fullscreen) {
            cv::setWindowProperty(m_windowName, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        } else {
            cv::resizeWindow(m_windowName, m_width, m_height);
        }
    }

    cv::Mat UIManager::makeComposite(const cv::Mat& mainFeed,
                                    const cv::Mat& mask,
                                    const cv::Mat& qrROI) const
    {
        cv::Mat mainResized;
        cv::resize(mainFeed, mainResized, cv::Size(640, 480));

        cv::Mat maskBGR;
        if (mask.channels() == 1) {
            cv::cvtColor(mask, maskBGR, cv::COLOR_GRAY2BGR);
        } else {
            maskBGR = mask.clone();
        }
        cv::Mat maskResized;
        cv::resize(maskBGR, maskResized, cv::Size(320, 240));

        cv::Mat qrDisplay;
        if (!qrROI.empty()) {
            cv::resize(qrROI, qrDisplay, cv::Size(320, 240));
        } else {
            qrDisplay = cv::Mat::zeros(240, 320, CV_8UC3);
        }

        cv::Mat sidePanel;
        cv::vconcat(maskResized, qrDisplay, sidePanel);

        // Pad side panel to match main feed height
        if (sidePanel.rows < mainResized.rows) {
            cv::copyMakeBorder(sidePanel, sidePanel, 0,
                            mainResized.rows - sidePanel.rows,
                            0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
        }

        cv::Mat composite;
        cv::hconcat(mainResized, sidePanel, composite);

        return composite;
    }

    cv::Mat UIManager::addTextPanel(const cv::Mat& composite,
                                    const std::string& lastQR,
                                    const std::string& destination,
                                    const std::string& suggestion) const
    {
        int panelHeight = 80;
        cv::Mat textPanel(panelHeight, composite.cols, CV_8UC3, cv::Scalar(50, 50, 50));

        cv::putText(textPanel, "Last scanned QR: " + lastQR,
                    {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    {255, 255, 255}, 1);

        cv::putText(textPanel, "Destination: " + destination,
                    {10, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    {255, 255, 255}, 1);

        cv::putText(textPanel, "Suggestion: " + suggestion,
                    {10, 75}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    {0, 255, 0}, 1);

        cv::Mat stacked;
        cv::vconcat(composite, textPanel, stacked);
        return stacked;
    }

    void UIManager::showWindow(const cv::Mat& composite) const {
        cv::imshow(m_windowName, composite);
    }
}