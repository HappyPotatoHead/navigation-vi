#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <optional>
#include <map>
#include <chrono>
#include "../utils/QRCode.h"

namespace NavigationVI{
    class QRDetector{
        public:
            QRDetector();

            bool initialise(const int cameraIndex = 0);
            void release();

            void setTargetColour(const QRColour colour);
            QRColour getTargetColour() const;
            
            void setDetectionThrottle(int framesInterval, int minGapMs);
            bool shouldAttemptDetection();

            std::vector<QRCode> detectQRCodes(const cv::Mat& frame, bool tryDecode = false);
            std::optional<QRCode> findNearestQRCode(const std::vector<QRCode>& codes) const;
            NavigationCommand getNavigationToQR(const QRCode& qr, const cv::Size& frameSize) const;

            void setMinArea(int area);
            void setAspectRatioTolerance(float low, float high);
            void setBoundingBoxPadding(int px);
            void setDistanceReference(float pxAt1m, float meters);

            void setColourVerificationEnabled(bool enabled);
            bool getColourVerificationEnabled() const;
            cv::Mat makeColourMask(const cv::Mat& hsv, QRColour colour) const;
            
        // optional: Can delete later
        public:
            bool isOpened() const { return m_camera.isOpened(); }
            bool readFrame(cv::Mat& out) { return m_camera.read(out); }
        
        private:
            cv::VideoCapture m_camera{};
            QRColour m_targetColour{ QRColour::NONE };

            int m_minAreaPx{ 1000 };
            float m_aspectRatioLow{ 0.6f };
            float m_aspectRatioHigh{ 1.8f };
            int m_bbboxPadding{ 25 }; 
            float m_referencePx{ 120.0f };
            float m_referenceMeters{ 1.0f };
            
            bool m_colourVerifyEnabled{ true };

            int m_detectionIntervalFrames{ 5 };
            int m_minDetectionGapMs{ 500 };
            int m_frameSkipCounter{ 0 };
            std::chrono::steady_clock::time_point m_lastDetectionTime;

            std::map<QRColour, std::vector<HSVRange>> m_colourRanges{};

            std::vector<cv::Rect> findCandidateROIs(const cv::Mat& mask) const;

            DetectResult robustDetectInROI(const cv::Mat& frame, const cv::Rect& roi, bool tryDecode) const;

            static cv::Rect padRect(const cv::Rect& r, int pad, const cv::Size& maxSize);
            static bool isAspectOk(const cv::Rect& r, float low, float high);
            static float estimateDistancePxToMeters(float bboxPx, float referencePx, float referenceMeters);
            static float averageSide(const cv::Rect& r);
            bool verifyColourInROI(const cv::Mat& frame, const cv::Rect& roi, QRColour colour) const;
        };
}