#pragma once
#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

namespace NavigationVI{
    enum class QRColour {
        RED,
        GREEN,
        BLUE,
        NONE
    };

    struct HSVRange{
        cv::Scalar lower{};
        cv::Scalar upper{};
    };

    struct DetectResult{
        bool found{ false };
        std::string content{};
        cv::Rect bbox{};
        std::vector<cv::Point2f> corners;
    };

    enum class Direction{
        FORWARD, 
        BACKWARD,
        LEFT,
        RIGHT,
        TURN_LEFT_45,
        TURN_RIGHT_45,
        TURN_LEFT_90,
        TURN_RIGHT_90
    };

    struct QRCode{
        cv::Point2f position;
        std::string content;
        QRColour colour;
        float distance;
        cv::Rect bbox;
        std::vector<cv::Point2f> corners;
    };

    struct NavigationCommand{
        Direction direction;
        float angle;
        float distance;
        std::string instruction;
    };
}