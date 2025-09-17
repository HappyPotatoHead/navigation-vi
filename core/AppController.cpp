#include "AppController.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

using namespace NavigationVI;

AppController::AppController()
    : mapSystem("FICT Building", "Ground Floor")
    , ui("Navigation View", false)
    , currentStepIndex(0) {}

void AppController::handleNewQR(const std::string& content) {
    lastQRData = content;
    auto resolvedStart{ mapSystem.resolveRoomId(content) };
    if (resolvedStart) lastRoomName = mapSystem.getRooms().at(resolvedStart.value()).m_name;
    else lastRoomName = content + " (unknown)";

    auto [instrs, summary]{ 
        guider.pathToInstructions(
        mapSystem,
        content,
        destinationId,
        unitScale,
        stepLengthM,
        "steps",
        20.0,
        true
    )};

    currentInstructions = instrs;
    currentStepIndex = 0;
    lastStepTime = std::chrono::steady_clock::now();

    if (!currentInstructions.empty()) currentSuggestion = currentInstructions[0].text;
    else currentSuggestion = "No path found.";
}

void AppController::run() {
    std::unordered_map<std::string, QRColour> colourMap{
        {"red", QRColour::RED},
        {"green", QRColour::GREEN},
        {"blue", QRColour::BLUE},
        {"none", QRColour::NONE}
    };

    std::string colourChoice{};
    std::cout << "Enter target QR colour (red/green/blue/none): ";
    std::cin >> colourChoice;
    std::transform(
        colourChoice.begin(), 
        colourChoice.end(), 
        colourChoice.begin(),
        [](unsigned char c){ 
            return std::tolower(c); 
    });

    QRColour targetColour{ QRColour::NONE };
    if (colourMap.count(colourChoice)) targetColour = colourMap[colourChoice];

    std::cout << "Enter destination room ID (e.g., N008): ";
    std::cin >> destinationId;
    std::transform(
        destinationId.begin(), 
        destinationId.end(), 
        destinationId.begin(),
        [](unsigned char c){ 
            return std::toupper(c); 
    });

    if (!mapSystem.loadRoomsFromFile("utils/rooms.txt") ||
        !mapSystem.loadConnectionsFromFile("utils/connections.txt")) {
        std::cerr << "Failed to load map data\n";
        return;
    }

    auto resolvedDest{ mapSystem.resolveRoomId(destinationId) };
    if (resolvedDest) {
        destinationId = resolvedDest.value();
        destinationName = mapSystem.getRooms().at(destinationId).m_name;
    }

    detector.setTargetColour(targetColour);
    detector.setMinArea(500);
    detector.setAspectRatioTolerance(0.5f, 2.0f);
    detector.setBoundingBoxPadding(25);
    detector.setDistanceReference(120.0f, 1.0f);
    detector.setColourVerificationEnabled(true);
    detector.setDetectionThrottle(5, 700);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return;
    }

    cv::Mat frame{};
    while (true) {
        if (!cap.read(frame) || frame.empty()) break;
        cv::flip(frame, frame, 1);

        auto codes{ detector.detectQRCodes(frame, false) };
        auto nearest{ detector.findNearestQRCode(codes) };

        cv::Mat qrROI{};
        if (nearest) {
            if (!nearest->corners.empty() && nearest->corners.size() == 4) {
                double side{ 0.0f };
                for (int i = 0; i < 4; i++) {
                    double dist{ cv::norm(nearest->corners[i] - nearest->corners[(i + 1) % 4]) };
                    side = std::max(side, dist);
                }
                int outSide{ static_cast<int>(std::clamp(side, 16.0, 1024.0)) };
                std::vector<cv::Point2f> dst{
                    {0, 0},
                    {static_cast<float>(outSide - 1), 0},
                    {static_cast<float>(outSide - 1), static_cast<float>(outSide - 1)},
                    {0, static_cast<float>(outSide - 1)}
                };
                cv::Mat M{ cv::getPerspectiveTransform(nearest->corners, dst) };
                cv::warpPerspective(frame, qrROI, M, cv::Size(outSide, outSide));
            } else {
                cv::Rect roi{ nearest->bbox & cv::Rect(0, 0, frame.cols, frame.rows) };
                if (roi.width > 0 && roi.height > 0) qrROI = frame(roi).clone();
            }

            if (!qrROI.empty()) {
                std::string content{ reader.reader(qrROI) };
                if (!content.empty()) {
                    auto trim{ [](std::string &s) {
                        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                            [](unsigned char ch){ return !std::isspace(ch); }));
                        s.erase(std::find_if(s.rbegin(), s.rend(),
                            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
                    } };
                    trim(content);
                    std::transform(content.begin(), content.end(), content.begin(),
                                   [](unsigned char c){ return std::toupper(c); });

                    if (content != lastQRData) handleNewQR(content);
                }
            }

            if (detector.shouldAttemptDetection()) {
                auto cmd{ detector.getNavigationToQR(*nearest, frame.size()) };
                cv::putText(frame, cmd.instruction, {10, 30},
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
            }

            cv::rectangle(frame, nearest->bbox, {0, 255, 0}, 2);
        }

        if (!currentInstructions.empty() && currentStepIndex + 1 < currentInstructions.size()) {
            auto now{ std::chrono::steady_clock::now() };
            if (now - lastStepTime >= stepInterval) {
                currentStepIndex++;
                currentSuggestion = currentInstructions[currentStepIndex].text;
                lastStepTime = now;
            }
        }

        cv::Mat hsv{}, mask{};
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        mask = detector.makeColourMask(hsv, detector.getTargetColour());

        cv::Mat composite{ ui.makeComposite(frame, mask, qrROI) };
        cv::Mat finalDisplay{ ui.addTextPanel(composite, lastRoomName, destinationName, currentSuggestion) };
        // ui.drawOverlay(composite, lastRoomName, destinationName, currentSuggestion);
        ui.showWindow(finalDisplay);

        char key = (char)cv::waitKey(1);
        if (key == 27) break; // ESC to quit
    }
}