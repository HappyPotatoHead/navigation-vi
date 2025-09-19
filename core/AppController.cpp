#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

#include "AppController.h"
#include "../modules/TextToSpeech.h"

#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

struct TTSItem {
    std::string text{};
    enum class Type { Nav, Announce } type;
};

using namespace NavigationVI;

std::queue<cv::Mat> frameQueue{};
std::mutex frameMutex{};
std::condition_variable frameCV{};

std::queue<TTSItem> ttsQueue{};
std::mutex ttsMutex{};
std::condition_variable ttsCV{};

std::atomic<bool> running{ true };

cv::Mat lastQRROI{};

std::mutex stateMutex{};
cv::Rect lastBBox{};
std::string lastInstruction{};

std::mutex speechMutex{};
std::condition_variable speechCV{};
std::chrono::steady_clock::time_point lastSpeechEndTime{ std::chrono::steady_clock::now() };
bool speechFinished{ false };
int navCompletedCount{ 0 };

std::atomic<bool> navSpeaking{ false };
std::atomic<bool> routeReset{ false };

std::atomic<bool> newQRScanned{ false };
std::chrono::steady_clock::time_point lastQRScanTime{};

constexpr int QR_DECODE_MIN_WIDTH{ 120 };
constexpr float REF_DISTANCE_M{ 1.0f };
constexpr float REF_PIXEL_WIDTH{ 120.0f };
constexpr float TARGET_DISTANCE_M{ 0.5f };


TextToSpeech tts;

AppController::AppController()
    : mapSystem("FICT Building", "Ground Floor")
    , ui("Navigation View", false)
    , currentStepIndex(0),
    m_firstStepAfterQR(true) {

    reader.onMessage = [&](const std::string& msg) {
        return;
    };

    guider.onMessage = [&](const std::string& msg) {
        (void)msg;
    };
}

void AppController::handleNewQR(const std::string& content) {
    auto [instrs, summary] {
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
    routeReset = true;
    std::lock_guard<std::mutex> lock(stateMutex);
    lastQRData = content;

    if (auto resolvedStart = mapSystem.resolveRoomId(content))
        lastRoomName = mapSystem.getRooms().at(resolvedStart.value()).m_name;
    else lastRoomName = content + " (unknown)";

    currentInstructions = instrs;
    currentStepIndex = 0;
    lastStepTime = std::chrono::steady_clock::now();
    currentSuggestion = currentInstructions.empty() ? std::string("No path found.") : currentInstructions[0].text;
    m_firstStepAfterQR = true;
    {
        std::lock_guard<std::mutex> slock(speechMutex);
        lastSpeechEndTime = std::chrono::steady_clock::now();
    }
    navSpeaking = false;
}

void AppController::ttsWorker(TextToSpeech& tts) {
    while (running) {
        std::unique_lock<std::mutex> lock(ttsMutex);
        ttsCV.wait(lock, [] { return !ttsQueue.empty() || !running; });
        if (!running) break;
        TTSItem item{ std::move(ttsQueue.front()) };
        ttsQueue.pop();
        lock.unlock();

        tts.speak(item.text);
        if (item.type == TTSItem::Type::Nav) {
            std::lock_guard<std::mutex> slock(speechMutex);
            lastSpeechEndTime = std::chrono::steady_clock::now();
            navSpeaking = false;
        }
    }
}

void AppController::detectionWorker(QRDetector& detector, QRReader& reader, RouteGuidance& guider) {
    while (running) {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            frameCV.wait(lock, [] { return !frameQueue.empty() || !running; });
            if (!running) break;
            frame = frameQueue.front().clone();
            frameQueue.pop();
        }

        cv::Mat hsv{};
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask{ detector.makeColourMask(hsv, detector.getTargetColour()) };
        cv::Mat maskedFrame{};
        cv::bitwise_and(frame, frame, maskedFrame, mask);

        if (!detector.shouldAttemptDetection()) continue;

        auto codes = detector.detectQRCodes(frame, false);
        auto nearest = detector.findNearestQRCode(codes);
        // Always update guidance
        if (nearest) {
            {
                auto cmd = detector.getNavigationToQR(*nearest, frame.size());
                std::lock_guard<std::mutex> lock(stateMutex);
                lastInstruction = cmd.instruction;
                lastBBox = nearest->bbox;
            }

            float currentWidthPx{ static_cast<float>(nearest->bbox.width) };
            float estimateDistanceM{ (REF_DISTANCE_M * REF_PIXEL_WIDTH )/ currentWidthPx };

            if (estimateDistanceM > TARGET_DISTANCE_M){
                {
                    std::lock_guard<std::mutex> lock(stateMutex);
                    lastInstruction = "Move closer to the QR";
                }
                continue;
            }
            // Only decode if QR is large enough
            if (nearest->bbox.width >= QR_DECODE_MIN_WIDTH) {
                cv::Mat qrROI;
                if (!nearest->corners.empty() && nearest->corners.size() == 4) {
                    double side = 0.0;
                    for (int i = 0; i < 4; ++i) {
                        double dist = cv::norm(nearest->corners[i] - nearest->corners[(i + 1) % 4]);
                        side = std::max(side, dist);
                    }
                    int outSide = static_cast<int>(std::clamp(side, 16.0, 1024.0));
                    std::vector<cv::Point2f> dst{
                        {0, 0},
                        { static_cast<float>(outSide - 1), 0 },
                        { static_cast<float>(outSide - 1), static_cast<float>(outSide - 1) },
                        { 0, static_cast<float>(outSide - 1) }
                    };
                    cv::Mat M = cv::getPerspectiveTransform(nearest->corners, dst);
                    cv::warpPerspective(frame, qrROI, M, cv::Size(outSide, outSide));
                } else {
                    cv::Rect roi = nearest->bbox & cv::Rect(0, 0, frame.cols, frame.rows);
                    if (roi.width > 0 && roi.height > 0) qrROI = frame(roi).clone();
                }

                if (!qrROI.empty()) {
                    {
                        std::lock_guard<std::mutex> lock(stateMutex);
                        lastQRROI = qrROI.clone();
                    }
                    std::string content = reader.reader(qrROI);
                    if (!content.empty()) {
                        auto trim = [](std::string& s) {
                            s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                                [](unsigned char ch) { return !std::isspace(ch); }));
                            s.erase(std::find_if(s.rbegin(), s.rend(),
                                [](unsigned char ch) { return !std::isspace(ch); }).base(), s.end());
                        };
                        trim(content);
                        std::transform(content.begin(), content.end(), content.begin(),
                                    [](unsigned char c) { return std::toupper(c); });

                        if (content != lastQRData) {
                            handleNewQR(content);
                            {
                                std::lock_guard<std::mutex> lock(stateMutex);
                                newQRScanned = true;
                                lastQRScanTime = std::chrono::steady_clock::now();
                            }
                            std::lock_guard<std::mutex> lock(ttsMutex);
                            ttsQueue.push(TTSItem{ "QR detected: " + content, TTSItem::Type::Announce });
                            ttsCV.notify_one();
                        }
                    }
                }
            }
        } else {
            std::lock_guard<std::mutex> lock(stateMutex);
            lastInstruction.clear();
            lastBBox = cv::Rect();
        }
    }
}

void AppController::run() {
    std::unordered_map<std::string, QRColour> colourMap{
        {"red", QRColour::RED},
        { "green", QRColour::GREEN },
        { "blue", QRColour::BLUE },
        { "none", QRColour::NONE }
    };

    std::string colourChoice{};
    std::cout << "Enter target QR colour (red/green/blue/none): ";
    std::cin >> colourChoice;
    std::transform(
        colourChoice.begin(),
        colourChoice.end(),
        colourChoice.begin(),
        [](unsigned char c) {
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
        [](unsigned char c) {
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

    //QRColour uiTargetColour{ QRColour::NONE };
    //if(colourMap.count(colourChoice)) uiTargetColour = colourMap[colourChoice];

    //detector.setTargetColour(QRColour::NONE);
    detector.setTargetColour(targetColour);
    detector.setMinArea(300);
    detector.setAspectRatioTolerance(0.5f, 2.0f);
    detector.setBoundingBoxPadding(75);
    detector.setDistanceReference(120.0f, 1.0f);
    detector.setColourVerificationEnabled(true);
    detector.setDetectionThrottle(1, 700);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return;
    }

    // Start workers
    std::thread ttsThread(&AppController::ttsWorker, this, std::ref(tts));
    std::thread detectThread(&AppController::detectionWorker, this, std::ref(detector), std::ref(reader), std::ref(guider));

    cv::Mat frame{};
    std::string lastSpokenSuggestion{};

    auto speakDelayFirst{ std::chrono::seconds(3) };
    auto speakDelayRest{ std::chrono::seconds(15) };

    if (routeReset) { lastSpokenSuggestion.clear(); routeReset = false; }

    while (true) {
        if (!cap.read(frame) || frame.empty()) break;
        //cv::flip(frame, frame, 1);
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            if (frameQueue.size() > 2) frameQueue.pop();
            frameQueue.push(frame.clone());
        }
        frameCV.notify_one();

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            if (!lastInstruction.empty()) {
                cv::putText(frame, lastInstruction, { 10, 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0, 255, 0 }, 2);
            }
            if (lastBBox.area() > 0) {
                cv::rectangle(frame, lastBBox, { 0, 255, 0 }, 2);
            }
        }
        {
    std::string nextText;
    bool canAdvance = false;

    std::chrono::steady_clock::time_point navEndCopy{};
    {
        std::lock_guard<std::mutex> slock(speechMutex);
        navEndCopy = lastSpeechEndTime;
    }

    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (newQRScanned && !currentInstructions.empty() &&
            currentStepIndex + 1 < currentInstructions.size()) {

            auto now = std::chrono::steady_clock::now();
            auto delay = std::chrono::seconds(3);

            if (!navSpeaking && (now - lastQRScanTime >= delay)) {
                canAdvance = true;
                nextText = currentInstructions[currentStepIndex + 1].text;
                newQRScanned = false; // FIXED: was '-' before
            }
        }
    }

    if (canAdvance) {
        navSpeaking = true;
        {
            std::lock_guard<std::mutex> qlock(ttsMutex);
            ttsQueue.push(TTSItem{ nextText, TTSItem::Type::Nav });
            ttsCV.notify_one();
        }
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            ++currentStepIndex;
            currentSuggestion = nextText;
            m_firstStepAfterQR = false;
        }
    }
}
        
        cv::Mat hsv{}, mask{};
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        mask = detector.makeColourMask(hsv, detector.getTargetColour());
        //mask = detector.makeColourMask(hsv, uiTargetColour);
        cv::Mat qrPreview{};
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            qrPreview = lastQRROI.clone();
        }
        // Build composite and show
        cv::Mat composite{ ui.makeComposite(frame, mask, qrPreview) };
        cv::Mat finalDisplay{ ui.addTextPanel(composite, lastRoomName, destinationName, currentSuggestion) };
        ui.showWindow(finalDisplay);

        char key = (char)cv::waitKey(1);
        if (key == 27) { // ESC
            running = false;
            frameCV.notify_all();
            ttsCV.notify_all();
            break;
        }
    }

    // Join threads
    detectThread.join();
    ttsThread.join();
}