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

enum class TTSEngine{
    ESPEAK,
    SAPI,
    GENERIC
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
constexpr float REF_PIXEL_WIDTH{ 140.0f };
constexpr float TARGET_DISTANCE_M{ 0.3f };

std::chrono::steady_clock::time_point lastDistanceTTS = std::chrono::steady_clock::now();

TextToSpeech tts;

static inline void toLowerInPlace(std::string& s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return std::tolower(c); });
}

static inline void toUpperInPlace(std::string& s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return std::toupper(c); });
}

cv::Mat AppController::waitForNextFrame() {
    std::unique_lock<std::mutex> lock(frameMutex);
    frameCV.wait(lock, [] { return !frameQueue.empty() || !running; });
    if (!running) return {};
    cv::Mat frame{ frameQueue.front().clone() };
    frameQueue.pop();
    return frame;
}

void AppController::updateGuidanceOverlay(const QRCode& qr, const cv::Size& frameSize) {
    auto cmd{ detector.getNavigationToQR(qr, frameSize) };
    std::lock_guard<std::mutex> lock(stateMutex);
    lastInstruction = cmd.instruction;
    lastBBox = qr.bbox;
}

bool AppController::isCloseEnough(const QRCode& qr) const {
    float currentWidthPx{ static_cast<float>(qr.bbox.width) };
    float estimateDistanceM{ (REF_DISTANCE_M * REF_PIXEL_WIDTH) / currentWidthPx };
    return estimateDistanceM <= TARGET_DISTANCE_M;
}

cv::Mat AppController::extractQRROI(const QRCode& qr, const cv::Mat& frame) {
    if (!qr.corners.empty() && qr.corners.size() == 4) {
        double side{ 0.0f };
        for (int i = 0; i < 4; ++i) {
            double dist{ cv::norm(qr.corners[i] - qr.corners[(i + 1) % 4]) };
            side = std::max(side, dist);
        }
        int outSide{ static_cast<int>(std::clamp(side, 16.0, 1024.0)) };
        std::vector<cv::Point2f> dst{
            {0, 0},
            {static_cast<float>(outSide - 1), 0},
            {static_cast<float>(outSide - 1), static_cast<float>(outSide - 1)},
            {0, static_cast<float>(outSide - 1)}
        };
        cv::Mat M{ cv::getPerspectiveTransform(qr.corners, dst) };
        cv::Mat roi{};
        cv::warpPerspective(frame, roi, M, cv::Size(outSide, outSide));
        return roi;
    } else {
        cv::Rect roiRect{ qr.bbox & cv::Rect(0, 0, frame.cols, frame.rows) };
        return (roiRect.width > 0 && roiRect.height > 0) ? frame(roiRect).clone() : cv::Mat{};
    }
}

std::string AppController::decodeQR(const cv::Mat& roi) {
    if (roi.empty()) return {};
    std::string content{ reader.reader(roi) };
    auto trim{ [](std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(),
            [](unsigned char ch){ return !std::isspace(ch); }));
        s.erase(std::find_if(s.rbegin(), s.rend(),
            [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
    } };
    trim(content);
    toUpperInPlace(content);
    return content;
}

void AppController::handleDecodedQR(const std::string& content) {
    std::string prevQR;

    // Always update lastQRData, but remember the previous one for comparison
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        prevQR = lastQRData;
        lastQRData = content; // <-- now always reflects the most recent scan
        if (auto resolved = mapSystem.resolveRoomId(content))
            lastRoomName = mapSystem.getRooms().at(resolved.value()).m_name;
        else
            lastRoomName = content + " (unknown)";
    }

    // Only run navigation logic if this is a different QR from the last one
    if (content != prevQR) {
        if (currentInstructions.empty()) {
            // First QR: build and store the full route
            handleNewQR(content);
        } else {
            // Later QR: find this QR in the stored route
            for (size_t i = 0; i < currentInstructions.size(); ++i) {
                if (currentInstructions[i].text.find(content) != std::string::npos) {
                    // Show the NEXT instruction if there is one
                    if (i + 1 < currentInstructions.size()) {
                        currentStepIndex = i+1;
                        currentSuggestion = currentInstructions[currentStepIndex].text;
                    } else {
                        // Already at the last step
                        currentStepIndex = i;
                        currentSuggestion = currentInstructions[i].text;
                        
                        std::lock_guard<std::mutex> qlock(ttsMutex);
                        ttsQueue.push(TTSItem{currentSuggestion, TTSItem::Type::Nav});
                        ttsCV.notify_one();
                    }
                    break;
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            newQRScanned = true;
            lastQRScanTime = std::chrono::steady_clock::now();
            lastQRData = content;
        }

        std::lock_guard<std::mutex> lock(ttsMutex);
        ttsQueue.push(TTSItem{"QR detected: " + content, TTSItem::Type::Announce});
        ttsCV.notify_one();
        if (!lastInstruction.empty()) {
            ttsQueue.push(TTSItem{lastInstruction, TTSItem::Type::Nav});
            ttsCV.notify_one();
        } 
        // If we're at the last step and lastInstruction is empty, speak currentSuggestion instead
        else if (!currentInstructions.empty() && currentStepIndex == currentInstructions.size() - 1) {
            std::cout << currentSuggestion; 
            ttsQueue.push(TTSItem{currentSuggestion, TTSItem::Type::Nav});
            ttsCV.notify_one();
        }
    }
}

void AppController::maybeAdvanceStep() {
    std::string nextText{};
    bool canAdvance = false;

    {
        std::lock_guard<std::mutex> lock(stateMutex);
        if (newQRScanned && !currentInstructions.empty() &&
            currentStepIndex + 1 < currentInstructions.size() - 1) {

            auto now = std::chrono::steady_clock::now();
            if (!navSpeaking && (now - lastQRScanTime >= std::chrono::seconds(3))) {
                canAdvance = true;
                nextText = currentInstructions[currentStepIndex + 1].text;
                newQRScanned = false;
            }
        }
    }

    if (canAdvance) {
        navSpeaking = true;
        {
            std::lock_guard<std::mutex> qlock(ttsMutex);
            ttsQueue.push(TTSItem{nextText, TTSItem::Type::Nav});
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

void AppController::setInstruction(const std::string& text) {
    std::lock_guard<std::mutex> lock(stateMutex);
    lastInstruction = text;
}

void AppController::queueFrame(const cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(frameMutex);
    if (frameQueue.size() > 2) frameQueue.pop();
    frameQueue.push(frame.clone());
    frameCV.notify_one();
}

void AppController::drawOverlay(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(stateMutex);
    if (!lastInstruction.empty())
        cv::putText(frame, lastInstruction, {10, 30},
            cv::FONT_HERSHEY_SIMPLEX, 0.7, {0, 255, 0}, 2);
    if (lastBBox.area() > 0)
        cv::rectangle(frame, lastBBox, {0, 255, 0}, 2);
}

void AppController::showComposite(const cv::Mat& frame, QRColour uiTargetColour) {
    cv::Mat hsv{}, mask{};
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    // mask = detector.makeColourMask(hsv, detector.getTargetColour());
    mask = detector.makeColourMask(hsv, uiTargetColour);
    cv::Mat qrPreview{};
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        qrPreview = lastQRROI.empty() ? cv::Mat(frame.size(), frame.type(), cv::Scalar(0,0,0)) : lastQRROI.clone();
        qrPreview = lastQRROI.clone();
    }
    // Build composite and show
    cv::Mat composite{ ui.makeComposite(frame, mask, qrPreview) };
    cv::Mat finalDisplay{ ui.addTextPanel(composite, lastRoomName, destinationName, currentSuggestion) };
    ui.showWindow(finalDisplay);
}

bool AppController::checkForExitKey() {
    char key{ static_cast<char>(cv::waitKey(1)) };
    if (key == 27) { // ESC
        running = false;
        frameCV.notify_all();
        ttsCV.notify_all();
        return true;
    }
    return false;
}

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
        cv::Mat frame{ waitForNextFrame() };
        if (frame.empty()) break;
        if (!detector.shouldAttemptDetection()) continue;

        cv::Mat hsv{};
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask{ detector.makeColourMask(hsv, detector.getTargetColour()) };
        cv::Mat maskedFrame{};
        cv::bitwise_and(frame, frame, maskedFrame, mask);

        auto codes = detector.detectQRCodes(frame, false);
        auto nearest = detector.findNearestQRCode(codes);
        // Always update guidance
        if (!nearest) {
            std::lock_guard<std::mutex> lock(stateMutex);
            lastInstruction.clear();
            lastBBox = {};
            continue;
        }

        updateGuidanceOverlay(*nearest, frame.size());

        if(!isCloseEnough(*nearest)) {
            setInstruction("Move closer to the QR");

            auto now{ std::chrono::steady_clock::now() };
            if (std::chrono::duration_cast<std::chrono::seconds>(now - lastDistanceTTS).count() >= 2){
                {
                    std::lock_guard<std::mutex> lock(ttsMutex);
                    ttsQueue.push(TTSItem{"Please move closer to the QR code", TTSItem::Type::Announce});
                    ttsCV.notify_one();
                }
                lastDistanceTTS = now;
            }
            continue;
        }

        if (nearest->bbox.width >= QR_DECODE_MIN_WIDTH) {
            auto roi{ extractQRROI(*nearest, frame) };
            auto content{ decodeQR(roi) };

            if (!content.empty()) {
                {
                    std::lock_guard<std::mutex> lock(stateMutex);
                    lastQRROI = roi.clone(); // store only on success
                }
                handleDecodedQR(content);
            } else {
                std::lock_guard<std::mutex> lock(stateMutex);
                lastQRROI.release(); // clear preview on failure
            }
        }

    }
}

void AppController::run() {
    std::vector<std::pair<std::string, QRColour>> colourMap{
        {"red", QRColour::RED},
        { "green", QRColour::GREEN },
        { "blue", QRColour::BLUE },
        { "none", QRColour::NONE }
    };
    int colourChoice{};

    // Start workers
    std::thread ttsThread(&AppController::ttsWorker, this, std::ref(tts));
    std::thread detectThread(&AppController::detectionWorker, this, std::ref(detector), std::ref(reader), std::ref(guider));

    {
        std::lock_guard<std::mutex> lock(ttsMutex);
        std::string menuSpeech{ "Please choose a target QR colour" + TextToSpeech::platformPause(800) + " ." };
        for (size_t i{ 0 }; i < colourMap.size(); ++i) 
            menuSpeech += "Press " + std::to_string(i + 1) + " for " + colourMap[i].first+ TextToSpeech::platformPause(600)  + " ." ;
        ttsQueue.push(TTSItem{menuSpeech, TTSItem::Type::Announce});
        ttsCV.notify_one();
    }

    while(true){
        std::cout << "Enter target QR colour:\n";
        for (size_t i {0}; i < colourMap.size(); i++) 
            std::cout << " " << (i+1) << ". " << colourMap[i].first << "\n";
        std::cout << "Enter choice: ";

        if (!(std::cin >> colourChoice)){
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            {
                std::lock_guard<std::mutex> lock(ttsMutex);
                ttsQueue.push(TTSItem{"Invalid input. Please enter a number", TTSItem::Type::Announce});
                ttsCV.notify_one();  
            }
            continue;
        }
        // std::cin >> colourChoice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        if (colourChoice >= 1 && colourChoice <= (int)colourMap.size()) break;
        else {
            {
                std::lock_guard<std::mutex> lock(ttsMutex);
                ttsQueue.push(TTSItem{"Invalid choice. Please try again.", TTSItem::Type::Announce});
                ttsCV.notify_one();
            }
        }
    }

    QRColour targetColour{ colourMap[colourChoice -1].second };
    std::string chosenColourName{ colourMap[ colourChoice -1]. first };
    std::cout << "You chose colour: " << chosenColourName << "\n";
    {
        std::lock_guard<std::mutex> lock(ttsMutex);
        ttsQueue.push(TTSItem{"You chose colour " + chosenColourName, TTSItem::Type::Announce});
        ttsCV.notify_one();
    }

    if (!mapSystem.loadRoomsFromFile("utils/rooms.txt") ||
        !mapSystem.loadConnectionsFromFile("utils/connections.txt")) {
        std::cerr << "Failed to load map data\n";
        return;
    }


    // std::string destinationId{};
    while(true){
        {
            std::lock_guard<std::mutex> lock(ttsMutex);
            ttsQueue.push(TTSItem{"Enter destination room ID", TTSItem::Type::Announce});
            ttsCV.notify_one();
        }
        std::cout << "Enter destination room ID: ";
        std::cin >> destinationId;

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        
        std::transform(
            destinationId.begin(),
            destinationId.end(),
            destinationId.begin(),
            [](unsigned char c) {
                return std::toupper(c);
            });
        
        auto resolvedDest{ mapSystem.resolveRoomId(destinationId) };
        if (resolvedDest) {
            destinationId = resolvedDest.value();
            destinationName = mapSystem.getRooms().at(destinationId).m_name;
            break;
        } else {
            {
                std::lock_guard<std::mutex> lock(ttsMutex);
                ttsQueue.push(TTSItem{"Destination not found, please try again", TTSItem::Type::Announce});
                ttsCV.notify_one();
            }
        }
    }
    {
        std::lock_guard<std::mutex> lock(ttsMutex);
        ttsQueue.push(TTSItem{"You chose destination "+ destinationId, TTSItem::Type::Announce});
        ttsCV.notify_one();
    }

    // QRColour uiTargetColour{ targetColour };
    // if(colourMap.count(colourChoice)) uiTargetColour = colourMap[colourChoice];

    // detector.setTargetColour(QRColour::NONE);
    detector.setTargetColour(targetColour);
    detector.setMinArea(1000);
    detector.setAspectRatioTolerance(0.8f, 1.25f);
    detector.setBoundingBoxPadding(150);
    detector.setDistanceReference(120.0f, 1.0f);
    detector.setColourVerificationEnabled(true);
    detector.setDetectionThrottle(2,1000);

    // If you're on linux
    std::string pipeline =
    "v4l2src device=/dev/video0 ! "
    "image/jpeg, width=1280, height=720, framerate=30/1 ! "
    "jpegdec ! videoconvert ! appsink";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);

    // If you're not on linux
    // cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return;
    }

    cv::Mat frame{};

    std::string lastSpokenSuggestion{};

    if (routeReset) { lastSpokenSuggestion.clear(); routeReset = false; }

    while (true){
        if (!cap.read(frame) || frame.empty()) break;
        queueFrame(frame);
        drawOverlay(frame);
        maybeAdvanceStep();
        showComposite(frame, targetColour);
        // showComposite(frame, uiTargetColour);

        if(checkForExitKey()) break;
    }

    // Join threads
    detectThread.join();
    ttsThread.join();
}


// void AppController::handleDecodedQR(const std::string& content) {
//     if (content != lastQRData) {
//         handleNewQR(content);
//         {
//             std::lock_guard<std::mutex> lock(stateMutex);
//             newQRScanned = true;
//             lastQRScanTime = std::chrono::steady_clock::now();
//         }
//         std::lock_guard<std::mutex> lock(ttsMutex);
//         ttsQueue.push(TTSItem{"QR detected: " + content, TTSItem::Type::Announce});
//         ttsCV.notify_one();
//     }
// }

// void AppController::maybeAdvanceStep() {
//     std::string nextText{};
//     bool canAdvance = false;

//     {
//         std::lock_guard<std::mutex> lock(stateMutex);
//         if (newQRScanned && m_firstStepAfterQR &&
//             !currentInstructions.empty() &&
//             currentStepIndex + 1 < currentInstructions.size()) {

//             auto now = std::chrono::steady_clock::now();
//             if (!navSpeaking && (now - lastQRScanTime >= std::chrono::seconds(3))) {
//                 canAdvance = true;
//                 nextText = currentInstructions[currentStepIndex + 1].text;
//                 newQRScanned = false;
//             }
//         }
//     }

//     if (canAdvance) {
//         navSpeaking = true;
//         {
//             std::lock_guard<std::mutex> qlock(ttsMutex);
//             ttsQueue.push(TTSItem{nextText, TTSItem::Type::Nav});
//             ttsCV.notify_one();
//         }
//         {
//             std::lock_guard<std::mutex> lock(stateMutex);
//             ++currentStepIndex;
//             currentSuggestion = nextText;
//             m_firstStepAfterQR = false; // reset after advancing
//         }
//     }
// }


// DetectionWorker
        // {
        //     static std::string lastSpokenInstruction;
        //     std::lock_guard<std::mutex> lock(stateMutex);
        //     if (!newQRScanned && !lastInstruction.empty() && lastInstruction != lastSpokenInstruction) {
        //         std::lock_guard<std::mutex> tlock(ttsMutex);
        //         ttsQueue.push(TTSItem{lastInstruction, TTSItem::Type::Nav});
        //         ttsCV.notify_one();
        //         lastSpokenInstruction = lastInstruction;
        //     }
        // }
