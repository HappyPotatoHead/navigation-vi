#pragma once
#include <string>
#include <vector>
#include <chrono>
#include "../modules/QRDetector.h"
#include "../modules/QRReader.h"
#include "../modules/CoordinateMapSystem.h"
#include "../modules/RouteGuidance.h"
#include "../modules/TextToSpeech.h"
#include "UIManager.h"

namespace NavigationVI{
    class AppController{
    public:
        AppController();

        void ttsWorker(TextToSpeech& tts);
        void detectionWorker(QRDetector& detector, QRReader& reader, RouteGuidance& guider);        
        void run();
    public: 
        bool m_firstStepAfterQR{};
        cv::Mat lastQRROI{};
    private:
        void handleNewQR(const std::string& content);
    private:
        QRDetector detector;
        QRReader reader;
        CoordinateMapSystem mapSystem;
        RouteGuidance guider;
        UIManager ui;
        
        std::string lastQRData{};
        std::string lastRoomName{};
        std::string destinationId{};
        std::string destinationName{};
        std::string currentSuggestion{};
        std::vector<Instruction> currentInstructions{};
        size_t currentStepIndex{};
        std::chrono::steady_clock::time_point lastStepTime{};
        const std::chrono::seconds stepInterval{ 8 };

        double unitScale{ 1.0 };
        double stepLengthM{ 0.75 };
    };
}