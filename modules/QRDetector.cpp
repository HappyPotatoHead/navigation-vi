#include "QRDetector.h"
#include <algorithm>
#include <cmath>

namespace NavigationVI{
    // Minimum ROI side
    static inline bool minRoiOk(const cv::Rect& r, int minSide = 16) {
        return r.width >= minSide && r.height >= minSide;
    }

    // Sanity check on 4-point polygon
    static inline bool polygonIsSane(const std::vector<cv::Point2f>& pts) {
        if (pts.size() != 4) return false;

        double area = std::fabs(cv::contourArea(pts));
        if (area < 25.0) return false;

        std::vector<cv::Point> polyInt; polyInt.reserve(4);
        for (auto& p : pts) polyInt.emplace_back(cv::Point(cv::saturate_cast<int>(p.x),
                                                        cv::saturate_cast<int>(p.y)));
        if (!cv::isContourConvex(polyInt)) return false;

        double d01 = cv::norm(pts[0] - pts[1]);
        double d12 = cv::norm(pts[1] - pts[2]);
        double d23 = cv::norm(pts[2] - pts[3]);
        double d30 = cv::norm(pts[3] - pts[0]);
        double longest = std::max(std::max(d01, d12), std::max(d23, d30));
        double shortest = std::min(std::min(d01, d12), std::min(d23, d30));
        if (shortest < 1.0 || longest / shortest > 4.0) return false;

        return true;
    }

    // Build padded, clamped bbox from polygon in frame space
    static inline cv::Rect bboxFromCorners(const std::vector<cv::Point2f>& corners,
                                        int pad,
                                        const cv::Size& frameSize) {
        cv::Rect r = cv::boundingRect(corners);
        r.x = std::max(0, r.x - pad);
        r.y = std::max(0, r.y - pad);
        r.width  += 2 * pad;
        r.height += 2 * pad;
        r &= cv::Rect(0, 0, frameSize.width, frameSize.height);
        return r;
    }

    QRDetector::QRDetector()
    : m_targetColour(QRColour::NONE),
      m_minAreaPx(1000),
      m_aspectRatioLow(0.6f),
      m_aspectRatioHigh(1.6f),
      m_bbboxPadding(25),
      m_referencePx(120.0f),
      m_referenceMeters(1.0f),
      m_colourVerifyEnabled(true),
      m_detectionIntervalFrames(5),
      m_minDetectionGapMs(500),
      m_frameSkipCounter(0),
      m_lastDetectionTime(std::chrono::steady_clock::now()){

        m_colourRanges[QRColour::BLUE] = {
            { 
                cv::Scalar(95, 60, 50), cv::Scalar(140, 255, 255)
            }
        };

        m_colourRanges[QRColour::GREEN] = {
            {
                cv::Scalar(35, 60, 50), cv::Scalar(85, 255, 255)
            }
        };

        m_colourRanges[QRColour::RED] = {
            {
                cv::Scalar(0, 60, 50), cv::Scalar(10, 255, 255)
            },
            {
                cv::Scalar(170, 60, 50), cv::Scalar(179, 255, 255)
            }
        };
      }
    //   Destructor is not needed

    bool QRDetector::initialise(int cameraIndex){
        m_camera.open(cameraIndex);
        return m_camera.isOpened();
    }

    void QRDetector::release(){
        if (m_camera.isOpened()) m_camera.release();
    }

    void QRDetector::setDetectionThrottle(int framesInterval, int minGapMs){
        m_detectionIntervalFrames = std::max(1, framesInterval);
        m_minDetectionGapMs = std::max(0, minGapMs);
    }

    bool QRDetector::shouldAttemptDetection(){
        ++m_frameSkipCounter;
        auto now{ std::chrono::steady_clock::now() };
        auto ms{ std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastDetectionTime).count() };

        bool intervalPassed{ m_frameSkipCounter >= m_detectionIntervalFrames };
        bool cooldownPassed{ ms >= m_minDetectionGapMs };

        if( intervalPassed && cooldownPassed ){
            m_frameSkipCounter = 0;
            m_lastDetectionTime = now;
            return true;
        }
        return false;
    }

    cv::Mat QRDetector::makeColourMask(const cv::Mat& hsv, QRColour colour) const{
        cv::Mat mask{ cv::Mat::zeros(hsv.size(), CV_8UC1) };
        if (colour == QRColour::NONE){
            mask.setTo(255);
            return mask;
        }

        auto it{ m_colourRanges.find(colour) };
        if (it == m_colourRanges.end()) return mask;

        for (const auto& rng : it->second){
            cv::Mat part{};
            cv::inRange(hsv, rng.lower, rng.upper, part);
            cv::bitwise_or(mask, part, mask);
        }

        cv::Mat kernel{ cv::getStructuringElement(cv::MORPH_RECT, {3,3}) };
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        return mask;
    }
    
    std::vector<cv::Rect> QRDetector::findCandidateROIs(const cv::Mat& mask) const {
        const double scale{ 0.5 };
        cv::Mat smallMask;
        cv::resize(mask, smallMask, cv::Size(), scale, scale, cv::INTER_NEAREST);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(smallMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Rect> rois;
        rois.reserve(contours.size());

        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < static_cast<double>(m_minAreaPx) * scale * scale) continue;

            cv::Rect bb = cv::boundingRect(c);

            // Scale ROI back to full resolution
            cv::Rect fullBB(
                static_cast<int>(bb.x / scale),
                static_cast<int>(bb.y / scale),
                static_cast<int>(bb.width / scale),
                static_cast<int>(bb.height / scale)
            );

            if (!isAspectOk(fullBB, m_aspectRatioLow, m_aspectRatioHigh)) continue;

            rois.push_back(fullBB);
        }

        std::sort(rois.begin(), rois.end(),
                [](const cv::Rect& a, const cv::Rect& b) {
                    return a.area() > b.area();
                });

        return rois;
    }

    DetectResult QRDetector::robustDetectInROI(
        const cv::Mat& frame,
        const cv::Rect& roi,
        bool tryDecode) const {
        DetectResult out{};

        // Pad ROI generously before detection
        cv::Rect padded{ padRect(roi, m_bbboxPadding, frame.size()) };
        padded &= cv::Rect{ 0, 0, frame.cols, frame.rows };
        if (padded.width < 2 || padded.height < 2) return out;

        cv::Mat roiBGR{ frame(padded).clone() };
        if (roiBGR.empty()) return out;

        cv::Mat gray{};
        cv::cvtColor(roiBGR, gray, cv::COLOR_BGR2GRAY);

        cv::QRCodeDetector qrd{};

        auto attempt = [&](const cv::Mat& img, bool decode, DetectResult& result) -> bool {
            if (img.empty() || img.cols < 2 || img.rows < 2) return false;

            std::vector<cv::Point> pts{};
            std::string data{};

            try{
                if (decode) data = qrd.detectAndDecode(img, pts);
                else if (!qrd.detect(img, pts)) return false;
            } catch (const cv::Exception& e){
                std::cerr << "robustDetectInROI detect error: " << e.what() << std::endl;
                return false;
            }

            if (pts.size() == 4) {
                std::vector<cv::Point2f> corners{};
                corners.reserve(4);
                // result.corners.clear();
                for (auto& p : pts) {
                    corners.emplace_back(
                        static_cast<float>(p.x + padded.x),
                        static_cast<float>(p.y + padded.y)
                    );
                }
                if(!polygonIsSane(corners)) return false;

                cv::Rect paddedBox{ bboxFromCorners(corners, 20, frame.size()) };
                if(!minRoiOk(paddedBox, 16)) return false;

                result.corners = std::move(corners);
                result.bbox = paddedBox;

                result.content = decode ? data : std::string{};
                result.found = decode ? (!data.empty()) : true;
                return result.found;
            }
            return false;
        };

        if (attempt(gray, tryDecode, out)) return out;

        cv::Mat invGray{}; cv::bitwise_not(gray, invGray);
        if (attempt(invGray, tryDecode, out)) return out;

        {
            std::vector<cv::Mat> ch{};
            cv::split(roiBGR, ch);
            for (auto& c : ch) {
                cv::Mat inv{};
                cv::bitwise_not(c, inv);
                if (attempt(inv, tryDecode, out)) return out;
            }
        }

        cv::Mat otsu{};
        cv::threshold(gray, otsu, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::Mat invOtsu{}; cv::bitwise_not(otsu, invOtsu);
        if (attempt(invOtsu, tryDecode, out)) return out;

        cv::Mat adap{};
        cv::adaptiveThreshold(gray, adap, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                            cv::THRESH_BINARY, 11, 2);
        cv::Mat invAdap{}; cv::bitwise_not(adap, invAdap);
        if (attempt(invAdap, tryDecode, out)) return out;

        return out;
    }

    cv::Rect QRDetector::padRect(const cv::Rect& r, int pad, const cv::Size& maxSize){
        int x{ std::max(0, r.x - pad) };
        int y{ std::max(0, r.y - pad) };
        int x2{ std::min(maxSize.width, r.x + r.width + pad) };
        int y2{ std::min(maxSize.height, r.y + r.height + pad) };
        return cv::Rect(x, y, x2- x, y2-y );
    }

    bool QRDetector::isAspectOk(const cv::Rect& r, float low, float high){
        float ar{ static_cast<float>(r.width) / std::max(1, r.height) };
        return (ar >= low && ar <= high );        
    }

    float QRDetector::averageSide(const cv::Rect& r){
        return 0.5f * (static_cast<float>(r.width) + static_cast<float>(r.height));
    }

    bool QRDetector::verifyColourInROI(const cv::Mat& frame, const cv::Rect& roi, QRColour colour) const{
        if (colour == QRColour::NONE) return true;
        cv::Mat bgr{ frame(roi) };
        cv::Mat hsv{}; cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

        auto it{ m_colourRanges.find(colour) };
        if (it == m_colourRanges.end()) return false;

        cv::Mat accum{ cv::Mat::zeros(hsv.size(), CV_8UC1) };
        for (const auto& rng : it->second){
            cv::Mat part{};
            cv::inRange(hsv, rng.lower, rng.upper, part);
            cv::bitwise_or(accum, part, accum);
        }

        double ratio{ static_cast<double>(cv::countNonZero(accum)) / (accum.rows * accum.cols + 1e-6) };
        return ratio > 0.25;
    }

    float QRDetector::estimateDistancePxToMeters(float bboxPx, float referencePx, float referenceMeters){
        if (bboxPx <= 1.0f) return std::numeric_limits<float>::infinity();
        return (referencePx / bboxPx) * referenceMeters;
    }

    std::vector<QRCode> QRDetector::detectQRCodes(const cv::Mat& frame, bool tryDecode) {
        std::vector<QRCode> out{};
        if (frame.empty()) return out;

        cv::Mat hsv{}; cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask{ makeColourMask(hsv, m_targetColour) };

        auto rois{ findCandidateROIs(mask) };

        for (const auto& roi : rois) {
            auto det{ robustDetectInROI(frame, roi, tryDecode) };
            if (!det.found) continue;

            cv::Rect box{};
            if(det.corners.size() == 4)
                box = bboxFromCorners(det.corners, m_bbboxPadding, frame.size());
            else{
                box = det.bbox.area() > 0 ? det.bbox : roi;
                box &= cv::Rect(0, 0, frame.cols, frame.rows);
            }

            if(!minRoiOk(box, 16)) continue;
            if (m_colourVerifyEnabled && !verifyColourInROI(frame, box, m_targetColour)) continue;

            QRCode qr{};
            qr.position = cv::Point2f{
                box.x + box.width * 0.5f,
                box.y + box.height * 0.5f
            };
            qr.content = tryDecode ? det.content : std::string{};
            qr.colour = m_targetColour;
            float px{ averageSide(box) };
            qr.distance = estimateDistancePxToMeters(px, m_referencePx, m_referenceMeters);
            qr.bbox = box;
            qr.corners = det.corners;

            out.push_back(qr);
        }

        if (out.empty() && m_targetColour == QRColour::NONE) {
            cv::Rect full{ 0, 0, frame.cols, frame.rows };
            auto det{ robustDetectInROI(frame, full, tryDecode) };
            if (det.found) {
                cv::Rect box{};
                if (det.corners.size() == 4) box = bboxFromCorners(det.corners, m_bbboxPadding, frame.size());
                else{
                    box = det.bbox.area() ? det.bbox : full;
                    box &= cv::Rect(0,0, frame.cols, frame.rows);
                }
                if (minRoiOk(box, 16)){
                    QRCode qr{};
                    qr.position = cv::Point2f{
                        box.x + box.width * 0.5f,
                        box.y + box.height * 0.5f
                    };
                    qr.colour = QRColour::NONE;
                    float px{ averageSide(box) };    
                    qr.distance = estimateDistancePxToMeters(px, m_referencePx, m_referenceMeters);
                    qr.bbox = box;
                    qr.corners = det.corners;
                    qr.content = tryDecode ? det.content : std::string{};
                    out.push_back(qr);
                }
            }
        }
        return out;
    }    

    std::optional<QRCode> QRDetector::findNearestQRCode(const std::vector<QRCode>& codes) const{
        if (codes.empty()) return std::nullopt;
        return *std::min_element(codes.begin(), codes.end(),
               [](const QRCode& a, const QRCode& b){return a.distance < b.distance; });
    }

    NavigationCommand QRDetector::getNavigationToQR(const QRCode& qr, const cv::Size& frameSize) const{
        cv::Point2f center(frameSize.width * 0.5f, frameSize.height * 0.5f);
        cv::Point2f delta{ qr.position - center };

        const float hfovDeg{ 60.0f };
        float normX{ (delta.x) / std::max(1.0f, frameSize.width * 0.5f) };
        float angleDeg{ normX* (hfovDeg * 0.5f) };

        NavigationCommand cmd{};
        cmd.distance = qr.distance;
        cmd.angle = std::fabs(angleDeg);

        const float straightThresh{ 5.0f };
        const float ninetyThresh{ 45.0f };

        if (std::fabs(angleDeg) <= straightThresh){
            cmd.direction = Direction::FORWARD;
            cmd.instruction = "Move forward";
        } else if (angleDeg > 0){
            if (cmd.angle >= ninetyThresh){
                cmd.direction = Direction::TURN_RIGHT_90;
                cmd.instruction = "Turn right about 90 degrees";
            } else {
                cmd.direction = Direction::TURN_RIGHT_45;
                cmd.instruction = "Turn right " + std::to_string(static_cast<int>(cmd.angle)) + " degrees";
            }
        } else {
            if (cmd.angle >= ninetyThresh){
                cmd.direction = Direction::TURN_LEFT_90;
                cmd.instruction = "Turn left about 90 degrees";
            } else { 
                cmd.direction = Direction::TURN_LEFT_45;
                cmd.instruction = "Turn left " + std::to_string(static_cast<int>(cmd.angle)) + " degrees";
            } 
        }
        return cmd;
    }

    void QRDetector::setTargetColour(QRColour colour){ m_targetColour = colour; }
    QRColour QRDetector::getTargetColour() const{ return m_targetColour; }
    void QRDetector::setMinArea(int area) { m_minAreaPx = area; }
    void QRDetector::setAspectRatioTolerance(float low, float high) { m_aspectRatioLow = low; m_aspectRatioHigh = high; }
    void QRDetector::setBoundingBoxPadding(int px){ m_bbboxPadding = px; }
    void QRDetector::setDistanceReference(float pxAt1m, float meters){ m_referencePx = pxAt1m; m_referenceMeters = meters; }
    void QRDetector::setColourVerificationEnabled(bool enabled) { m_colourVerifyEnabled = enabled; }
    bool QRDetector::getColourVerificationEnabled() const { return m_colourVerifyEnabled; }
}