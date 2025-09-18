#pragma once

#include <string>
#include <vector>
#include <map>
#include <set>
#include <optional>
#include <functional>

#include "../utils/MapEntities.h"
#include "../utils/RouteTypes.h"
#include "CoordinateMapSystem.h"

namespace NavigationVI{
    struct Instruction{
        std::string text{};
        double distance_m{ 0.0f };
        int steps{ 0 };
        Instruction() = default;
        Instruction(const std::string& t, double d = 0.0, int s = 0 )
            : text(t), distance_m(d), steps(s) {}
    };

    class RouteGuidance{
    public:
        RouteGuidance() = default;

        std::pair<std::vector<Instruction>, std::map<std::string, double>>
            pathToInstructions(CoordinateMapSystem map,
                const std::string& startRoom,
                const std::string& goalRoom,
                double unitScale = 1.0,
                double stepLengthM = 0.75,
                const std::string& mode = "step",
                double landmarkRadius = 20.0,
                bool anchorEverySegment = true
            );
        
        double estimateStrideFromHeightCm(double height_cm) const; 
    public:
        std::function<void(const std::string&)> onMessage{};
    
    private:
        std::pair<double, double> pointSegmentDistance(const Point& p, const Point& a, const Point& b) const;
        std::string sideOfPoint(const Point& p, const Point& a, const Point& b, double eps = 1e-6) const;
        std::optional<Room> roomAtPoint(const Point& p, const CoordinateMapSystem& map, double tol = 1e-5) const;
        std::optional<std::pair<Room, std::string>> segmentBestLandmark(
            const Point& a, const Point& b,
            const std::set<RoomType>& includeTypes,
            double radius,
            const std::set<std::string>& excludeIds,
            const CoordinateMapSystem& map) const;
        double bearingDeg(const Point& a, const Point& b) const;
        std::string turnPhrase(std::optional<double> prevBearing, double currBearing) const;
        double segmentDistanceM(const Point& a, const Point& b, double unitScale) const;
        double calibrateUnitScaleFromSteps(const std::string& aRoom, const std::string& bRoom,
            int steps, const CoordinateMapSystem& map, double stepLengthM = 0.75) const;
    private:
        std::map<std::string, Room> rooms{};
    };
}