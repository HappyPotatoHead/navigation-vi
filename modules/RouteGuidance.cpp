#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <sstream>

#include "CoordinateMapSystem.h"
#include "RouteGuidance.h"
#include "./utils/Geometry.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace NavigationVI {
    std::pair<double, double> RouteGuidance::pointSegmentDistance(
        const Point& p, const Point& a, const Point& b) const {
        double vx{ b.m_x - a.m_x };
        double vy{ b.m_y - a.m_y };
        double wx{ p.m_x - a.m_x };
        double wy{ p.m_y - a.m_y };
        double denom{ vx * vx + vy * vy };
        if (denom < 1e-12) return { a.distanceTo(p), 0.0 };
        double t{ std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / denom)) };
        Point proj{};
        proj.m_x = a.m_x + t * vx;
        proj.m_y = a.m_y + t * vy;
        return { proj.distanceTo(p), t };
    }

    std::string RouteGuidance::sideOfPoint(const Point& p, const Point& a, const Point& b, double eps) const {
        double cross{ (b.m_x - a.m_x) * (p.m_y - a.m_y) - (b.m_y - a.m_y) * (p.m_x - a.m_x) };
        if (cross > eps) return "left";
        if (cross < -eps) return "right";
        return "ahead";
    }

    std::optional<Room> RouteGuidance::roomAtPoint(const Point& p, const CoordinateMapSystem& map, double tol) const {
        for (const auto& kv : map.getRooms()) {
            if (kv.second.m_center.distanceTo(p) <= tol) return kv.second;
        }
        return std::nullopt;
    }

    double RouteGuidance::bearingDeg(const Point& a, const Point& b) const { return std::atan2(b.m_y - a.m_y, b.m_x - a.m_x) * 180.0 / M_PI; }

    std::string RouteGuidance::turnPhrase(std::optional<double> prevBearing, double currBearing) const {
        if (!prevBearing.has_value()) return "Head";

        // Signed smallest difference in degrees, normalized to (−180, 180]
        double diff = currBearing - prevBearing.value();
        diff = std::fmod(diff + 540.0, 360.0) - 180.0; // normalize

        double ad = std::fabs(diff);
        if (ad < 15)   return "Continue straight";
        if (ad < 45)   return diff > 0 ? "Slight left" : "Slight right";
        if (ad < 135)  return diff > 0 ? "Turn left" : "Turn right";
        return "Make a U-turn";
    }


    double RouteGuidance::segmentDistanceM(const Point& a, const Point& b, double unitScale) const { return a.distanceTo(b) * unitScale; }

    double RouteGuidance::estimateStrideFromHeightCm(double height_cm) const { return 0.414 * (height_cm / 100.0); }

    double RouteGuidance::calibrateUnitScaleFromSteps(
        const std::string& aRoom,
        const std::string& bRoom,
        int steps,
        const CoordinateMapSystem& map,
        double stepLengthM) const {
        if (!map.getRooms().count(aRoom) || !map.getRooms().count(bRoom) || steps <= 0) return 1.0;
        double mapUnits{ map.getRooms().at(aRoom).m_center.distanceTo(map.getRooms().at(bRoom).m_center) };
        double real_m{ steps * stepLengthM };
        return real_m / std::max(mapUnits, 1e-9);
    }

    std::pair<std::vector<Instruction>, std::map<std::string, double>>
        RouteGuidance::pathToInstructions(CoordinateMapSystem map,
            const std::string& startRoom,
            const std::string& goalRoom,
            double unitScale,
            double stepLengthM,
            const std::string& mode,
            double landmarkRadius,
            bool anchorEverySegment)
    {

        std::vector<Instruction> instrs{};
        std::map<std::string, double> summary;

        auto result{ map.findShortestPath(startRoom, goalRoom) };
        if (!result.m_found || result.m_path.empty()) {
            instrs.emplace_back("No path found from" + startRoom + " to " + goalRoom + ".");
            summary["found"] = 0.0;
            summary["total_m"] = 0.0;
            summary["total_steps"] = 0.0;
            summary["segment"] = 0.0;
            return { instrs, summary };
        }

        std::vector<Point> pts{ map.stitchWayPoints(result.m_path) };
        if (pts.empty()) {
            instrs.emplace_back("No waypoints for path.");
            summary["found"] = 0.0;
            return { instrs, summary };
        }

        std::vector<Point> dedup{};
        dedup.push_back(pts.front());
        for (size_t i{ 1 }; i < pts.size(); ++i) {
            if (dedup.back().distanceTo(pts[i]) > 1e-6) dedup.push_back(pts[i]);
        }
        pts.swap(dedup);

        double total_m{ 0.0f };
        int total_steps{ 0 };
        std::optional<double> prev_bearing{};

        std::set<std::string> excludeLandmarks(result.m_path.begin(), result.m_path.end());
        std::string startName{ map.getRooms().at(result.m_path.front()).m_name };
        std::string goalName{ map.getRooms().at(result.m_path.back()).m_name };

        instrs.emplace_back("Starting at " + startName + ".");
        if (onMessage) onMessage("Starting at " + startName + ".");

        //std::set<RoomType> includeTypes{ {
        //        //RoomType::CLASSROOM, RoomType::LABORATORY, RoomType::TOILET, RoomType::OFFICE 
        //    }
        //};

        for (size_t i{ 0 }; i + 1 < pts.size(); ++i) {
            const Point& a{ pts[i] };
            const Point& b{ pts[i + 1] };

            double seg_m_if_scaled{ a.distanceTo(b) * unitScale };
            int seg_steps{};
            if (unitScale != 1.0) seg_steps = std::max(1, (int)std::lround(seg_m_if_scaled / std::max(stepLengthM, 1e-6)));
            else seg_steps = std::max(1, (int)std::lround(a.distanceTo(b) / std::max(stepLengthM, 1e-6)));

            double approx_m{ seg_steps * stepLengthM };

            double bearing{ bearingDeg(a, b) };
            std::string action{ turnPhrase(prev_bearing, bearing) };
            prev_bearing = bearing;

            std::string at_phrase{};
            auto b_node{ roomAtPoint(b, map) };
            if (b_node.has_value() && (anchorEverySegment || action != "Continue straight")) at_phrase = " to " + b_node->m_name;

            /*std::optional<std::pair<Room, std::string>> lm{};
            if (i < pts.size() - 2) lm = segmentBestLandmark(a, b, includeTypes, landmarkRadius, excludeLandmarks, map);*/

            /*std::string landmark_phrase{};
            if (lm.has_value()) {
                const Room& lmRoom{ lm->first };
                const std::string& lmSide{ lm->second };

                if (!b_node.has_value() || lmRoom.m_id != b_node->m_id) landmark_phrase = ", passing " + lmRoom.m_name + " on your " + lmSide;
            }*/

            std::string distance_phrase{};
            if (mode == "landmarks") distance_phrase = "";
            else if (mode == "map") {
                std::ostringstream oss{};
                oss << " for about " << (int)std::lround(seg_m_if_scaled) << " meters";
                distance_phrase = oss.str();
            }
            else {
                std::ostringstream oss{};
                oss << " for about " << seg_steps << " steps (~ " << (int)std::lround(approx_m) << " m)";
                distance_phrase = oss.str();
            }

            //std::string text{ action + at_phrase + distance_phrase + landmark_phrase + "." };
            std::string text{ action + at_phrase + distance_phrase + "." };
            instrs.emplace_back(text, approx_m, seg_steps);
            if (onMessage) onMessage(text);
            total_m += approx_m;
            total_steps += seg_steps;
        }

        instrs.emplace_back("Arrive at " + goalName + ".");
        if (onMessage) onMessage("Arrive at " + goalName + ".");
        summary["found"] = 1.0;
        summary["total_m"] = total_m;
        summary["total_steps"] = total_steps;
        summary["segment"] = static_cast<double>(std::max(0, (int)pts.size() - 1));
        summary["unit_scale"] = unitScale;
        summary["step_length_m"] = stepLengthM;

        return { instrs, summary };

    }
}

/*std::optional<std::pair<Room, std::string>> RouteGuidance::segmentBestLandmark(
    const Point& a, const Point& b,
    const std::set<RoomType>& includeTypes,
    double radius,
    const std::set<std::string>& excludeIds,
    const CoordinateMapSystem& map) const {

    std::optional<std::pair<Room, std::string>> best{};
    double best_d{ std::numeric_limits<double>::infinity() };

    for (const auto& kv : map.getRooms()) {
        const auto& rid{ kv.first };
        const auto& r{ kv.second };

        if (excludeIds.count(rid)) continue;
        if (!includeTypes.count(r.m_RoomType)) continue;

        auto pr{ pointSegmentDistance(r.m_center, a, b) };
        double d{ pr.first };

        if (d <= radius && d < best_d) {
            std::string side{ sideOfPoint(r.m_center, a, b) };
            best = std::make_pair(r, side);
            best_d = d;
        }
    }
    return best;
}*/