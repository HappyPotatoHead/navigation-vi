#pragma once

#include <string>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <stdexcept>

#include "Geometry.h"

namespace NavigationVI{
    enum class RoomType{
        CLASSROOM,
        LABORATORY,
        OFFICE,
        TOILET,
        STAIRCASE,
        CORRIDOR,
        ENTRANCE,
    };

    inline RoomType roomTypeFromString(const std::string& typeStr) {
        static const std::unordered_map<std::string, RoomType> map{ {
            {"CLASSROOM", RoomType::CLASSROOM},
            {"LABORATORY", RoomType::LABORATORY},
            {"OFFICE", RoomType::OFFICE},
            {"TOILET", RoomType::TOILET},
            {"STAIRCASE", RoomType::STAIRCASE},
            {"CORRIDOR", RoomType::CORRIDOR},
            {"ENTRANCE", RoomType::ENTRANCE},
        } };

        auto it{ map.find(typeStr) };
        if (it != map.end()) return it->second;
        throw std::invalid_argument("Unknown RoomType string: " + typeStr);
    }

    struct Connection{
        std::string fromRoom{};
        std::string toRoom{};
        float distance{};
        std::string pathwayType{ "corridor" };
        std::vector<Point> wayPoints{};
        bool isAccessible{ true };
        float width{ 2.0 };
    };

    struct Room{
        std::string m_id{};
        std::string m_name{};
        RoomType m_RoomType{};
        Point m_center{};
        Rectangle m_bounds{};

        std::unordered_set<std::string> m_connections{};
        std::optional<int> m_capacity{};
        std::string m_floor{ "ground" };
        std::optional<std::string> m_description{};
        std::vector<Point> m_access_points{};
        
        void addConnections(const std::string& roomId);
    };
}