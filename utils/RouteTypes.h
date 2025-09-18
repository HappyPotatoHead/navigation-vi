#pragma once

#include <vector>
#include <string>

#include "Geometry.h"

namespace NavigationVI{
    struct PathResult{
        std::vector<std::string> m_path{};
        float m_totalDistance{};
        std::vector<Point> m_wayPoints{};
        bool m_found{};
        float m_executionTime{};
    };
}