#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <unordered_set>
#include <optional>
#include <queue>
#include <chrono>
#include <algorithm>
#include <cmath>

#include "../utils/RouteTypes.h"
#include "../utils/RouteInternal.h"
#include "../utils/MapEntities.h"

namespace NavigationVI{
    class CoordinateMapSystem{
        public:
            CoordinateMapSystem(const std::string& buildingName, const std::string& floorName);

            std::unordered_map<std::string, Room> getRooms() const; //new
            void addRoom(const Room& room);
            void addConnection(const Connection& c);
            std::vector<std::string> getNeighbours(const std::string& roomId) const;
            std::optional<Connection> getConnection(const std::string& a, const std::string& b) const;
            float heuristic(const std::string& a, const std::string& b) const;
            float connectionLength(const Connection& conn) const;
            float segmentCost(const Connection& con) const;
            PathResult aStarPathFind(const std::string& startRoom, const std::string& goalRoom);
            PathResult findShortestPath(const std::string& startRoom, const std::string& goalRoom, bool _verbose = false);
            bool loadRoomsFromFile(const std::string& filePath);
            bool loadConnectionsFromFile(const std::string& filePath);
            std::vector<Point> stitchWayPoints(std::vector<std::string> pathIds);
            std::optional<std::string> resolveRoomId(const std::string& indent) const;
        private:
            std::string m_buildingName{};
            std::string m_floorName{};
            std::unordered_map<std::string, Room> m_rooms{};
            std::unordered_map<std::string, std::vector<Connection>> m_connections{};
            int m_pushCounter{0};
    };
}