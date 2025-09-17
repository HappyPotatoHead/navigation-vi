#include <string>
#include <unordered_map> // for converting dictionaries
#include <vector> // for converting strings
#include <unordered_set> // converting sets
#include <optional>
#include <queue>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <utility> // For std::forward
#include <fstream>
#include <sstream>
#include <iostream>

#include "../utils/RouteTypes.h"
#include "../utils/MapEntities.h"
#include "../utils/RouteInternal.h"

#include "CoordinateMapSystem.h"

namespace NavigationVI{
    CoordinateMapSystem::CoordinateMapSystem(
        const std::string& buildingName, 
        const std::string& floorName)
        : m_buildingName(buildingName)
        , m_floorName(floorName) {}


        std::unordered_map<std::string, Room> CoordinateMapSystem::getRooms() const{
            return m_rooms;
        }

        void CoordinateMapSystem::addRoom(const Room& room) {
            m_rooms.emplace(room.m_id, room);
            m_connections[room.m_id];
        }

        void CoordinateMapSystem::addConnection(const Connection& c){
            m_connections[c.fromRoom].push_back(c);

            Connection rev{
                c.toRoom,
                c.fromRoom,
                c.distance,
                c.pathwayType,
                std::vector<Point>(c.wayPoints.rbegin(), c.wayPoints.rend()),
                c.isAccessible,
                c.width
            };

            m_connections[c.toRoom].push_back(rev);
            m_rooms[c.fromRoom].addConnections(c.toRoom);
            m_rooms[c.toRoom].addConnections(c.fromRoom);
        }

        std::vector<std::string> CoordinateMapSystem::getNeighbours(const std::string& roomId) const{
            std::vector<std::string> neighbours{};
            auto it{ m_connections.find(roomId) };
            if (it != m_connections.end()){
                for (const auto& c : it->second){
                    if (c.isAccessible) neighbours.push_back(c.toRoom);
                }
            }
            return neighbours;
        }

        std::optional<Connection> CoordinateMapSystem::getConnection(const std::string& a, const std::string& b) const {
            auto it{ m_connections.find(a) };
            if (it != m_connections.end()){
                for (const auto& c : it->second){
                    if (c.toRoom == b) return c;
                }
            }
            return std::nullopt;
        }

        float CoordinateMapSystem::heuristic(const std::string& a, const std::string& b) const{
            return m_rooms.at(a).m_center.distanceTo(m_rooms.at(b).m_center);
        }

        float CoordinateMapSystem::connectionLength(const Connection& conn) const{
            if (conn.fromRoom.find("CORRIDOR") != std::string::npos ||
                conn.toRoom.find("CORRIDOR") != std::string::npos) return conn.distance;
            

            std::vector<Point> pts{};
            pts.push_back(m_rooms.at(conn.fromRoom).m_center);
            pts.insert(pts.end(), conn.wayPoints.begin(), conn.wayPoints.end());
            pts.push_back(m_rooms.at(conn.toRoom).m_center);

            float total{ 0.0f };
            for (size_t i{ 0 }; i < pts.size() - 1; ++i) total += pts[i].distanceTo(pts[i+1]);
            return total;
        }

        float CoordinateMapSystem::segmentCost(const Connection& conn) const{
            return connectionLength(conn);
        }

        std::vector<Point> CoordinateMapSystem::stitchWayPoints(std::vector<std::string> pathIds){
            if (pathIds.empty()) return {};

            std::vector<Point> pts{};
            pts.push_back(m_rooms.at(pathIds[0]).m_center);

            for (size_t i{ 0 }; i < pathIds.size() - 1; ++i){
                auto c{ getConnection(pathIds[i], pathIds[i+1]) };
                if (!c) continue;
                pts.insert(pts.end(), c->wayPoints.begin(), c->wayPoints.end());
                pts.push_back(m_rooms.at(pathIds[i+1]).m_center);
            }

            std::vector<Point> cleaned{};
            cleaned.push_back(pts[0]);

            for (size_t i{ 1 }; i < pts.size(); ++i){
                if (cleaned.back().distanceTo(pts[i]) > 0.05f) cleaned.push_back(pts[i]);
            }
            return cleaned;
        }

        PathResult CoordinateMapSystem::aStarPathFind(
            const std::string& startRoom,
            const std::string& goalRoom 
        ){
            auto t0{ std::chrono::high_resolution_clock::now() };
            
            if (m_rooms.find(startRoom) == m_rooms.end() || 
                m_rooms.find(goalRoom) == m_rooms.end()){
                    auto elapsed{ 
                        std::chrono::duration<float>(
                            std::chrono::high_resolution_clock::now() - t0
                        ).count() 
                    };
                return PathResult{{}, 0.0f, {}, false, elapsed};
            }

            if (startRoom == goalRoom){
                auto elapsed{ std::chrono::duration<float>(
                    std::chrono::high_resolution_clock::now() - t0).count() };
                return PathResult{{startRoom}, 0.0f, {m_rooms.at(startRoom).m_center}, true, elapsed};
            }

            std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> openHeap{};
            std::unordered_map<std::string, Node> openMap{};
            std::unordered_set<std::string> closed{};

            float h0{ heuristic(startRoom, goalRoom) };
            openMap[startRoom] = Node{ startRoom, 0.0f, h0, std::nullopt };
            openHeap.push(PQEntry{h0, h0, ++m_pushCounter, startRoom});

            while(!openHeap.empty()){
                std::string uId{ openHeap.top().m_nodeId };
                openHeap.pop();

                if (closed.count(uId)) continue;

                const Node& u{ openMap[uId] };
                closed.insert(uId);

                if (uId == goalRoom){
                    std::vector<std::string> path{};
                    std::optional<std::string> cur{ uId };
                    while (cur.has_value()){
                        path.push_back(cur.value());
                        cur = openMap[cur.value()].getParent();
                        // cur = openMap[cur.value()].m_parent;
                    }
                    std::reverse(path.begin(), path.end());

                    auto elapsed{ 
                        std::chrono::duration<float>(
                            std::chrono::high_resolution_clock::now() - t0).count() 
                        };
                    return PathResult{ path, u.getG(), stitchWayPoints(path), true, elapsed };
                }

                for (const auto& vId : getNeighbours(uId)){
                    if(closed.count(vId)) continue;

                    auto conn{ getConnection(uId, vId) };
                    if (!conn || !conn->isAccessible) continue;

                    float tentativeG{ u.getG() + segmentCost(conn.value()) };

                    auto it{ openMap.find(vId) };
                    if (it == openMap.end() || tentativeG < it->second.getG() - 1e-12f){
                        float h{ heuristic(vId, goalRoom) };
                        openMap[vId] = Node(vId, tentativeG, h, uId);
                        float f{ tentativeG + h };
                        openHeap.push(PQEntry{f, h, ++m_pushCounter, vId});
                    }
                }
            }

            auto elapsed{ 
                std::chrono::duration<float>(
                    std::chrono::high_resolution_clock::now() - t0
                ).count()};
            
            return PathResult{ {}, 0.0f, {}, false, elapsed };
        }

        PathResult CoordinateMapSystem::findShortestPath(const std::string& startRoom, const std::string& goalRoom, bool _verbose){
            auto sId{ resolveRoomId(startRoom) };
            auto gId{ resolveRoomId(goalRoom) };

            if (!sId || !gId) return PathResult{ {}, 0.0f, {}, false, 0.0f };

            return aStarPathFind(sId.value(), gId.value());
        }


        std::optional<std::string> CoordinateMapSystem::resolveRoomId(const std::string& ident) const{
            if (m_rooms.find(ident) != m_rooms.end()) return ident;
            std::string lowerIdent{ ident };
            std::transform(
                lowerIdent.begin(), 
                lowerIdent.end(),
                lowerIdent.begin(),
                ::tolower);
            
            for (const auto& [rId, r] : m_rooms){
                std::string lowerName = r.m_name;
                std::transform(
                    lowerName.begin(), 
                    lowerName.end(), 
                    lowerName.begin(), 
                    ::tolower);
                if (lowerName == lowerIdent) return rId;
            }
            return std::nullopt;
    }

    bool CoordinateMapSystem::loadRoomsFromFile(const std::string& filePath){
        std::ifstream file(filePath);
        if (!file.is_open()) return false;

        std::string line{};

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue; // skip comments

            std::stringstream ss(line);
            std::string id{}, name{}, typeStr{};
            float x{}, y{}, w{}, h{};
            char delim{};

            std::getline(ss, id, '|');
            std::getline(ss, name, '|');
            std::getline(ss, typeStr, '|');

            auto trim{ [](std::string &s) {
                s.erase(s.begin(), std::find_if(s.begin(), s.end(),
                    [](unsigned char ch){return !std::isspace(ch); }));
                s.erase(std::find_if(s.rbegin(), s.rend(),
                    [](unsigned char ch){return !std::isspace(ch); }).base(), s.end());
            } };
            trim(typeStr);
            std::transform(typeStr.begin(), typeStr.end(), typeStr.begin(),
                    [](unsigned char c) {return std::toupper(c); });
        
            ss >> x >> delim >> y >> delim >> w >> delim >> h;

            Room r{};
            r.m_id = id;
            r.m_name = name;
            
            try{
                r.m_RoomType = roomTypeFromString(typeStr);
            }
            catch(const std::invalid_argument& e)
            {
                std::cerr << "Skipping room due to invalid type: " << e.what() << "\n";
                continue;
            }
            
            r.m_bounds = Rectangle{ x, y, w, h };
            r.m_center.m_x = x;
            r.m_center.m_y = y;
            addRoom(r);
        }

        return true;
    }

    bool CoordinateMapSystem::loadConnectionsFromFile(const std::string& filePath){
        std::ifstream file(filePath);
        if (!file.is_open()) return false;
    
        std::string line{};

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;

            std::stringstream ss(line);
            std::string a{}, b{}, type{};

            if (!std::getline(ss, a, '|')) continue;
            if (!std::getline(ss, b, '|')) continue;
            if (!std::getline(ss, type, '|')) continue;

            Point pa{ m_rooms.at(a).m_center };
            Point pb{ m_rooms.at(b).m_center };
            float dist{ pa.distanceTo(pb) };

            Connection c{ a, b, dist, type, {}, true, 0.0f };
            addConnection(c);
        }

        return true;
    }
}