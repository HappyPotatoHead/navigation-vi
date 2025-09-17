#include "MapEntities.h"

namespace NavigationVI{
    std::string to_string(RoomType type){
        switch (type) {
            case RoomType::CLASSROOM: return "classroom";
            case RoomType::LABORATORY: return "laboratory";
            case RoomType::OFFICE: return "office";
            case RoomType::TOILET: return "toilet";
            case RoomType::STAIRCASE: return "staircase";
            case RoomType::CORRIDOR: return "corridor";
            case RoomType::ENTRANCE: return "entrance";
            default: return {};
        }
    }
}