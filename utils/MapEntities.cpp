#include "MapEntities.h"

#include <string>
#include <unordered_set>

namespace NavigationVI{
    void Room::addConnections(const std::string& roomId){
        m_connections.insert(roomId);
    }
}