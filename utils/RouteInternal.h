#pragma once

#include <string>
#include <optional>

namespace NavigationVI{
    class PQEntry{
    public:
        PQEntry(const float f, 
                const float h, 
                const int counter, 
                const std::string& nodeId)
            : m_f(f)
            , m_h(h)
            , m_counter(counter)
            , m_nodeId(nodeId){}
        
        bool operator>(const PQEntry& other) const {
            if (m_f == other.m_f){
                if (m_h == other.m_h) return m_counter > other.m_counter;
                return m_h > other.m_h;
            }
            return m_f > other.m_f;
        }

    public:
        std::string m_nodeId{};
    private: 
        int m_counter{};
        float m_f{};
        float m_h{};
    };

    class Node{
        public:
            Node() : m_roomId(""), m_g(0.0f), m_h(0.0f), m_parent(std::nullopt){}
            Node(const std::string& roomId, 
                 const float g, 
                 const float h, 
                 const std::optional<std::string> parent) 
            : m_roomId (roomId) 
            , m_g (g)
            , m_h (h)
            , m_parent(parent){}

            std::optional<std::string> getParent() const {return m_parent; }
            float getG() const {return m_g;}
            float getH() const {return m_h;}
        private:
            std::string m_roomId{};
            float m_g{};
            float m_h{};
            std::optional<std::string> m_parent{};
    };
}