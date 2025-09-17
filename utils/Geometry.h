#pragma once

namespace NavigationVI{
    struct Point{
        
        float distanceTo(const Point& other) const;
        float manhattanDistanceTo(const Point& other) const;

        float m_x{};
        float m_y{};
    };

    struct Rectangle{
        bool containsPoint(const Point& p) const;
        Point center() const;
        bool intersects(const Rectangle& o) const;
        float m_x{}, 
              m_y{}, 
              m_width{}, 
              m_height{};
    };
}