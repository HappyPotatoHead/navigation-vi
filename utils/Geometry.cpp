#include "Geometry.h"

#include <cmath>

namespace NavigationVI{

    float Point::distanceTo(const Point& other) const{
        return std::hypot(m_x - other.m_x, m_y - other.m_y);
    }

    float Point::manhattanDistanceTo(const Point& other) const{
        return std::abs(m_x - other.m_x) + std::abs(m_y - other.m_y);
    }

    bool Rectangle::containsPoint(const Point& p) const{
        return (
            (m_x <= p.m_x <= m_x + m_width) &&
            (m_y <= p.m_y <= m_y + m_height)
        );
    }

    Point Rectangle::center() const{
        return Point{m_x + m_width/2, m_y + m_height/2};
    }

    bool Rectangle::intersects(const Rectangle& o) const{
        return !(
            (m_x + m_width < o.m_x) || (o.m_x + o.m_width < m_x) ||
            (m_y + m_height < m_y) || (m_y + m_height < m_y) 
        );
    }
} 
