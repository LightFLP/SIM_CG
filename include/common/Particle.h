#pragma once

#include <gfx/vec2.h>

class Particle {
public:
    Particle(const Vec2 &ConstructPos);
    Particle(const Vec2 &ConstructPos, const float mass);

    virtual ~Particle(void);

    void reset();

    void draw(bool show_velocity, bool show_force);

    Vec2 m_ConstructPos;

    Vec2 m_Position;
    Vec2 m_Velocity;

    Vec2 m_ForceAccum;
    const float m_Mass;
};
