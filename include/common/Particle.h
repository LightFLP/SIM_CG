#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2 & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw(bool show_velocity, bool show_force);

	Vec2 m_ConstructPos;

	Vec2 m_Position;
	Vec2 m_Velocity;
	
	Vec2 m_ForceAccum;
	float m_Mass = 0.01;
};
