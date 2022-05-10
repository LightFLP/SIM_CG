#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw(bool show_velocity, bool show_force);

	Vec2f m_ConstructPos;

	Vec2f m_Position;
	Vec2f m_Velocity;
	
	Vec2f m_ForceAccum;
	float m_Mass = 0.001;
};
