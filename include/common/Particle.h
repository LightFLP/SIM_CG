#pragma once

#include <gfx/vec2.h>

class Particle
{
public:

	Particle(const Vec2f & ConstructPos);
	virtual ~Particle(void);

	void reset();
	void draw();

	Vec2f m_ConstructPos;
	Vec2f m_Position;
	Vec2f m_Velocity;
	float m_Mass = 0.001;
	Vec2f m_ForceAccum;
};
