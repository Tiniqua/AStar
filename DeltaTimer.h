#pragma once

#include <cmath>
struct DeltaTimer
{
	DeltaTimer(float t = 0.0f) : timer(t) {}
	float timer;
	// counts down and resets when time has elapsed
	bool HasTimeElapsed(float dt, float resetOnElapsed)
	{
		timer -= dt;
		bool elapsed(timer <= 0.0f);
		if (elapsed)
			timer = std::abs(resetOnElapsed);

		return elapsed;
	}

	void Reset(float time)
	{
		timer = time;
	}
};