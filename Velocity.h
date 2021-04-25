#pragma once

#include "SimpleMath.h"


class GameTimer;

class Velocity
{
	DirectX::SimpleMath::Vector3 velocity;
	float acceleration;
	float drag;
	float maxVel;
public:
	Velocity();

	void SetVel(const DirectX::SimpleMath::Vector3& v, float rate = 1.0f);
	float GetSpeed()
	{
		return velocity.Length();
	}
	void Update(const GameTimer& gt);
	DirectX::XMFLOAT3& GetVelocity()
	{
		return velocity;
	}

	
};

