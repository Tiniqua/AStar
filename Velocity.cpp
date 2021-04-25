#include "Velocity.h"
#include "GameApp.h"


Velocity::Velocity()
	:
	acceleration(1.0f),
	drag(0.9f),
	maxVel(0.001f)
{
	
}


void Velocity::SetVel( const DirectX::SimpleMath::Vector3 & v, float rate)
{

	// limits 

	float len = v.Length();

	// bigger than deadone
	if (len > 0.001f)
	{
		// reached max
		if (len > maxVel)
		{
			velocity.Normalize();
			velocity *= maxVel;
		}
		
		//apply vel
		velocity = DirectX::SimpleMath::Vector3::Lerp(velocity , v * acceleration, rate );

	}
	else
	{
		velocity *= drag*rate;
	}
}

void Velocity::Update(const GameTimer & gt)
{
	//velocity *= drag;
}
