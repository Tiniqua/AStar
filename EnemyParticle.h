#pragma once
#include "GameObject.h"

class EnemyParticle : public GameObject
{
public:
	EnemyParticle()		//Constructor
	{
		this->Initialize("particle");
		this->mpInstance->MaterialIndex = 5;
		this->SetPos(DirectX::XMFLOAT3(0.0f, -100.0f, 0.0f));		//Just a starting position thats not in the way
	};

	void Effect(DirectX::XMFLOAT3 enemyPos, const float angle)
	{
		//Rotates the particles around enemies position
		mpInstance->World._41 = enemyPos.x;
		mpInstance->World._42 = enemyPos.y + 0.8f;
		mpInstance->World._43 = enemyPos.z;

		DirectX::XMMATRIX transformation = DirectX::XMMatrixTranslation(xOffset + particleDropPosition, 0.0f, zOffset);
		transformation *= DirectX::XMMatrixScaling(0.4f, 0.4f, 0.4f);
		RotationAroundPoint(transformation, -angle, DirectX::XMFLOAT3(mpInstance->World._41, mpInstance->World._42 - particleDropPosition, mpInstance->World._43));

		//Moves particle diagonally to the ground
		particleDropPosition += particleDropSpeed;

		if (mpInstance->World._42 < 0)		//If particle reaches the floor, reset its position to do the drop again, prevents having more particles in the vector
		{
			particleDropPosition = 0.1f;
		}
	};

	void RemoveEffect()		//Hides effect if enemy attack is not active
	{
		mpInstance->World._42 = -100.0f;
		particleDropPosition = 0.1f;
	};

	float particleDropSpeed = static_cast<float>(rand() % 50) / 1000 + 0.02f;		//Speed of which the particle falls to the floor, is random for each particle
	float particleDropPosition = 0.1f;

	//Position of each particle from the enemy, is random for each particle
	float xOffset = static_cast<float>(rand() % 10) / 10 + 1.0f;
	float zOffset = static_cast<float>(rand() % 10) / 10 - 0.2f;
};