#include "CombatController.h"
#include "GameApp.h"

void CombatController::Initialize(Player* player, PlayerWeapon* playerWeapon, std::vector<Enemy>* enemies)
{
	assert(player && enemies && playerWeapon);

	mpPlayer = player;
	mpEnemies = enemies;
	mpPlayerWeapon = playerWeapon;
	mpPlayerWeapon->mpInstance->MaterialIndex = 4;
}

void CombatController::Update(const GameTimer& gt)
{
	mpPlayerWeapon->Update(gt);
	isAttacking = CheckIfAttackIsFinished();		//Stops the attack

	if (isAttacking)
	{
		mpPlayerWeapon->mpInstance->World = mpPlayer->mpInstance->World;
		mpPlayerWeapon->SwingWeapon();
	}
	else
	{
		mpPlayerWeapon->SetDirection(mpPlayer->playerDir);		//Gets direction the player is facing, doesn't detect when attacking to prevent sword switching positions

		std::for_each(mpEnemies->begin(), mpEnemies->end(), [&](Enemy& e)
		{
			e.SetDirection(mpPlayer->GetPos());				//Gets direction player is facing into the enemy class to correctly blowback the enemy from the player
		});

	}
}

void CombatController::PlayerAttack()
{
	mpPlayerWeapon->Attack();
}

bool CombatController::CheckIfAttackIsFinished()
{
	return mpPlayerWeapon->GetAttackStatus();
}

void CombatController::EquipWeapon(std::string weaponName)
{
	equippedWeapon = weaponName;		//Sets the equipped weapon
	mpPlayer->attack = mpPlayer->attack + mpPlayerWeapon->GetWeaponStats(weaponName);			//Updates player attack value to base attack + equipped weapons attack amount
}

std::string CombatController::GetCurrentWeapon()
{
	return equippedWeapon;
}

void CombatController::Reset()
{
	isAttacking = false;
	equippedWeapon = "Stick";
}

void PlayerWeapon::Initialize(const std::string& renderItemName)
{
	// Setup a render item
	AddRenderItemInstance("Weapon");

	//float attackDuration = GC::PLAYER_ATTACK_DURATION;
	//float attackDelay = GC::PLAYER_ATTACK_DELAY;

	mCanAttack = false;
}

void PlayerWeapon::Update(const GameTimer& gt)
{
	if (mPlayerAttackTimer.HasTimeElapsed(gt.DeltaTime(), GC::PLAYER_ATTACK_DURATION))
	{
		mCanAttack = true;
	}
}

void PlayerWeapon::Reset()
{
	weaponRotation = weaponEndRotation;
	mCanAttack = false;
	mIsCurrentlySwinging = false;
	SetPos(DirectX::XMFLOAT3(0.f, 0.f, -80.f));
	//When having different models for diff weapons, reset model to first weapon
}

void PlayerWeapon::Attack()
{
	if (mCanAttack)
	{
		PositionWeapon();
		SwingWeapon();
		GameApp::Get().GetAudio().Play("playerAttack", nullptr, false, 1.0f, GC::VOICE_PITCH[rand() % GC::VOICE_PITCH_COUNT]);

	}
}

void PlayerWeapon::PositionWeapon()
{
	mIsCurrentlySwinging = true;
	mCanAttack = false;

	if (playerDirection > 1)		//If the player direction is Up (2 in the enum) or Down (3 in the enum)
	{
		weaponStartingRotation = (GC::WEAPONSTART + 90.0f);		//Sets new starting to rotation to 90 degrees for up/down
		weaponEndRotation = (GC::WEAPONEND + 90.0f);			//Sets new end rotation to 450 degrees
	}
	else if (playerDirection <= 1)				//If player direction is Left (0 in the enum) or Right (1 in the enum)
	{
		weaponStartingRotation = GC::WEAPONSTART;
		weaponEndRotation = GC::WEAPONEND;			//80 degrees
	}

	weaponRotation = weaponStartingRotation;

	UpdateWeaponMatrix();
}

void PlayerWeapon::SwingWeapon()
{
	if (weaponRotation > weaponEndRotation)		//80 degrees, not final until final sword/player models are in
		ResetWeaponPosition();
	else
	{
		weaponRotation += weaponIncrementRotationAmount;
		UpdateWeaponMatrix();		//Updates to show the new rotation
	}
}

void PlayerWeapon::ResetWeaponPosition()
{
	weaponRotation = GC::WEAPONSTART;
	mCanAttack = false;
	mIsCurrentlySwinging = false;

	//Resets weapon underneath the map until used again
	SetPos(XMFLOAT3(mpInstance->World._41, mpInstance->World._42, mpInstance->World._43 - 80.f));
}

void PlayerWeapon::UpdateWeaponMatrix()
{
	XMMATRIX transformation;

	switch (playerDirection)
	{
	case 0:						//Left
		//transformation = XMMatrixRotationX(3.14159f / 2); // check this works 
		transformation = XMMatrixTranslation(-0.5f, 0.0f, 0.0f);
		break;

	case 1:						//Right
		transformation = XMMatrixRotationZ(3.14159f); //Rotate model before starting rotation around point
		//transformation = XMMatrixRotationX(3.14159f / 2); // check this works 
		transformation *= XMMatrixTranslation(0.5f, 0.0f, 0.0f);
		break;

	case 2:						//Up
		transformation = XMMatrixTranslation(-0.5f, 0.0f, 0.0f);
		break;

	case 3:						//Down
		transformation = XMMatrixRotationZ(3.14159f);			//Rotate model before starting rotation around point
		transformation *= XMMatrixTranslation(0.5f, 0.0f, 0.0f);
		break;

	default:
		assert(playerDirection);
		break;
	}

	RotationAroundPoint(transformation, weaponRotation, XMFLOAT3(mpInstance->World._41, mpInstance->World._42 + 1.0f, mpInstance->World._43));
}

bool PlayerWeapon::GetAttackStatus()
{
	return mIsCurrentlySwinging;
}

void PlayerWeapon::SetDirection(int dir)
{
	playerDirection = dir;
}

int PlayerWeapon::GetWeaponStats(std::string equippedWeapon)
{
	int attack = 0;
	if (equippedWeapon == "Stick")
	{
		attack = 10;
		mpInstance->MaterialIndex = 4;		//Visual show of weapons being changed when equipped
		///Find way to change model to something else
	}

	if (equippedWeapon == "Leadpipe")
	{
		attack = 20;
		mpInstance->MaterialIndex = 1;
		//etc.
	}

	if (equippedWeapon == "Nail Bat")
	{
		attack = 40;
	}

	if (equippedWeapon == "Scrap Metal")
	{
		attack = 5;
	}

	return attack;
}