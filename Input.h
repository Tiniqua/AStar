#pragma once

#include "Singleton.h"
#include <Keyboard.h>
#include <GamePad.h>
#include "SimpleMath.h"


class Input : public Singleton<Input>
{


private:
	std::unique_ptr<DirectX::Keyboard> mKeyboard;
	DirectX::Keyboard::State mKeyState;
	DirectX::Keyboard::KeyboardStateTracker mKeyTracker;

	std::unique_ptr<DirectX::GamePad> mGamepad;
	DirectX::GamePad::State mGamePadState;
	DirectX::GamePad::ButtonStateTracker mGamePadTracker; 
	int mGamePadPlayer = 0; // 1 - 4

	bool useGamePad = false;

public:
	// Single frame key is pressed
	bool KeyPressed(unsigned char c)
	{
		return mKeyTracker.IsKeyPressed((DirectX::Keyboard::Keys)  c);
	}

	// Single frame key is released
	bool KeyReleased(unsigned char c)
	{
		return mKeyTracker.IsKeyReleased((DirectX::Keyboard::Keys)  c);
	}

	// true while held
	bool KeyHeld(unsigned char c)
	{
		return mKeyState.IsKeyDown((DirectX::Keyboard::Keys)  c);
	}

	char GetKey()
	{
		char c;
		for (c = 0x41; c <= 0x5a; c++)		//Searches through A to Z
		{
			if (mKeyTracker.IsKeyPressed((DirectX::Keyboard::Keys) c))
				return c;
		}

		if (mKeyTracker.IsKeyPressed((DirectX::Keyboard::Keys) 0x20))		//Checks for space bar
			return 0x20;

		if (mKeyTracker.IsKeyPressed((DirectX::Keyboard::Keys) 0x8))		//Checks for backspace
			return 0x8;

		return 0;
	}

	// Use to retrieve trigger and stick floats
	DirectX::GamePad::State GamePadState()
	{
		return mGamepad->Get().GetState(mGamePadPlayer);
	}

	DirectX::GamePad::ButtonStateTracker GamePad()
	{
		return mGamePadTracker;
	}

	// Place after PDO is built
	void Initialize();

	void Update();

	bool GamePadConnected()
	{
		return mGamePadState.IsConnected();
	}

	// The motor values range from 0 to 1. Returns true if the vibration was successfully set
	bool SetVibration(float leftMotor, float rightMotor, float leftTrigger = 0.0f, float rightTrigger = 0.0f)
	{
		return mGamepad->SetVibration(mGamePadPlayer, leftMotor, rightMotor, leftTrigger, rightTrigger);
	}

	// Normalised between -1 to 1
	DirectX::XMFLOAT2 LeftStickF2()
	{
		return	{ mGamePadState.thumbSticks.leftX,	mGamePadState.thumbSticks.leftY };
	}

	// Normalised between -1 to 1
	DirectX::XMFLOAT2 RightStickF2()
	{
		return		{ mGamePadState.thumbSticks.leftX,	mGamePadState.thumbSticks.leftY };
	}

	DirectX::XMFLOAT2 LeftStick()
	{
		return { mGamePadState.thumbSticks.rightX,	mGamePadState.thumbSticks.rightY };
	}

	DirectX::SimpleMath::Vector3 LeftStickXZ()
	{
		return { mGamePadState.thumbSticks.leftX,0.0f,	mGamePadState.thumbSticks.leftY };
	}


	DirectX::XMFLOAT2 RightStick()
	{
		return { mGamePadState.thumbSticks.rightX,	mGamePadState.thumbSticks.rightY };
	}

	DirectX::SimpleMath::Vector3 RightStickXZ()
	{
		return { mGamePadState.thumbSticks.rightX,0.0f,	mGamePadState.thumbSticks.rightY };
	}

	float LeftTrigger()
	{
		return mGamePadState.triggers.left;
	}

	float RightTrigger()
	{
		return mGamePadState.triggers.right;

	}
	bool MenuInputUp();
	bool MenuInputDown();
	bool MenuInputRight();
	bool MenuInputLeft();

	bool MenuLeftHeld();
	bool MenuUpHeld();
	bool MenuRightHeld();
	bool MenuDownHeld();

	// the moment its pressed
	bool AnyMenuButtonPressed();

};