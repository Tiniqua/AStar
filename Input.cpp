//#include "Input.h"
#include "Common/d3dApp.h"
#include <vector>
#include "GameApp.h"



void Input::Initialize()
{
	mGamepad = std::make_unique<DirectX::GamePad>();
	mKeyboard = std::make_unique<DirectX::Keyboard>();
}

void Input::Update()
{
	//Keyboard state and tracker
	mKeyState = mKeyboard->GetState();
	mKeyTracker.Update(mKeyState); // for down/release trigger

	// Gamepad state and tracker
	mGamePadState = mGamepad->GetState(mGamePadPlayer);
	if (mGamePadState.IsConnected())
	{
		mGamePadTracker.Update(mGamePadState); // for down/release trigger
	}
}


bool Input::MenuInputUp()
{
	return
		KeyReleased(GC::KEY_FW) ||
		GamePad().dpadUp == GamePad::ButtonStateTracker::ButtonState::RELEASED ||
		GamePad().y == GamePad::ButtonStateTracker::ButtonState::RELEASED;
}

bool Input::MenuInputDown()
{
	return 
		KeyReleased(GC::KEY_BK) ||
		GamePad().dpadDown == GamePad::ButtonStateTracker::ButtonState::RELEASED ||
		GamePad().a == GamePad::ButtonStateTracker::ButtonState::RELEASED;
}

bool Input::MenuInputRight()
{
	return 
		KeyReleased(GC::KEY_RT) ||
		GamePad().dpadRight == GamePad::ButtonStateTracker::ButtonState::RELEASED ||
		GamePad().b == GamePad::ButtonStateTracker::ButtonState::RELEASED;
}

bool Input::MenuInputLeft()
{
	return 
		KeyReleased(GC::KEY_LT) ||
		mGamePadTracker.dpadLeft == GamePad::ButtonStateTracker::ButtonState::RELEASED ||
		mGamePadTracker.x == GamePad::ButtonStateTracker::ButtonState::RELEASED;
}


bool Input::MenuLeftHeld()
{
	return
		KeyPressed(GC::KEY_LT) ||
		mGamePadTracker.dpadLeft == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.x == GamePad::ButtonStateTracker::ButtonState::PRESSED;
}

bool Input::MenuUpHeld()
{
	return 
		KeyPressed(GC::KEY_FW) ||
		GamePad().dpadUp == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		GamePad().y == GamePad::ButtonStateTracker::ButtonState::PRESSED;
}

bool Input::MenuRightHeld()
{
	return
		KeyPressed(GC::KEY_RT) ||
		GamePad().dpadRight == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		GamePad().b == GamePad::ButtonStateTracker::ButtonState::PRESSED;
}

bool Input::MenuDownHeld()
{
	return
		KeyPressed(GC::KEY_BK) ||
		GamePad().dpadDown == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		GamePad().a == GamePad::ButtonStateTracker::ButtonState::PRESSED;
}

bool Input::AnyMenuButtonPressed()
{
	return
		KeyPressed(GC::KEY_LT) ||
		KeyPressed(GC::KEY_FW) ||
		KeyPressed(GC::KEY_BK) ||
		KeyPressed(GC::KEY_RT) ||
		mGamePadTracker.dpadDown == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.dpadLeft == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.dpadRight == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.dpadUp == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.y == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.x == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.a == GamePad::ButtonStateTracker::ButtonState::PRESSED ||
		mGamePadTracker.b == GamePad::ButtonStateTracker::ButtonState::PRESSED;
}
