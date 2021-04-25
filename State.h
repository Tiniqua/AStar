#pragma once


#include <vector>
#include <memory>
#include <map>
#include <string>
#include "GameObject.h"
#include "SpriteManager.h"

class GameTimer;

// Base class for all states
class State
{
public:
	virtual void Initialize() = 0;
	virtual void Update(const GameTimer& gt) = 0;
	// Draw sprites and fonts onto spritebatch
	virtual void Draw(const GameTimer& gt) = 0; 
	// Called on state change
	virtual void OnResume() = 0;
	virtual void OnPause() = 0;
	virtual void Reset() = 0;


	virtual void OnMouseDown(WPARAM btnState, int x, int y) = 0;
	virtual void OnMouseUp(WPARAM btnState, int x, int y) = 0;
	virtual void OnMouseMove(WPARAM btnState, int x, int y) = 0;
	virtual void OnKeyboardInput(const GameTimer& gt) = 0;
};

class StateManager
{
	enum class Fade { NONE, FADE_IN, FADE_OUT, READY };	//IN to mode, OUT from mode
	Fade mFade = Fade::FADE_IN;
	//Fade mFade;
	// Holds states
	std::unordered_map<std::string, std::unique_ptr<State> > mStates;
	// Key to the state that will update and render
	std::string mCurrentState;
	std::string mQueuedState;

	Sprite mMenuBackground;
	Sprite mFadeForeground;
	float mFadeAlpha = 1.0f; 
	const float mFadeRate = 4.0f;
	//DeltaTimer mFadeTime;

	// For enabling rendering in GameApp
	bool mIsMenu;

	// Menu or 3d
	void EvaluateState();

	// Validate state exists
	bool IsValidState(const std::string stateName);

	// Manages fading and switching of states
	void FadeUpdate(const GameTimer& gt);

public:
	// Adds the game states
	void Init() ;
	void Update(const GameTimer& gt);
	// Draw sprites and fonts onto spritebatch
	void Draw(const GameTimer& gt);
	// Add unique_ptr of derived State class.
	void AddState(const std::string& name, std::unique_ptr<State> newState);
	// Cannot remove active state
	void RemoveState(const std::string& name);
	// Queues the next state and begins fade out/in process
	void ChangeState(const std::string& name);
	// Access data between states
	State* GetState(const std::string& name);
	// For enabling rendering 3d in GameApp of menu bg
	bool IsMenu()
	{
		return mIsMenu;
	}

	static bool Story; // USED FOR STORY STATE

	void OnMouseDown(WPARAM btnState, int x, int y);
	void OnMouseUp(WPARAM btnState, int x, int y);
	void OnMouseMove(WPARAM btnState, int x, int y);
	void OnKeyboardInput(const GameTimer& gt);

};