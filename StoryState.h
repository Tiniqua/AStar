#pragma once
#include "State.h"
#include "SpriteManager.h"

class StoryState :	public State
{
	Sprite mBackground;
	Sprite mForeground;
	Text mBody;
	Text mPrompt;

	bool mIsIntro = true; // goes to menu/game
public:
	StoryState();

	virtual void Initialize() override;
	virtual void Update(const GameTimer& gt) override;
	// Draw sprites and fonts onto spritebatch
	virtual void Draw(const GameTimer& gt) override;
	// Called on state change
	virtual void OnResume() override;
	virtual void OnPause() override;
	virtual void Reset() override;


	virtual void OnMouseDown(WPARAM btnState, int x, int y) override {};
	virtual void OnMouseUp(WPARAM btnState, int x, int y) override {};
	virtual void OnMouseMove(WPARAM btnState, int x, int y)override {};
	virtual void OnKeyboardInput(const GameTimer& gt) override {};
};

