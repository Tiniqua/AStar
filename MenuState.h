#pragma once
#include "State.h"
#include "SpriteManager.h"

class MenuState : public State
{
	Text mTitle;
	Text mBody;
	Text mEnteredName;		//Used only in enter name menu
	Panel mPanel;
	std::string menuName;
	std::vector<Button> mButtons;

public:
	MenuState(const Text& t1, const Text& b, const Button& b0, const Button& b1, const Button& b2, const Button& b3, const std::string& menuName);
	MenuState(const Button& b0, const Button& b1, const Button& b2, const Button& b3, const std::string& menuName);
	MenuState(const Text& t1, const Text& t2, const Button& b0, const Panel& p0, const std::string& menuName);		//Enter name menu

	virtual void Initialize() override;
	virtual void Update(const GameTimer& gt) override;
	virtual void Draw(const GameTimer& gt) override;
	virtual void Reset()				override {};

	virtual void OnMouseDown(WPARAM btnState, int x, int y) override {};
	virtual void OnMouseUp(WPARAM btnState, int x, int y)	override {};
	virtual void OnMouseMove(WPARAM btnState, int x, int y) override {};
	virtual void OnKeyboardInput(const GameTimer& gt)		override {};
	virtual void OnResume()									override;
	virtual void OnPause()									override {};
};