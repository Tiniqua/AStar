#pragma once
#include "State.h"
#include "SpriteManager.h"
#include <vector>
#include "Item.h"
#include <array>
//

class Player;
class Trader;
class Item;


class TradeState : public State
{
	enum{
		TEXT_TRADER=0,TEXT_REWARD,TEXT_REQUEST,TEXT_COUNT,
		PANEL_TOP = 0,PANEL_LEFT,PANEL_RIGHT,PANEL_COUNT
	};
	// from playstate on resume
	Player* mpPlayer = nullptr;
	//from playstate on resume
	Trader* mpTrader = nullptr;

	Inventory* mpInventory = nullptr;
	Inventory::iterator* mpInventoryPosition;

	Sprite mBackground;
	std::array<Text, TEXT_COUNT> mText;
	std::array<Panel, PANEL_COUNT> mPanels;
	float mPitch = 1.0f;
	FadeText mHelpMessage;
	
public:
	TradeState();

	virtual void Initialize()								override;
	virtual void Update(const GameTimer& gt)				override;
	virtual void Draw(const GameTimer& gt)					override;
	virtual void OnResume()									override;
	virtual void OnPause()									override;
	virtual void Reset()									override {};


	virtual void OnMouseDown(WPARAM btnState, int x, int y) override {};
	virtual void OnMouseUp(WPARAM btnState, int x, int y)	override {};
	virtual void OnMouseMove(WPARAM btnState, int x, int y) override {};
	virtual void OnKeyboardInput(const GameTimer& gt)		override {};
	void CleanInventory(Inventory& inv);
	bool Trade();
	// on resume and on trade
	void RefreshText();
};

