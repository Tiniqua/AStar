#include "TradeState.h"
#include "PlayState.h"
#include "Constants.h"
#include "GameApp.h"

using ButtonState = GamePad::ButtonStateTracker::ButtonState;

TradeState::TradeState()
{
}

void TradeState::Initialize()
{
	const DirectX::XMINT2 s = GameApp::Get().GetClientSizeU2(); //todo on resize

	const long
		top = s.y - (s.y / 2),// 2/3s down
		colTop = top + 64,
		colDivX = s.x / 2; // split down the middle

	const float topStartFloat = (float)top,
		colTopFloat = (float)colTop,
		colDivXFloat = (float)colDivX,
		textPadding = 20.0f;


	mBackground.destinationRectangle = { 0, top, s.x, s.y };
	mBackground.Initialise("iceTex");

	
	mText[TEXT_TRADER].position = { textPadding ,topStartFloat + textPadding }; // top
	mText[TEXT_REQUEST].position = { textPadding , colTopFloat + textPadding }; // left col
	mText[TEXT_REWARD].position = { colDivXFloat + textPadding, colTopFloat + textPadding}; // right col
	
	for (auto& t : mText)
	{
		t.fontIndex = 1;
		t.color = GC::TEXT_LIGHT_COLOR;
	}

	const RECT r{		GC::PANEL_SRC[0],		GC::PANEL_SRC[1],		GC::PANEL_SRC[2],		GC::PANEL_SRC[3],	};

	mPanels.at(PANEL_TOP).Initialize("uiTex", r, { 0, top, s.x, colTop });// top
	mPanels.at(PANEL_LEFT).Initialize("uiTex", r, { 0,colTop,colDivX,s.y });// left col
	mPanels.at(PANEL_RIGHT).Initialize("uiTex", r, { colDivX,colTop,s.x,s.y });// right col

	mHelpMessage.mText.center = true;
	mHelpMessage.mText.color = DirectX::Colors::Red;
	mHelpMessage.mText.position = DirectX::SimpleMath::Vector2{400.f, 240.f };
	mHelpMessage.Activate("D key to accept deal. S key to exit trade", 1.f);
}

void TradeState::Update(const GameTimer & gt)
{

	if (Input::Get().AnyMenuButtonPressed())
	{
		GameApp::Get().PlayClickDownAudio();
	}

	if (Input::Get().MenuInputUp()) // W
	{
	}

	if (Input::Get().MenuInputLeft()) // A
	{
	}

	if (Input::Get().MenuInputRight()) // D
	{
		if (Trade())
		{
			RefreshText(); // update text
			GameApp::Get().PlayClickUpAudio(true);
		}
		else
		{
			GameApp::Get().PlayClickUpAudio(false);
		}
	}

	if (Input::Get().MenuInputDown()) // S
	{
		GameApp::Get().PlayClickUpAudio(true);

		// return to game
		GameApp::Get().ChangeState(GC::STATE_PLAY);
	}
}

void TradeState::Draw(const GameTimer & gt)
{
	mBackground.Draw();

	for (auto& p : mPanels)
	{
		p.Draw();
	}
	for (auto& t : mText)
	{
		t.Draw();
	}

	mHelpMessage.Draw();
}

void TradeState::OnResume()
{
	PlayState* pPlayState((PlayState*)GameApp::Get().GetState(GC::STATE_PLAY));
	assert(pPlayState);

	mpPlayer = pPlayState->GetPlayer();
	mpTrader = pPlayState->GetTrader();
	mpInventory = pPlayState->GetInventory();
	mpInventoryPosition = pPlayState->GetInventoryPos();
	// if any one element is not present then go back to PlayState
	if (!mpTrader || !mpPlayer || !mpInventory)
	{
		assert(false);
		GameApp::Get().ChangeState(GC::STATE_PLAY);
	}
	else
	{
		mPitch = mpTrader->GetPitch();
		RefreshText();

	}

	GameApp::Get().GetAudio().Play("menuMusic", nullptr, true);

}

void TradeState::OnPause()
{
}

void TradeState::CleanInventory(Inventory& inv)
{
	for (Inventory::iterator it = inv.begin(); it != inv.end();)
	{
		if ((*it).second <= 0)
		{
			it = inv.erase(it);
		}
		else 
		{
			++it;
		}
	}
}

bool TradeState::Trade()
{

	Inventory::iterator tempIt = *mpInventoryPosition;
	
	const InventoryUnordered* request = mpTrader->GetRequestItems();

	// copy to manipulate for trade
	Inventory temp = *mpInventory;

	bool validTrader = true;
	bool ret = false;

	// start at top of request
	InventoryUnordered::const_iterator rIt = request->begin();

	// evaluates if trade can be done and removes items
	while (rIt != request->end() && validTrader)
	{
		// request items minused from temp inventory.
		// If item isn't there, it is created. the trade will fail so temp inventory will be discarded
		temp[rIt->first] -= rIt->second;

		// no trade if not enough of an item int the traders request
		if (temp.at(rIt->first) < 0)
		{
			validTrader = false;

		}

		++rIt;
	}

	if (validTrader)
	{
		//add items to temp
		if (mpTrader->GiveRewards(temp))
		{
			CleanInventory(temp); // remove anything that the inventory has nothing of

			__int64 distance = std::distance(mpInventory->begin(), tempIt); // location of current itterator
			__int64 difference = mpInventory->size() - temp.size();

			// save changes to inventory
			(*mpInventory) = std::move(temp);

			// if temp is smaller minus difference
			if (difference > 0)
			{
				distance -= difference;
			}

			// reset iterator since inventory ahs changed
			tempIt = mpInventory->begin();
			// go to location
			std::advance(tempIt, distance);

			(*mpInventoryPosition) = std::move(tempIt);

			ret = true;
		}
	}

	return ret;
}

void TradeState::RefreshText()
{
	
	mText[TEXT_TRADER].string = mpTrader->GetDialog().c_str();

	GameApp::Get().GetAudio().Play(GC::TRADER_TALK[rand()%GC::TRADER_TALK_COUNT],nullptr,false,1.0f,mPitch);

	if (mpTrader->CanTrade())
	{
		// show trade items if can trade
		mText[TEXT_REQUEST].string = mpTrader->GetRequestAsString().c_str();
	}
	else
	{
		//show nothing
		mText[TEXT_REWARD].string = mText[TEXT_REQUEST].string;
	}
}
