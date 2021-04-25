#include "State.h"
#include <assert.h>
#include "GameApp.h"
#include "PlayState.h"
#include "MenuState.h"
#include "TradeState.h"
#include "GameOverState.h"
#include "WinState.h"
#include "StoryState.h"

bool StateManager::Story = true;

void StateManager::EvaluateState()
{
	if (mCurrentState == GC::STATE_PLAY || mCurrentState == GC::STATE_PAUSE || mCurrentState == GC::STATE_TRADE || mCurrentState == GC::STATE_TUTORIAL)
	{
		mIsMenu = false;
	}
	else // menus
	{
		mIsMenu = true;
	}
}

bool StateManager::IsValidState(const std::string stateName)
{
	return mStates.count(stateName) == 1;
}

void StateManager::FadeUpdate(const GameTimer & gt)
{
	float dt = gt.DeltaTime();
	switch (mFade)
	{
	case Fade::FADE_IN:
		
		mFadeAlpha -= mFadeRate * dt;
		if ((mFadeAlpha <= 0.0f))
		{
			mFade = Fade::NONE;
		}
		; break;
	case Fade::FADE_OUT:
		mFadeAlpha += mFadeRate * dt;
		if ((mFadeAlpha > 1.0f))
		{
			mFade = Fade::READY;
		}; break;
	case Fade::READY:
		
		// Ready to change state
		if (IsValidState(mQueuedState))
		{
			if (mQueuedState != mCurrentState)
			{
				// Last call before pausing previous state
				mStates[mCurrentState]->OnPause(); 
				
				// Promote queued state to current state
				mCurrentState = mQueuedState;
				mQueuedState = "";
				EvaluateState();

				// First call promoted queued state
				mStates[mCurrentState]->OnResume();

				if (mCurrentState == GC::STATE_GAMEOVER || mCurrentState == GC::STATE_WIN)
					GameApp::Get().GetState("PlayState")->Reset();			//Resets playstate after fade has ended so you cant see stuff being reset in the world
			}
			

		}
		else
		{
			assert(false);
		}

		mFade = Fade::FADE_IN;

		
		; break;
	case Fade::NONE:; break;
	default:;
	}
}

void StateManager::Init() // initialised in gameapp
{
	//mMenuBackground.Initialise("iceTex"); // initialize background for ALL menus

	mMenuBackground.Initialise("tileTex"); 
	mFadeForeground.Initialise("uiTex");

	DirectX::XMINT2 s = GameApp::Get().GetClientSizeU2(); //todo on resize
	mMenuBackground.sourceRectangle = { 0,0,s.x,s.y }; // whole viewport tiled

	RECT fgBlack  = { 1,511,1,511 };
	//RECT fgWhite = { 65,511,65,511 };

	mFadeForeground.sourceRectangle = fgBlack; // texture rect
	mFadeForeground.destinationRectangle = { 0 ,0,s.x ,s.y };// whole viewport 

	//set position of text elements of menus
	Text menuTitle, menuBody;
	menuTitle.position = GC::MENU_TITLE_POSITION;
	menuBody.position = GC::MENU_BODY_POSITION;

	// Set up menu buttons
	Sprite buttonBg;
	buttonBg.textureSize.x = buttonBg.textureSize.x >> 1;
	buttonBg.textureSize.y = buttonBg.textureSize.y >> 1;
	buttonBg.sourceRectangle = { 0,0,(LONG)buttonBg.textureSize.x,(LONG)buttonBg.textureSize.y >> 2 };
	buttonBg.destinationRectangle = { -1,-1,-1,-1 };
	buttonBg.Initialise("uiTex",true); // initialise button texture for ALL menus
	

	// Intro and Game story
	AddState("Story", std::make_unique<StoryState>());

	// INTRO SCREEN	
	menuTitle.string = ""; 
	menuBody.string = "It began in the 20th year of the 2nd millenium, on a tuesday, when the plague was \nreleased. The world quickly fell to ruin as it quickly spread & mutated moving between\nhosts, human, animal & plant alike. A few wars & civil uprisings later. And this is the\nworld as we now know it.\nTerrifying I know.\n\n\n                                                                            Press W to play";
	Button btnW(buttonBg, "W - Play", Button::Action::GOTO_MAIN_MENU);
	Button btnA(buttonBg, "A - Play", Button::Action::NO_ACTION);
	Button btnD(buttonBg, "D - Play", Button::Action::NO_ACTION);
	Button btnS(buttonBg, "S - Play", Button::Action::NO_ACTION);
	AddState("Story1", std::make_unique<MenuState>(menuTitle, menuBody, btnW, btnA, btnD, btnS, "Story1"));

	//Enter Name Screen
	Panel textbox;
	const RECT src{ 192, 128, 256, 192 };
	const RECT dst{ 300,	220, 520, 310 };
	textbox.Initialize("uiTex", src, dst);
	menuTitle.string = "Enter Name (8 Characters Max)";
	menuTitle.position = DirectX::SimpleMath::Vector2{ 260.f, 180.f };
	menuBody.string = "Press Enter to Start";
	menuBody.position = DirectX::SimpleMath::Vector2{ 330.f, 350.f };
	Button btnEnter(buttonBg, "Enter Play", Button::GOTO_GAME);
	btnEnter.SetPos({ 200.f, 150.f });
	AddState("EnterName", std::make_unique<MenuState>(menuTitle, menuBody, btnEnter, textbox, "EnterNameMenu"));

	// PASSING LEVEL SCREEN?
	menuTitle.string = "";
	menuBody.string = "And so the story continues...";
	btnW = Button(buttonBg, "W - Play", Button::Action::GOTO_GAME);
	btnA = Button(buttonBg, "A - Play", Button::Action::NO_ACTION);
	btnD = Button(buttonBg, "D - Play", Button::Action::NO_ACTION);
	btnS = Button(buttonBg, "S - Play", Button::Action::NO_ACTION);
	AddState(GC::STATE_NEW_AREA, std::make_unique<MenuState>(menuTitle, menuBody, btnW, btnA, btnD, btnS, GC::STATE_NEW_AREA));

	// Main menu
	menuTitle.string = "Safe Haven";
	menuBody.string = "";
	btnW = Button(buttonBg, "W Play", Button::Action::GOTO_ENTER_NAME_MENU);
	btnA = Button(buttonBg, "A No Tutorial", Button::Action::TUT_OFF);
	btnD = Button(buttonBg, "D Set Volume", Button::Action::GOTO_VOLUME);
	btnS = Button(buttonBg, "S Help", Button::Action::GOTO_HELP);
	AddState(GC::STATE_MAINMENU, std::make_unique<MenuState>(menuTitle, menuBody,btnW, btnA, btnD, btnS, "MainMenu"));

	menuTitle.string = "Volume";
	menuBody.string = "";

	btnW = Button(buttonBg, "W - Volume Up", Button::Action::VOLUME_UP);
	btnA = Button(buttonBg, "A - Main Menu", Button::Action::GOTO_MAIN_MENU);
	btnD = Button(buttonBg, "D - Main Menu", Button::Action::GOTO_MAIN_MENU);
	btnS = Button(buttonBg, "S - Volume Down", Button::Action::VOLUME_DOWN);
	AddState(GC::STATE_VOLUME, std::make_unique<MenuState>(menuTitle, menuBody, btnW, btnA, btnD, btnS, GC::STATE_VOLUME));

	//mMenuBackground.Initialise("iceTex"); // overrides previous set
	//PauseMenu
	menuTitle.string = "Pause";
	menuBody.string = "";
	btnW = Button(buttonBg, "W Resume", Button::Action::GOTO_GAME);
	btnA = Button(buttonBg, "A Resume", Button::Action::GOTO_GAME);
	btnD = Button(buttonBg, "D Main Menu", Button::Action::GOTO_MAIN_MENU);
	btnS = Button(buttonBg, "S Resume", Button::Action::GOTO_GAME);
	AddState(GC::STATE_PAUSE, std::make_unique<MenuState>(menuTitle, menuBody,btnW, btnA, btnD, btnS, GC::STATE_PAUSE));

	menuTitle.string = "Help";
	menuBody.string = "Kill things for loot, trade loot for key items, use items to fix the car";
	btnW = Button(buttonBg, "W Play", Button::Action::GOTO_ENTER_NAME_MENU);
	btnA = Button(buttonBg, "A Set Volume", Button::Action::GOTO_VOLUME);
	btnD = Button(buttonBg, "D Set Volume", Button::Action::GOTO_VOLUME);
	btnS = Button(buttonBg, "S Main Menu", Button::Action::GOTO_MAIN_MENU);
	AddState("HelpMenu", std::make_unique<MenuState>(menuTitle, menuBody, btnW, btnA, btnD, btnS, "HelpMenu"));

	//Tutorial help screens
	menuTitle.string = "Help";
	menuTitle.center = true;
	menuTitle.position = DirectX::SimpleMath::Vector2{400.f, 340.f};
	menuBody.center = true;
	menuBody.position = DirectX::SimpleMath::Vector2{400.f, 400.f};
	Button btnAdvance = Button(buttonBg, "Enter To Resume", Button::Action::GOTO_GAME);
	btnAdvance.SetPos({400.f, 550.f});
	Panel msgPanel;
	const RECT ds = {100, 300, 700, 600};
	msgPanel.Initialize("uiTex", src, ds);
	AddState(GC::STATE_TUTORIAL, std::make_unique<MenuState>(menuTitle, menuBody, btnAdvance, msgPanel, GC::STATE_TUTORIAL));

	// GameState
	AddState(GC::STATE_PLAY, std::make_unique<PlayState>());
	
	//Trade state
	AddState(GC::STATE_TRADE, std::make_unique<TradeState>());

	//Game Over State
	Text GOTitle;
	GOTitle.scale = 2.f;
	GOTitle.position = DirectX::SimpleMath::Vector2(300.f, 200.f);
	GOTitle.string = "Game Over";

	Button btnRestart(buttonBg, "W Back to Main Menu", Button::Action::GOTO_MAIN_MENU);
	AddState(GC::STATE_GAMEOVER, std::make_unique<GameOverState>(GOTitle, btnRestart));

	//Win State
	Text WinMenuTitle;
	WinMenuTitle.scale = 2.f;
	WinMenuTitle.position = DirectX::SimpleMath::Vector2(280.f, 60.f);
	WinMenuTitle.string = "You Escaped";

	Text WinMenuTxt2;
	WinMenuTxt2.scale = 1.f;
	WinMenuTxt2.position = DirectX::SimpleMath::Vector2(300.f, 480.f);
	WinMenuTxt2.string = "Press W to Play Again";

	Button btnRestartfromWin(buttonBg, "", Button::Action::GOTO_MAIN_MENU);
	AddState(GC::STATE_WIN, std::make_unique<WinState>(WinMenuTitle, WinMenuTxt2, btnRestartfromWin));

	// Init all states
	std::for_each(mStates.begin(), mStates.end(), [](auto& s) { s.second->Initialize(); });

	EvaluateState();

}

void StateManager::Update(const GameTimer & gt)
{
	auto test = GameApp::Get().AspectRatio(); 

	mFadeForeground.color.w = mFadeAlpha;

	FadeUpdate(gt);



	if (IsValidState(mCurrentState))
	{
		mStates[mCurrentState]->Update(gt);
	}
	else
	{
		assert(false);
	}

}
void StateManager::Draw(const GameTimer & gt)
{
	if (IsValidState(mCurrentState))
	{
		if (IsMenu())
		{
			mMenuBackground.Draw();
		}

		mStates[mCurrentState]->Draw(gt);

		mFadeForeground.Draw();

	}
	else
	{
		assert(false);
	}
}

void StateManager::AddState(const std::string & name, std::unique_ptr<State> newState)
{
	//Check the a state of the same name doesn't exist
	if (mStates.count(name) == 0)
	{
		//Defaults to first state 
		if (mStates.size() == 0)
			mCurrentState = name;

		//Adds state to map
		mStates[name] = std::move(newState);
	}
	else
	{
		assert(false);
	}
}
void StateManager::RemoveState(const std::string & name)
{
	if (mCurrentState != name &&  IsValidState(name))
	{
		mStates.erase(name);
	}
	else
	{
		assert(false);
	}
}
void StateManager::ChangeState(const std::string & name)
{
	
	// Queues the requested state. Actual change happens in FadeUpdate when mFade == Fade::READY
	if (IsValidState(name))
	{
		if (mQueuedState.length() == 0 && mCurrentState != name && mFade == Fade::NONE)
		{
			mQueuedState = name;
			mFade = Fade::FADE_OUT; //fade to black

		}
	}
	else
	{
		assert(false);
	}
}

State * StateManager::GetState(const std::string & name)
{

	if (IsValidState(name))
	{
		return mStates[name].get();
	}
	else
	{
		// No such state
		assert(false);
		return nullptr;
	}
}

void StateManager::OnMouseDown(WPARAM btnState, int x, int y)
{
	if (IsValidState(mCurrentState))
		mStates[mCurrentState]->OnMouseDown(btnState,x,y);
	else
		assert(false);
}

void StateManager::OnMouseUp(WPARAM btnState, int x, int y)
{
	if (IsValidState(mCurrentState))
		mStates[mCurrentState]->OnMouseUp(btnState, x, y);
	else
		assert(false);
}

void StateManager::OnMouseMove(WPARAM btnState, int x, int y)
{
	if (IsValidState(mCurrentState))
		mStates[mCurrentState]->OnMouseMove(btnState, x, y);
	else
		assert(false);
}

void StateManager::OnKeyboardInput(const GameTimer & gt)
{
	if (IsValidState(mCurrentState))
		mStates[mCurrentState]->OnKeyboardInput(gt);
	else
		assert(false);
}

