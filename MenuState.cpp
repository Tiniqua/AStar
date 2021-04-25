#include "MenuState.h"
#include "GameApp.h"
#include "Constants.h"
#include <fstream>

using ButtonState = GamePad::ButtonStateTracker::ButtonState;


MenuState::MenuState(const Text & t, const Text & b, const Button & b0, const Button & b1, const Button & b2, const Button & b3, const std::string& menu)
{
	menuName = menu;
	DirectX::SimpleMath::Vector2 clientSize = GameApp::Get().GetClientSize();
	mTitle = t;
	mBody = b;

	mButtons.reserve(4);

	mButtons.push_back(b0);
	mButtons.push_back(b1);
	mButtons.push_back(b2);
	mButtons.push_back(b3);


	DirectX::XMFLOAT2 relButtonPivot = clientSize * GC::MENU_BUTTON_PIVOT;

	mButtons.at(0).SetPos({ relButtonPivot.x								,	relButtonPivot.y - GC::MENU_BUTTON_DISTANCE.y });
	mButtons.at(1).SetPos({ relButtonPivot.x - GC::MENU_BUTTON_DISTANCE.x	,	relButtonPivot.y });
	mButtons.at(2).SetPos({ relButtonPivot.x + GC::MENU_BUTTON_DISTANCE.x	,	relButtonPivot.y });
	mButtons.at(3).SetPos({ relButtonPivot.x								,	relButtonPivot.y + GC::MENU_BUTTON_DISTANCE.y });
}

MenuState::MenuState(const Button & b0, const Button & b1, const Button & b2, const Button & b3, const std::string& menu)
{
	menuName = menu;
	mButtons.reserve(4);

	mButtons.push_back(b0);
	mButtons.push_back(b1);
	mButtons.push_back(b2);
	mButtons.push_back(b3);

	DirectX::XMFLOAT2 centre = GameApp::Get().GetClientSize();
	centre.x *= 0.5f;
	centre.y *= 0.6f;

	const DirectX::XMFLOAT2 offset = { 200.0f , 100.0f };

	mButtons.at(0).SetPos({ centre.x			,	centre.y - offset.y });
	mButtons.at(1).SetPos({ centre.x - offset.x	,	centre.y });
	mButtons.at(2).SetPos({ centre.x + offset.x	,	centre.y });
	mButtons.at(3).SetPos({ centre.x			,	centre.y + offset.y });
}

MenuState::MenuState(const Text& t1, const Text& t2, const Button& b0, const Panel& p0, const std::string& menu)
{
	mTitle = t1;
	mBody = t2;
	menuName = menu;
	mPanel = p0;

	mButtons.reserve(1);
	mButtons.push_back(b0);

	if (menuName == "EnterNameMenu")
	{
		mEnteredName.position = DirectX::SimpleMath::Vector2{ 320.f, 250.f };
		mEnteredName.color = DirectX::Colors::White;
		mEnteredName.scale = 1.5f;
	}
}

void MenuState::Initialize()
{

}

void MenuState::Update(const GameTimer & gt)
{
	if (menuName == GC::STATE_TUTORIAL)
	{
		if (Input::Get().AnyMenuButtonPressed())
		{
			GameApp::Get().PlayClickDownAudio();
		}

		if (Input::Get().KeyReleased(GC::KEY_INPUT_NAME))
		{
			GameApp::Get().PlayClickUpAudio(true);

			mButtons.at(0).Activate();
		}
	}

	else if (menuName == "EnterNameMenu")
	{
		if (Input::Get().KeyReleased(GC::KEY_INPUT_NAME) && mEnteredName.string.size() > 0)	// Enter, used to enter name and continue to next screen
		{
			GameApp::Get().PlayClickUpAudio(true);

			assert(mEnteredName.string.size() != 0);
			std::ifstream fin;
			fin.open("Data/Leaderboard.txt", std::ios::out);

			if (fin.fail())		//Cannot find the txt file / It doesn't exist
			{
				std::ofstream fout("Data/Leaderboard.txt");		//Create the file
				if (fout.fail())
					assert(fout.fail());
				else
				{
					fout << mEnteredName.string;		//Only add a single score into the new txt file
					fout.close();
				}
			}
			else
			{
				std::ofstream fout("Data/Leaderboard.txt", std::ios::app);
				if (fout.fail())
					assert(fout.fail());
				else
				{
					fout << "\n" << mEnteredName.string;
					fout.close();
				}
			}

			mButtons.at(0).Activate();
		}

		//Gets inputted key from the keyboard
		const char key = Input::Get().GetKey();

		if (key == 0x8)		//Backspace
		{
			if (mEnteredName.string.size() > 0)
				mEnteredName.string.pop_back();
		}
		else if (key != '\0' && mEnteredName.string.size() < 8)		//Keeps size of name to 8 max
			mEnteredName.string += key;
	}

	else
	{
		assert(mButtons.size() == 4);

		if (Input::Get().AnyMenuButtonPressed())
		{
			GameApp::Get().PlayClickDownAudio();
		}

		//down
		if (Input::Get().MenuUpHeld()) mButtons.at(0).SetColor(GC::BUTTON_DOWN_COLOR);
		if (Input::Get().MenuLeftHeld()) mButtons.at(1).SetColor(GC::BUTTON_DOWN_COLOR);
		if (Input::Get().MenuRightHeld())mButtons.at(2).SetColor(GC::BUTTON_DOWN_COLOR);
		if (Input::Get().MenuDownHeld())mButtons.at(3).SetColor(GC::BUTTON_DOWN_COLOR);

		//Release
		if (Input::Get().MenuInputUp()) // W
		{
			GameApp::Get().PlayClickUpAudio(true);
			mButtons.at(3).SetColor(GC::BUTTON_UP_COLOR);
			mButtons.at(0).Activate();
		}

		if (Input::Get().MenuInputLeft()) // A
		{
			GameApp::Get().PlayClickUpAudio(true);
			mButtons.at(3).SetColor(GC::BUTTON_UP_COLOR);
			mButtons.at(1).Activate();
		}

		if (Input::Get().MenuInputRight()) // D
		{
			GameApp::Get().PlayClickUpAudio(true);
			mButtons.at(3).SetColor(GC::BUTTON_UP_COLOR);
			mButtons.at(2).Activate();
		}

		if (Input::Get().MenuInputDown()) // S
		{
			GameApp::Get().PlayClickUpAudio(true);
			mButtons.at(3).SetColor(GC::BUTTON_UP_COLOR);
			mButtons.at(3).Activate();
		}
	}
}

void MenuState::Draw(const GameTimer & gt)
{
	if (menuName == GC::STATE_TUTORIAL)
		mPanel.Draw();

	mTitle.Draw();
	mBody.Draw();

	//mTempPanel.Draw();

	if (menuName == "EnterNameMenu")
	{
		mPanel.Draw();
		mEnteredName.Draw();
	}
	
	for (auto& b : mButtons)
	{
		if (menuName != "Story1" && menuName != "EnterNameMenu") // story screen code
			b.Draw();
	}
}

void MenuState::OnResume()
{
	if (menuName == GC::STATE_TUTORIAL)
	{
		mBody.string = GameApp::Get().GetTutorialText();
		GameApp::Get().menusShown++;
	}
	else
		GameApp::Get().GetAudio().Play("menuMusic", nullptr, true);
}
