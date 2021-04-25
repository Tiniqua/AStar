#include "GameOverState.h"
#include "GameApp.h"

GameOverState::GameOverState(const Text& t1, const Button& b1)
{
	mTitle = t1;
	mBtn1 = b1;

	DirectX::XMFLOAT2 centre = GameApp::Get().GetClientSize();
	centre.x *= 0.5f;
	centre.y *= 0.6f;

	mBtn1.SetPos({ centre.x, centre.y });
}

void GameOverState::Initialize()
{

}

void GameOverState::Update(const GameTimer& gt)
{
	if (Input::Get().AnyMenuButtonPressed())
	{
		GameApp::Get().PlayClickDownAudio();
	}

	if (Input::Get().MenuUpHeld())
		mBtn1.SetColor(GC::BUTTON_DOWN_COLOR);

	if (Input::Get().MenuInputUp()) // W		///Change key in the future
	{
		GameApp::Get().PlayClickUpAudio(true);
		mBtn1.SetColor(GC::BUTTON_UP_COLOR);
		mBtn1.Activate();
	}

}

void GameOverState::Draw(const GameTimer& gt)
{
	mTitle.Draw();
	mBtn1.Draw();
}

void GameOverState::OnResume()
{
	GameApp::Get().GetAudio().Play("menuMusic", nullptr, true);
}

void GameOverState::OnPause()
{
}
