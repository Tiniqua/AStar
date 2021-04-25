#include "StoryState.h"
#include "GameApp.h"

StoryState::StoryState()
{
	
}

void StoryState::Initialize()
{
	mBackground.Initialise("uiTex");
	mForeground.Initialise("uiTex");

	const DirectX::XMINT2 s = GameApp::Get().GetClientSizeU2(); //todo on resize
	const DirectX::SimpleMath::Vector2 v = GameApp::Get().GetClientSize();

	RECT src = { 1,511,1,511 }; // black
	//RECT fgWhite = { 65,511,65,511 };

	mBackground.sourceRectangle = src; // texture rect
	mBackground.destinationRectangle = { 0 ,0,s.x ,s.y };// whole viewport 

	src = { 65,511,65,511 }; // white
	mForeground.sourceRectangle = src;
	const LONG padding = s.y >> 2;
	mForeground.destinationRectangle = { 0 ,0 + padding,s.x ,s.y - padding }; // wide screen 

	// origin in middle of viewport
	mBody.center = true;
	mBody.position = v * 0.5f; 
	mBody.color = DirectX::Colors::Black;
	mBody.fontIndex = 0;
	mBody.string = GameApp::Get().GetStoryText();

	mPrompt.position = {12.0f, v.y - 64.0f};
	mPrompt.color = DirectX::Colors::White;
	mPrompt.fontIndex = 1;
	mPrompt.string = "Press W/A/S/D to continue";
}

void StoryState::Update(const GameTimer & gt)
{
	if (Input::Get().AnyMenuButtonPressed())
	{
		if (mIsIntro)
		{
			mIsIntro = false;
			GameApp::Get().ChangeState(GC::STATE_MAINMENU);
		}
		else
		{
			GameApp::Get().ChangeState(GC::STATE_PLAY);
		}
	}
}

void StoryState::Draw(const GameTimer & gt)
{
	mBackground.Draw();
	mForeground.Draw();
	mBody.Draw();
	mPrompt.Draw();
}

void StoryState::OnResume()
{

	mBody.string = GameApp::Get().GetStoryText();

}

void StoryState::OnPause()
{
}

void StoryState::Reset()
{
}
