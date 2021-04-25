#include "WinState.h"
#include "GameApp.h"
#include <iomanip>

WinState::WinState(const Text& t1, const Text& t2, const Button& btn)
{
	mTitle = t1;
	mDescription = t2;
	mBtn = btn;

	DirectX::XMFLOAT2 centre = GameApp::Get().GetClientSize();
	centre.x *= 0.5f;
	centre.y *= 0.6f;

	mBtn.SetPos({ centre.x, centre.y });
}

void WinState::Initialize()
{

}

void WinState::Update(const GameTimer& gt)
{
	if (Input::Get().AnyMenuButtonPressed())
		GameApp::Get().PlayClickDownAudio();

	if (Input::Get().AnyMenuButtonPressed())
		mBtn.SetColor(GC::BUTTON_DOWN_COLOR);

	if (Input::Get().MenuInputUp())		//Go back to start menu
	{
		GameApp::Get().PlayClickUpAudio(true);
		mBtn.SetColor(GC::BUTTON_UP_COLOR);
		mBtn.Activate();
	}

}

void WinState::Draw(const GameTimer& gt)
{
	mTitle.Draw();
	mDescription.Draw();
	DrawLeaderboard();
}

void WinState::OnResume()
{

}

void WinState::OnPause()
{
	GameApp::Get().GetAudio().Play("menuMusic", nullptr, true);
}

void WinState::DrawLeaderboard()
{
	std::vector<std::string> lines(5);
	std::vector<std::string> names(5);
	std::vector<int> scores(5);
	mLeaderboard.scale = 1.5f;
	mLeaderboard.position = DirectX::SimpleMath::Vector2(220.f, 140.f);
	std::stringstream txt;
	Text leaderboardScores;
	leaderboardScores.scale = 1.5f;
	leaderboardScores.position = DirectX::SimpleMath::Vector2(500.f, 140.f);

	//Get leaderboard from txt file
	std::ifstream fin;
	fin.open("Data/Leaderboard.txt", std::ios::out);
	if (fin.fail())
		assert(fin.fail());
	else
	{
		for (int i = 0; i < lines.size(); i++)		//gets lines from txt file
			fin >> lines.at(i);

		//Separate scores from names for each line
		for (size_t i = 0; i < lines.size(); i++)
		{
			names.at(i) = lines.at(i).substr(0, lines.at(i).find("."));
			scores.at(i) = std::stoi(lines.at(i).substr(lines.at(i).find(".") + 1));
		}

		fin.close();
	}

	txt << "1.    " << names.at(0) << std::setw(8) << "\n\n"
		<< "2.    " << names.at(1) << std::setw(8) << "\n\n"
		<< "3.    " << names.at(2) << std::setw(8) << "\n\n"
		<< "4.    " << names.at(3) << std::setw(8) << "\n\n"
		<< "5.    " << names.at(4) << std::setw(8) << "\n\n";

	leaderboardScores.string = std::to_string(scores.at(0)) + "\n\n" + std::to_string(scores.at(1)) + "\n\n" + std::to_string(scores.at(2)) +
		"\n\n" + std::to_string(scores.at(3)) + "\n\n" + std::to_string(scores.at(4));

	mLeaderboard.string = txt.str().c_str();
	mLeaderboard.Draw();
	leaderboardScores.Draw();
}