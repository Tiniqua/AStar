#include "XmfloatOverload.h"
#include "PlayState.h"
#include "GameApp.h"
#include "XmfloatOverload.h"
#include "SimpleMath.h"
//#include "Input.h"
#include "Constants.h"
#include <unordered_map>
#include <algorithm>
#include <iomanip>

using ButtonState = GamePad::ButtonStateTracker::ButtonState;

std::vector<Building> PlayState::GetBuildings()
{
	return mBuildings;
}

std::vector<Enemy> PlayState::GetEnemies()
{
	return mEnemies;
}

Player PlayState::GetPlayerPos()
{
	return mPlayer;
}

void PlayState::InitializeBuildings()
{
	mBuildings.reserve(3);

	mBuildings.push_back(Building(GC::BUILDING_0));
	mBuildings.push_back(Building(GC::BUILDING_1));
	mBuildings.push_back(Building(GC::BUILDING_2));
	
	
	for (auto& b : mBuildings)
	{
		b.Initialize(GC::GO_BUILDING);
		b.mpInstance->MaterialIndex = 0;
	};
}

bool PlayState::FindNearestBuildingInRadius()
{
	struct SortByDistance
	{
		SimpleMath::Vector3 from;

		bool operator()(const Building& a, const Building& b)
		{
			distanceA = SimpleMath::Vector3::Distance(from, a.GetPosV3());
			distanceB = SimpleMath::Vector3::Distance(from, b.GetPosV3());

			return distanceA < distanceB;
		}
	private:
		float distanceA, distanceB;
	};

	SortByDistance s;
	// distance from player location
	s.from = mPlayer.GetPosV3();

	std::sort(mBuildings.begin(), mBuildings.end(), std::ref(s));

	float distance = SimpleMath::Vector3::Distance(mPlayer.GetPosV3(), mBuildings.front().GetPosV3());

	if (distance < GC::BUILDING_RADIUS)
	{
		mpActiveBuilding = &mBuildings.front();
		return true;
	}
	else
	{
		// if none in range clear any active building
		mpActiveBuilding = nullptr;
		return false;
	}
}

void PlayState::InitializeTraders()
{

	mTraders.reserve(10);

	mTraders.push_back(Trader(GC::TRADER_NAME_TEST, GC::TRADER_NAME_TEST, GC::TRADER_NAME_TEST));
	mTraders.push_back(Trader(GC::TRADER_NAME_1, GC::TRADER_NAME_1, GC::TRADER_NAME_1));
	mTraders.push_back(Trader(GC::TRADER_NAME_2, GC::TRADER_NAME_2, GC::TRADER_NAME_2));
	mTraders.push_back(Trader(GC::TRADER_NAME_3, GC::TRADER_NAME_3, GC::TRADER_NAME_3));
	mTraders.push_back(Trader(GC::TRADER_NAME_1, GC::TRADER_NAME_2, GC::TRADER_NAME_3));

	//sets up a line of traders
	for (auto& t : mTraders)
	{
		t.Initialize(GC::GO_TRADER);
		t.mpInstance->MaterialIndex = 0;
	}

	// Side doors
	mTraders.push_back(Trader(GC::TRADER_OBJ_1, GC::TRADER_OBJ_1, GC::TRADER_OBJ_1));
	mTraders.back().Initialize("CarBody");
	mTraders.back().SetPos({ 0.0f, 0.0f, 1.5f });
	mTraders.back().SetAsStoryItem();

	//Rear of car
	mTraders.push_back(Trader(GC::TRADER_OBJ_2, GC::TRADER_OBJ_2, GC::TRADER_OBJ_2));
	mTraders.back().Initialize("CarTire");
	mTraders.back().SetPos({ -1.0f, 0.0f, 1.5f });
	mTraders.back().SetAsStoryItem();

	//Front of car - engine
	mTraders.push_back(Trader(GC::TRADER_OBJ_3, GC::TRADER_OBJ_3, GC::TRADER_OBJ_3));
	mTraders.back().Initialize("CarTire");
	mTraders.back().SetPos({ 1.0f, 0.0f, 1.5f });
	mTraders.back().SetAsStoryItem();

}

bool PlayState::FindNearestTraderInRadius()
{
	struct SortByDistance
	{
		SimpleMath::Vector3 from;

		bool operator()(const Trader& a, const Trader& b)
		{
			distanceA = SimpleMath::Vector3::Distance(from, a.GetPosV3());
			distanceB = SimpleMath::Vector3::Distance(from, b.GetPosV3());

			return distanceA < distanceB;
		}
	private:
		float distanceA = 0.0f, distanceB = 0.0f;
	};

	SortByDistance s;
	// distance from player location
	s.from = mPlayer.GetPosV3();

	std::sort(mTraders.begin(), mTraders.end(), std::ref(s));

	float distance = SimpleMath::Vector3::Distance(mPlayer.GetPosV3(), mTraders.front().GetPosV3());

	if (distance < GC::TRADER_RADIUS)
	{
		mpActiveTrader = &mTraders.front();
		return true;
	}
	else
	{
		// if none in range clear any active trader
		mpActiveTrader = nullptr;
		return false;
	}


}

Enemy PlayState::Spawn(std::string enemyType) // USED TO INITIALISE ENEMIES
{
	//Enemy e(GC::ENEMY_TYPE_1, GC::ENEMYTYPE1_ATTACK);
	if (enemyType == GC::ENEMY_TYPE_1)
	{
		Enemy e(GC::ENEMY_TYPE_1, GC::ENEMYTYPE1_ATTACK);
		e.Initialize("EnemyGhoul");
		e.SetHealth(GC::ENEMYTYPE1_HEALTH);
		e.SetRandomPosition();
		e.particles.resize(20);
		return e;
	}
	else if (enemyType == GC::ENEMY_TYPE_2)
	{
		Enemy e(GC::ENEMY_TYPE_2, GC::ENEMYTYPE2_ATTACK);
		e.Initialize(GC::GO_ENEMY);
		e.SetHealth(GC::ENEMYTYPE2_HEALTH);
		e.SetRandomPosition();
		return e;
	}
	else {
		assert(enemyType != GC::ENEMY_TYPE_1 && enemyType != GC::ENEMY_TYPE_2);
		return Enemy(GC::ENEMY_TYPE_1, 15);		//Default return parameter
	}
}

PlayState::PlayState()
	:
	mExperience(0, GC::EXP_EXPONENT, GC::EXP_OFFSET, 0)
{
}

void PlayState::Initialize()
{
	GameApp::Get().SetActiveCamera(&mCameras.at(CAMERA_TYPE::GAME));

	mTileManager.Initialize();

	timeSet();

	mPlayer.Initialize("Player");
	mPlayerWeapon.Initialize("Weapon");

	mPlants.reserve(20);

	// ui bar
	mPlayerHealthBar.Initialise(GC::BAR_GRN, GC::BAR_RED);

	InitializeTraders();
	InitializeBuildings();


	

	// Setup temp enemies
	{
		mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2)); // number of enemies, Enemy(GC::enemytype, attack)
		mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));

	}

	mCombatController.Initialize(&mPlayer, &mPlayerWeapon, &mEnemies);


	// Sprites
	{
		const DirectX::XMINT2 s = GameApp::Get().GetClientSizeU2(); //todo on resize
		SimpleMath::Vector2 v = GameApp::Get().GetClientSize();

		mScoreText.color = DirectX::Colors::White;
		mScoreText.fontIndex = 1;
		mScoreText.string = "Score: " + std::to_string(score);
		mScoreText.position = DirectX::SimpleMath::Vector2(650.f, 20.f);

		mScoreTextShadow.color = DirectX::Colors::Black;
		mScoreTextShadow.fontIndex = 1;
		mScoreTextShadow.string = "Score: " + std::to_string(score);
		mScoreTextShadow.position = DirectX::SimpleMath::Vector2(651.f, 21.f);

		// panel soruce
		const RECT src{ GC::PANEL_SRC[0],	GC::PANEL_SRC[1],	GC::PANEL_SRC[2],	GC::PANEL_SRC[3], };
		// inv panel
		{
			const RECT dst
			{
				s.x,							//  starts off viewport to the right
				10,								// padding
				s.x + (s.x / 3) + 128,			//  third of viewport plus offset
				s.y - 10						// padding
			};

			mInventoryPanel.Initialize("uiTex", src, dst);
			mInventoryText.color = DirectX::Colors::White;
			mInventoryText.fontIndex = 1;
		}

		//Help panel
		{
			const RECT dst
			{
				10,							//  starts off viewport to the right
				s.y,								// padding
				s.x - 10	,					// padding
				s.y + (s.y / 3) + 128			//  third of viewport plus offset
			};
			mHelpPanel.Initialize("uiTex", src, dst);
			mHelpText.color = DirectX::Colors::White;
			mHelpText.fontIndex = 1;
			mHelpText.string = "(W)(A)(S)(D) - Movement \n(Spacebar) - attack \n(Enter) - Opens/Closes inventory \n(P) - Pauses the game \n(Q) - Trades when near a trader \n(E) - Use an item when in inventory";
		}

		//Help text
		{
			mHelpMessage.mText.center = true;
			mHelpMessage.mText.position = DirectX::SimpleMath::Vector2{ 40.f, 550.f };
			mHelpMessage.mText.color = DirectX::Colors::Red;
			mHelpMessage.Activate(GC::HELP_MESSAGES[2], 6.0f);

			mTradeHelpMessage.mText.center = true;
			mTradeHelpMessage.mText.position = DirectX::SimpleMath::Vector2{ 250.f, 150.f };
			mTradeHelpMessage.mText.color = DirectX::Colors::Red;
		}
	}

	//Instantiates enemies, inventory, trader positions
	Reset();

	// needed in init for dirty frame
	for (auto& c : mCameras)
	{
		c.SetPosition(0.0f, 25.0f, 0.0f);
		c.Pitch(XMConvertToRadians(CAM_ANGLE)); // SETS CAMERA TO FACE DOWN
		c.UpdateViewMatrix();
	}

}

void PlayState::eGen(bool fill) { // fill = true is for pitch respawning
	// use time cycle to determine number of enemies to add, enemy types are selected at random

	if (fill == false) {
		mEnemies.clear();
	}

	if (timeCycle == 1) { // dawn (small / 2)
		//	// inserts n of enemies
		int e1 = (rand() % 2) + 1; // random 1-2
		int e2 = (rand() % 2) + 1; // random 1-2
		if (e1 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e1 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		if (e2 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e2 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		//Init all enemies
		for (auto& e : mEnemies)
		{
			e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
				});
			for (auto& t : mTraders)									//Check each trader in the game
				while (e.CheckCollision(e.GetPos(), t.GetPos()))	//Prevents enemies from spawning inside a trader
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
			for (auto& b : mBuildings)									
				while (e.CheckCollision(e.GetPos(), b.GetPos()))	
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						1.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
		}
	}
	if (timeCycle == 2) { // noon (small / 2)
		//	// inserts n of enemies
		int e1 = (rand() % 2) + 1; // random 1-2
		int e2 = (rand() % 2) + 1; // random 1-2
		if (e1 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e1 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		if (e2 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e2 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		//Init all enemies
		for (auto& e : mEnemies)
		{
			e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
				});
			for (auto& t : mTraders)									//Check each trader in the game
				while (e.CheckCollision(e.GetPos(), t.GetPos()))	//Prevents enemies from spawning inside a trader
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						1.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
			for (auto& b : mBuildings)
				while (e.CheckCollision(e.GetPos(), b.GetPos()))
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						1.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
		}
	}
	if (timeCycle == 3) { // evening (large / 3)
			// inserts n of enemies
		int e1 = (rand() % 2) + 1; // random 1-2
		int e2 = (rand() % 2) + 1; // random 1-2
		int e3 = (rand() % 2) + 1; // random 1-2
		if (e1 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e1 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		if (e2 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e2 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		if (e3 == 1) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
		}
		else if (e3 == 2) {
			mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
		}

		//Init all enemies
		for (auto& e : mEnemies)
		{
			e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
			});

			for (auto& t : mTraders)									//Check each trader in the game
				while (e.CheckCollision(e.GetPos(), t.GetPos()))	//Prevents enemies from spawning inside a trader
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
			for (auto& b : mBuildings)
				while (e.CheckCollision(e.GetPos(), b.GetPos()))
					e.SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						1.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
		}
	}
	if (timeCycle == 4) { // pitch (large / constant)
		// fill on basic
		if (fill == false) {
			int e1 = (rand() % 2) + 1; // random 1-2
			int e2 = (rand() % 2) + 1; // random 1-2
			int e3 = (rand() % 2) + 1; // random 1-2
			if (e1 == 1) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
			}
			else if (e1 == 2) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
			}

			if (e2 == 1) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
			}
			else if (e2 == 2) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
			}

			if (e3 == 1) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
			}
			else if (e3 == 2) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
			}

			//Init all enemies
			for (auto& e : mEnemies)
			{
				e.SetPos({
							static_cast<float>(rand() % 10 + 2.0f),
							0.0f,
							static_cast<float>(rand() % 10 + 2.0f)
					});
				for (auto& t : mTraders)									//Check each trader in the game
					while (e.CheckCollision(e.GetPos(), t.GetPos()))	//Prevents enemies from spawning inside a trader
						e.SetPos({
							static_cast<float>(rand() % 10 + 2.0f),
							0.0f,
							static_cast<float>(rand() % 10 + 2.0f)
							});
				for (auto& b : mBuildings)
					while (e.CheckCollision(e.GetPos(), b.GetPos()))
						e.SetPos({
							static_cast<float>(rand() % 10 + 2.0f),
							1.0f,
							static_cast<float>(rand() % 10 + 2.0f)
							});
			}
		}

		// fill until full again
		if (fill == true) { 

			int e1 = (rand() % 2) + 1; // random 1-2
			if (e1 == 1) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_1));
			}
			else if (e1 == 2) {
				mEnemies.push_back(Spawn(GC::ENEMY_TYPE_2));
			}
			//todo enemy model based on type  -- "EnemyGhoul"   "Enemy"
			mEnemies.back().SetPos({
				static_cast<float>(rand() % 10 + 2.0f),
				0.0f,
				static_cast<float>(rand() % 10 + 2.0f)
				});
			for (auto& t : mTraders)									//Check each trader in the game
				while (mEnemies.back().CheckCollision(mEnemies.back().GetPos(), t.GetPos()))	//Prevents enemies from spawning inside a trader
					mEnemies.back().SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						0.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});
			for (auto& b : mBuildings)									
				while (mEnemies.back().CheckCollision(mEnemies.back().GetPos(), b.GetPos()))
					mEnemies.back().SetPos({
						static_cast<float>(rand() % 10 + 2.0f),
						1.0f,
						static_cast<float>(rand() % 10 + 2.0f)
						});

		}
	}
}

void PlayState::reInitialize() { // USED TO LOAD A NEW MAP & ENEMIES, ETC, WHEN LEAVING AN AREA

	ReGen();
	eGen(false);
	timeSet();

	timeChange = 0.0f;

	mCombatController.Initialize(&mPlayer, &mPlayerWeapon, &mEnemies);

	mPlayer.AreaClear = false;
	mPlayer.genArea = false;
	bool set = false;
	if (mPlayer.GetPos().x >= GC::PLAYER_RIGHTBOUND - 0.9375f && set == false) { mPlayer.SetPos({ -14.0f,0.0f,0.0f }); set = true; }
	if (mPlayer.GetPos().x <= GC::PLAYER_LEFTBOUND + 0.9375f && set == false) { mPlayer.SetPos({ 14.0f,0.0f,0.0f }); set = true; }
	if (mPlayer.GetPos().z >= GC::PLAYER_UPBOUND - 0.9375f && set == false) { mPlayer.SetPos({ 0.0f,0.0f,-14.0f }); set = true; }
	if (mPlayer.GetPos().z <= GC::PLAYER_DOWNBOUND + 0.9375f && set == false) { mPlayer.SetPos({ 0.0f,0.0f,14.0f }); set = true; }
}

void PlayState::timeSet() {
	PassConstants* pMainPassCB = GameApp::Get().GetMainPassCB();
	//GameApp::Get().setPhase(timeCycle);
	if (timeCycle == 1) {
		pMainPassCB->AmbientLight = GC::DAWN_COLOUR; // dawn
		pMainPassCB->Lights[1].Strength = GC::DAWN_STRENGTH;
	}
	if (timeCycle == 2) {
		pMainPassCB->AmbientLight = GC::NOON_COLOUR; // noon
		pMainPassCB->Lights[1].Strength = GC::NOON_STRENGTH;
	}
	if (timeCycle == 3) {
		pMainPassCB->AmbientLight = GC::EVENING_COLOUR; // evening
		pMainPassCB->Lights[1].Strength = GC::EVENING_STRENGTH;
	}
	if (timeCycle == 4) {
		pMainPassCB->AmbientLight = GC::PITCH_COLOUR; // pitch
		pMainPassCB->Lights[1].Strength = GC::PITCH_STRENGTH;
	}
}

void PlayState::Update(const GameTimer& gt)
{
	//mTileManager.Update(gt);
	mCombatController.Update(gt);

	switch (GameApp::Get().menusShown)
	{
	case GC::NO_TUTORIAL_VALUE:
		shownAttackTutorial = true;
		shownInventoryUseTutorial = true;
		shownPlantTutorial = true;
	case 0:
		GameApp::Get().mTutorialText = GC::TUTORIAL_INTRO;
		GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
		break;

	case 1:
		GameApp::Get().mTutorialText = GC::TUTORIAL_OBJECTIVE;
		GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
		break;

	case 2:
		GameApp::Get().mTutorialText = GC::TUTORIAL_MOVE;
		GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
		break;

	case 3:
		GameApp::Get().mTutorialText = GC::TUTORIAL_TRADE;
		GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
		break;

	case 4:
		GameApp::Get().mTutorialText = GC::TUTORIAL_INVENTORY;
		GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
		break;
	}

	if (revolvingHintTimer.HasTimeElapsed(gt.DeltaTime(), 8.f))
	{
		mHelpMessage.mText.center = false;
		mHelpMessage.mText.position = DirectX::SimpleMath::Vector2{ 250.f, 550.f };
		mHelpMessage.mText.color = DirectX::Colors::Red;
		mHelpMessage.Activate(GC::HELP_MESSAGES[revolvingHintPosition], 7.0f);
		revolvingHintPosition++;
		if (revolvingHintPosition > 8)
			revolvingHintPosition = 2;
	}

	if (playerName == "")		//Only does this once
		GetName();

	timeChange += gt.DeltaTime();
	
	// AFTER 2 MINUTES CHANGE TIME PHASE IN GAME (ONLY PARTICULARLY USEFUL DURING PITCH )
	if (timeChange >= 10.0f) {
		timeChange = 0.0f;
		timeCycle += 1;
		if (timeCycle > 4)
			timeCycle = 1;
		eGen(false);
		timeSet();
	}

	//Transition to win state when collecting all items needed for the car
	if (TraderStoryComplete())
	{
		StoreScore();
		GameApp::Get().ChangeState("WinState");
	}

	for (auto& t : mTraders)		//Checks all traders in the game
	{
		if (mPlayer.CheckCollision(mPlayer.GetPositionWithVelocity(gt), t.GetPos()) ||		//Prevents player from walking through trader and being pushed by an enemy into trader
			mPlayer.CheckCollision(mPlayer.GetPos() + mPlayer.BouncebackPosition, t.GetPos()))
		{
			mPlayer.SetVelocity(0.0f);		//Prevents moving through the trader
			mPlayer.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);		//Prevents the bounceback from happening
		}

		if (mPlayerWeapon.CheckCollision(mPlayerWeapon.GetPos(), t.GetPos()))	//Prevents weapon from going through trader
			mPlayerWeapon.ResetWeaponPosition();
	}

	for (auto& b : mBuildings)		
	{
		if (mPlayer.CheckCollision(mPlayer.GetPositionWithVelocity(gt), b.GetPos()) ||		
			mPlayer.CheckCollision(mPlayer.GetPos() + mPlayer.BouncebackPosition, b.GetPos()))
		{
			mPlayer.SetVelocity(0.0f);		
			mPlayer.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);		
		}

		if (mPlayerWeapon.CheckCollision(mPlayerWeapon.GetPos(), b.GetPos()))	
			mPlayerWeapon.ResetWeaponPosition();
	}

	//Checking player remains in bounds when being attacked
	if (!mPlayer.WithinBounds(mPlayer.GetPos() + mPlayer.BouncebackPosition))
		mPlayer.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);
	

	mPlayer.Update(gt);

	// reset hazard timer for tile hazards 
	if (mPlayer.hazardTimer >= 0) {
		mPlayer.hazardTimer -= gt.DeltaTime();
	}

	// Game Camera follows player
	mCameras.at(CAMERA_TYPE::GAME).SetPosition(
		Lerp(mCameras.at(CAMERA_TYPE::GAME).GetPosition(), mPlayer.GetPos() + CAM_OFFSET, 0.9999f * gt.DeltaTime())
	);

	// MUST CHECK BOTH METHODS
	// currently -15 to 15 on both (BOUNDARIES SHOULD BE PUBLIC AND USED TO CALCULATE)
	// tiles are 32 by 32
	// 0.0f is middle of grid
	// each tile is 0.9375 of a world position
	// player position + half max tile * (MaxWorldPos / MaxTile) to find current tile (REPEAT FOR X & Y APPROPRIATELY)
	// LAST COMPONENT OF THE CALCULATION WILL CAUSE PROBLEMS IF SIZE OF EACH TILE ITSELF INCREASED
	//const float diff = float(mTileManager.MaxGen) / 30.0f;
	const float diff2 = 30.0f / float(mTileManager.MaxGen); // WORLDSPACE / GRID SIZE
	int underX = (int)round(mPlayer.GetPos().x); // worldspace position does not correspond to tilemap coordinate
	int underZ = (int)round(mPlayer.GetPos().z);
	float tileX = (underX + (0.5f * (mTileManager.MaxGen))/* * diff*/); // greater
	float tileZ = (underZ + (0.5f * (mTileManager.MaxGen)))/* * diff*/; // greater
	float tileX2 = (underX + (0.5f * (mTileManager.MaxGen)) * diff2); //lesser
	float tileZ2 = (underZ + (0.5f * (mTileManager.MaxGen)) * diff2); // lesser

	// USED FOR DEBUGGING THE SAFETY NET FOR THE PLAYER GRID CHECKER - KEEP IF ERRORS ARISE LATER
	//if ((tileX < 0 || tileZ < 0 || tileX > mTileManager.MaxGen || tileZ > mTileManager.MaxGen) ||
	//	(tileX2 < 0 || tileZ2 < 0 || tileX2 > mTileManager.MaxGen || tileZ2 > mTileManager.MaxGen)) {
	//
	//	int r = 0;
	//}

	// if player over the grid execute the grid check 
	if ((tileX > 0 && tileZ > 0 && tileX < mTileManager.MaxGen && tileZ < mTileManager.MaxGen) &&
		(tileX2 >= 0 && tileZ2 >= 0 && tileX2 <= mTileManager.MaxGen && tileZ2 <= mTileManager.MaxGen)) {

		// POSSIBLE GLITCH HERE (TILE ACTIVE WHEN PLAYER NOT OVER IT, TILE RIGHT / RIGHT&UP) - FIXED ITSELF??
		// 
		// if the player is over a poison/damage tile
		if (mTileManager.GetIndex(static_cast<int>(tileX), static_cast<int>(tileZ)) == mTileManager.Haz1Tex ||
			mTileManager.GetIndex(static_cast<int>(tileX2), static_cast<int>(tileZ2)) == mTileManager.Haz1Tex) {
			if (mPlayer.hazardTimer <= 0) { // if hazard should be active
				mPlayer.health -= 5;
				mPlayerHealthBar.SetValue(mPlayer.health);
				mPlayer.hazardTimer = 3; // reset hazard timer 

			}
		}

		// if the player is over a slow tile
		if (mTileManager.GetIndex(static_cast<int>(tileX), static_cast<int>(tileZ)) == mTileManager.Haz2Tex ||
			mTileManager.GetIndex(static_cast<int>(tileX2), static_cast<int>(tileZ2)) == mTileManager.Haz2Tex) {
			mPlayer.Slowed = true;
		}

		if (mTileManager.GetIndex(static_cast<int>(tileX), static_cast<int>(tileZ)) != mTileManager.Haz2Tex &&
			mTileManager.GetIndex(static_cast<int>(tileX2), static_cast<int>(tileZ2)) != mTileManager.Haz2Tex) {
			mPlayer.Slowed = false;
		}

		if (mTileManager.GetIndex(static_cast<int>(tileX), static_cast<int>(tileZ)) == mTileManager.Haz3Tex ||
			mTileManager.GetIndex(static_cast<int>(tileX2), static_cast<int>(tileZ2)) == mTileManager.Haz3Tex) {
			mPlayer.Slippy = true;
		}

		if (mTileManager.GetIndex(static_cast<int>(tileX), static_cast<int>(tileZ)) != mTileManager.Haz3Tex &&
			mTileManager.GetIndex(static_cast<int>(tileX2), static_cast<int>(tileZ2)) != mTileManager.Haz3Tex) {
			mPlayer.Slippy = false;
		}
	}


	int i = 0;
	std::for_each(mEnemies.begin(), mEnemies.end(), [&](Enemy& e)
	{
		e.Update(gt);


		if (e.mEnabled)
		{
			//Enemy look at players position (only do when in range), only look when not attacking either
			XMVECTOR playerPosition = XMLoadFloat3(&mPlayer.GetPos());

			//Generically gets the enemy range without duplicating code
			float enemyRange;
			if (e.GetType() == GC::ENEMY_TYPE_1)
				enemyRange = GC::ENEMYTYPE1_RANGE;	// ENEMY TYPE EXCLUSIVE LOGIC LOCATED HERE
			else
				enemyRange = GC::ENEMYTYPE2_RANGE;

			if (timeCycle == 4)
				enemyRange = GC::ALL_ENEMY_MAXRANGE;

			// TODO: (REMEMBER) IF PLAYER IN RANGE OF SIGHT LOCATED HERE, COULD IMPROVE & IMPLEMENT FOR OTHER ENEMY TYPES
			if (mPlayer.GetPos().x >= (e.GetPos().x - enemyRange) &&
				mPlayer.GetPos().x <= (e.GetPos().x + enemyRange)) { // player within - range on x
				if (mPlayer.GetPos().z >= (e.GetPos().z - enemyRange) &&
					mPlayer.GetPos().z <= (e.GetPos().z + enemyRange)) { // player within - range on z
					e.LookAt(playerPosition);

					if (shownAttackTutorial == false)
					{
						GameApp::Get().mTutorialText = GC::TUTORIAL_ATTACK;
						GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
						shownAttackTutorial = true;
					}
				}
			}

			// enemy movement behaviour based on player radius
			if (DirectX::SimpleMath::Vector3::Distance(mPlayer.GetPos(), e.GetPos()) < enemyRange &&
				!e.GetIfCanAttack())
			{
				e.mBehaviour = Enemy::Behaviour::CHASE;
			}
			else
			{
				e.mBehaviour = Enemy::Behaviour::NONE;
			}
		}

		// enemy collision with player
		if (mPlayer.CheckCollision(mPlayer.GetPos(), e.GetPos()) && e.GetHealth() > 0)
		{
			mPlayer.DamagePlayer(e.GetAttack(), e, gt);
			mPlayerHealthBar.SetValue(mPlayer.health);

			if (shownPlantTutorial == false)	//replace bool?
			{
				GameApp::Get().mTutorialText = GC::TUTORIAL_PLANT;
				GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
				shownPlantTutorial = true;
			}

			//Transition to game over state
			if (mPlayer.health <= 0)
			{
				GameApp::Get().ChangeState("GameOver");
				GameApp::Get().GetAudio().Play("playerDie", nullptr, false, 1.0f, GetRandomVoicePitch());
			}
			else
			{
				GameApp::Get().GetAudio().Play("playerHit01", nullptr, false, 1.0f, GetRandomVoicePitch());
			}
		}

		//When checking if enemy is in range, have this be in that section to prevent enemy particles from far from being checked
		for (auto& p : e.particles)
		{
			if (mPlayer.CheckCollision(mPlayer.GetPos(), p.GetPos()) && e.GetHealth() > 0)
			{
				mPlayer.DamagePlayer(e.GetAttack(), e, gt);
				mPlayerHealthBar.SetValue(mPlayer.health);

				if (shownPlantTutorial == false)	//replace bool?
				{
					GameApp::Get().mTutorialText = GC::TUTORIAL_PLANT;
					GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
					shownPlantTutorial = true;
				}

				GameApp::Get().GetAudio().Play("playerHit01", nullptr, false, 1.0f, GetRandomVoicePitch());
				//Transition to game over state
				if (mPlayer.health <= 0)
				{
					GameApp::Get().ChangeState("GameOver");
					GameApp::Get().GetAudio().Play("playerDie", nullptr, false, 1.0f, GetRandomVoicePitch());
				}
				else
				{
					GameApp::Get().GetAudio().Play("playerHit01", nullptr, false, 1.0f, GetRandomVoicePitch());
				}
				break;
			}
		}

		if (mPlayerWeapon.CheckCollision(mPlayerWeapon.GetPos(), e.GetPos()))
		{
			// TODO: (URGENT) FIX ENEMY MODEL LEFT ON SCREEN

			e.DamageEnemy(mPlayer.attack);		//Takes away health from enemy + blowsback enemy position
			if (e.GetHealth() <= 0)
			{
				// gain exp
				mExperience.AddExp(GC::EXP_DEFAULT);
				score += GC::SCORE_ENEMY;

				//Could be put into an exists function in Inventory Class
				std::string droppedItem = e.GetDropItem();
				if (droppedItem != "Empty")		//I.e An item was dropped from enemy
				{
					++mInventory[droppedItem];
				}


				e.MoveOffScreen();
				e.mEnabled = false;
				if (timeCycle == 4)
					newEnemy = true;

				GameApp::Get().GetAudio().Play("EnemyDie1", nullptr, false, 1.0f, GetRandomVoicePitch());
			}
			else
			{
				GameApp::Get().GetAudio().Play("EnemyHit1", nullptr, false, 1.0f, GetRandomVoicePitch());
			}
			e.SetVelocity(mPlayer.GetPos(), gt);
		}

		//Checking enemy remains in bounds
		if (!e.WithinBounds(e.GetPos() + e.BouncebackPosition))
			e.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);

		// enemy collides with traders
		for (auto& t : mTraders)
			if (e.CheckCollision(e.GetPos() + e.BouncebackPosition, t.GetPos()))		//If there is a collision between any of the traders and the bounceback position of the enemy
				e.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);

		// enemy collides with buildings
		for (auto& b : mBuildings)
			if (e.CheckCollision(e.GetPos() + e.BouncebackPosition, b.GetPos()))		
				e.BouncebackPosition = DirectX::XMFLOAT3(0.0f, 0.0f, 0.0f);


		e.SetVelocity(mPlayer.GetPos(), gt);

		

		i++;

	});

	// TODO: (REMEMBER) METHOD OF REGENNING ENEMIES AFTER THEIR DEATH HERE
	if (newEnemy == true) {
		newEnemy = false;
		eGen(true);
	}
	PassConstants* pMainPassCB = GameApp::Get().GetMainPassCB();

	//flashing red for low health
	if (mPlayer.health <= GC::PLAYER_LOW_HEALTH)
	{
		float strength = sin(gt.TotalTime() * 5.0f) * 0.5f + 0.5f;

		pMainPassCB->Lights[1].Strength = { strength ,0.1f,0.1f };
		// Less intense vibration
		strength *= 0.1f;

		Input::Get().SetVibration(strength, strength);
	}
	else
	{
		pMainPassCB->Lights[1].Strength = { 0.0f, 0.0f, 0.0f };
	}

	UpdatePlants(gt);



	UiUpdate(gt);

	if (mHelpLocation.first.y < 500.f)
		mHelpMessage.Activate("", 1.0f);		//Deactivates other hints from showing while help menu is open

	// Sprite update

	if (mExperience.HasLeveledUp())
	{
		mMessage.Activate("LEVEL UP", 5.0f);
		score += GC::SCORE_LVLUP;
		mPlayer.health = GC::PLAYER_MAX_HEALTH;
		mPlayerHealthBar.SetValue(mPlayer.health);
	}

	// show/hide item menu
	if (itemMenuOpen)
	{
		if (shownInventoryUseTutorial == false)
		{
			GameApp::Get().mTutorialText = GC::TUTORIAL_USE;
			GameApp::Get().ChangeState(GC::STATE_TUTORIAL);
			shownInventoryUseTutorial = true;
		}
		mInventoryText.string = "Inventory\n";
		// Selected item

		const std::string selectedItem = (*inventoryPosition).first;
		
		// all inventory items. Selected item prefixed with '>'
		std::for_each(mInventory.begin(), mInventory.end(), [&](auto& inv)
		{
			if (inventoryPosition != mInventory.end())
			{

				if ((*inventoryPosition).first == inv.first)
				{
					mInventoryText.string += ">";
				}
				else
				{
					mInventoryText.string += " ";
				}
			}
			mInventoryText.string += " " + inv.first + " (" + std::to_string(inv.second) + ")\n";

		});

		// Display weapon attack power
		if (GC::ITEM_LIST.at(selectedItem).category == ItemCategory::WEAPON)
		{
			mInventoryText.string += "\nAttack +" + std::to_string(mPlayerWeapon.GetWeaponStats(selectedItem)) + "\n";
		}

		//Help instructions for inventory
		mHelpMessage.mText.center = true;
		mHelpMessage.mText.position = DirectX::SimpleMath::Vector2{ 650.f, 450.f };
		mHelpMessage.mText.color = DirectX::Colors::White;
		mHelpMessage.Activate(GC::HELP_MESSAGES[0], 0.1f);
	}

	// IMPLEMENT CHECK FOR ENEMIES HERE
	if (EnemiesRemaining() == 0 && !mPlayer.AreaClear)
		mPlayer.AreaClear = true;

	// final goal is to reach a safe house, need to harvest a certain number of plants as payment to get in?

	if (mPlayer.AreaClear && mPlayer.genArea) { // TODO: (REMEMBER) IMPLEMENT CHANGE STATE FOR NEW AREA HERE
		// change state, trigger regen
		areas += 1;
		timeCycle += 1;
		if (timeCycle > 4)
			timeCycle = 1;
		timeChange = 0.0f;
		if (areas < 4) {
			reInitialize();
			++GameApp::Get().mStoryIndex;
			GameApp::Get().ChangeState("Story"); //"NewArea1"
			//GameApp::Get().ChangeState(GC::STATE_NEW_AREA);
		}
		if (areas == 4) { // TODO: (REMEMBER) CURRENTLY WIN CONDITION IS HERE
			reInitialize();
			areas = 0;
			timeCycle = 1;
			StoreScore();
			GameApp::Get().ChangeState(GC::STATE_WIN);
		}
	}

	// for dirty frame
	for (auto& c : mCameras)
	{
		c.UpdateViewMatrix();
	}
}

void PlayState::Draw(const GameTimer& gt)
{
	// UI

	mPlayerHealthBar.Draw();

	mInventoryPanel.Draw();
	mInventoryText.Draw();


	mMessage.Draw();
	mHelpMessage.Draw();
	mTradeHelpMessage.Draw();

	mHelpPanel.Draw();
	mHelpText.Draw();

	mScoreTextShadow.Draw();
	mScoreText.Draw();

}

void PlayState::OnMouseDown(WPARAM btnState, int x, int y)
{
}

void PlayState::OnMouseUp(WPARAM btnState, int x, int y)
{
	//if (((btnState & MK_LBUTTON) != 0))
	//{

	//	POINT lastMousePos = GameApp::Get().GetLastMousePos();

	//	// Make each pixel correspond to a quarter of a degree.
	//	float dx = XMConvertToRadians(0.25f*static_cast<float>(x - lastMousePos.x));
	//	float dy = XMConvertToRadians(0.25f*static_cast<float>(y - lastMousePos.y));

	//	mCameras.at(CAMERA_TYPE::GAME).Pitch(dy);
	//	mCameras.at(CAMERA_TYPE::GAME).RotateY(dx);
	//}
}

void PlayState::OnMouseMove(WPARAM btnState, int x, int y)
{
}

void PlayState::OnKeyboardInput(const GameTimer& gt)
{

	Controls(gt);

}

void PlayState::OnResume()
{
	// remove focus from trader
	mpActiveTrader = nullptr;

	GameAudio& ga = GameApp::Get().GetAudio();

	ga.Play("actionMusic", nullptr, true);

	// Random ambient track
	if (rand() % 2) // non zero
	{
		ga.Play("ambientWind", nullptr, true);
	}
	else // zero
	{
		ga.Play("ambientCrows", nullptr, true);

	}
}

void PlayState::OnPause()
{
	GameApp::Get().GetAudio().Pause("ambient");
}

/*

If gamepad
	gamepad
else keyboard
	keyboard
	switch cam
		play then keyboard player move
		debug then keyboard debug cam move
*/

void PlayState::Controls(const GameTimer& gt)
{


	//bool playerMoved = false;

	//Gamepad
	if (Input::Get().GamePadConnected())
	{

		Gamepad(gt);

	}
	else // Keyboard controls. Game/Debug cam
	{
		switch (mCamType)
		{
		case CAMERA_TYPE::GAME:
		{
			Keyboard(gt);
		}
		break;
		case CAMERA_TYPE::DEBUG:
		{
			KeyboardDebug(gt);
		}
		break;
		}


	}


}

void PlayState::InventoryUp()
{
	if (inventoryPosition == mInventory.begin())
	{
		inventoryPosition = mInventory.rbegin().base(); // last element 
	}

	--inventoryPosition;
}

void PlayState::InventoryDown()
{
	++inventoryPosition;

	if (inventoryPosition == mInventory.end())
	{
		inventoryPosition = mInventory.begin();						//Loop back round to top of inventory
	}
}

void PlayState::ItemAction()
{
	assert(inventoryPosition != mInventory.end());

	switch (GC::ITEM_LIST.at((*inventoryPosition).first).category)
	{
	case  ItemCategory::HARVESTED_PLANT:
		if (inventoryPosition->second > 0)
		{
			if (inventoryPosition->first == GC::PLANT_NAME_0)
			{
				mPlayer.health += GC::HEAL_SMALL;
			}
			else if (inventoryPosition->first == GC::PLANT_NAME_1)
			{
				mPlayer.health += GC::HEAL_MED;
			}
			else if (inventoryPosition->first == GC::PLANT_NAME_2)
			{
				mPlayer.health += GC::HEAL_LARGE;
			}
			else
			{
				assert(false); // check ItemCategory for item
			}
			--inventoryPosition->second; //removes a potion

			// Apply health limit
			if (mPlayer.health > GC::PLAYER_MAX_HEALTH)
			{
				mPlayer.health = GC::PLAYER_MAX_HEALTH;
			}

			mPlayerHealthBar.SetValue(mPlayer.health);
		}
		break;
	case ItemCategory::WEAPON:
		if (mCombatController.GetCurrentWeapon() != (*inventoryPosition).first)
		{
			mCombatController.EquipWeapon((*inventoryPosition).first);
		}
		break;
	case ItemCategory::SEED:
		// if there isnt a plant in a radius then instance 
		if (CreatePlant())
		{
			--inventoryPosition->second;
		}

		break;
	case ItemCategory::KEY_ITEM:
		if (inventoryPosition->first == "Radio")
		{
			mMessage.Activate(GC::RADIO_MESSAGES[rand() % GC::RADIO_MESSAGES_SIZE],5.0f);
		}

		break;
	}

	CleanInventory(mInventory); // tidy up

}

void PlayState::CleanInventory(Inventory& inv)
{
	for (Inventory::iterator it = inv.begin(); it != inv.end();)
	{
		if ((*it).second <= 0)
		{
			it = inv.erase(it);
			inventoryPosition = inv.begin();
		}
		else {
			++it;
		}
	}
}

void PlayState::ReGen() { // DOESN'T WORK PROPERLY, JUST CUTS FRAME RATE, NEED TO REMOVE TEXTURES FROM OBJECTS

	mTileManager.REGEN();
}

void PlayState::UiUpdate(const GameTimer& gt)
{

	mPlayerHealthBar.Update(gt);

	// animate inventory toggle
	SimpleMath::Vector2 move = SimpleMath::Vector2::Lerp(mInventoryPanel.GetPos(), mInventoryLocation.first, gt.DeltaTime() * 2.0f); // todo Luc move to constant.h
	mInventoryPanel.Move(move, false);
	// padding for text
	move += SimpleMath::Vector2{ 32.0f, 32.0f }; //todo luc move to constants.h
	mInventoryText.position = move;

	//animate help toggle
	move = SimpleMath::Vector2::Lerp(mHelpPanel.GetPos(), mHelpLocation.first, gt.DeltaTime() * 2.0f); // todo Luc
	mHelpPanel.Move(move, false);
	//padding for help text
	move += SimpleMath::Vector2{ 32.0f, 32.0f };//todo luc move to constants.h
	mHelpText.position = move;

	mScoreText.string = "Score: " + std::to_string(score);
	mScoreTextShadow.string = mScoreText.string;

	mMessage.Update(gt);
	mHelpMessage.Update(gt);
	mTradeHelpMessage.Update(gt);
}

bool PlayState::CreatePlant()
{

	// to tile nearest to player
	SimpleMath::Vector3 pos(mPlayer.GetPosV3());
	pos.x = round(pos.x);
	pos.y = round(pos.y);
	pos.z = round(pos.z);

	// check if plant is already at location
	bool found = false;
	int i = 0;

	while (i < mPlants.size() && !found)
	{
		if (SimpleMath::Vector3::Distance(pos, mPlants.at(i).GetPosV3()) < 0.1f)
		{
			// There is already a plant at this location
			found = true;
		}
		++i;
	}


	if (!found)
	{
		// todo Luc change basic on item rather than random
		int r = rand() % 3;
		switch (r)
		{
		case 0:	mPlants.push_back(Plant(GC::PLANT_0)); break;
		case 1:	mPlants.push_back(Plant(GC::PLANT_1)); break;
		case 2:	mPlants.push_back(Plant(GC::PLANT_2)); break;
		default:
			break;
		}

		mPlants.back().Initialize("Potato");
		mPlants.back().SetPos(pos);
	}

	return !found;

}

void PlayState::UpdatePlayer(const GameTimer& gt)
{
}

void PlayState::UpdatePlants(const GameTimer& gt)
{
	for (auto& p : mPlants)
	{
		p.Update(gt);
	}
}

void PlayState::UpdateTiles(const GameTimer& gt)
{
}

void PlayState::UpdateEnemies(const GameTimer& gt)
{
}

void PlayState::HarvestByRadius()
{
	for (auto& p : mPlants)
	{
		// player within range
		if (SimpleMath::Vector3::Distance(mPlayer.GetPosV3(), p.GetPosV3()) < 1.5f) // todo Luc move to constant.h
		{
			if (p.Harvest()) // applies changes to instance state
			{
				// add all items from lookup to inventory
				for (auto& m : GC::PLANT_LOOKUP_HARVEST.at(p.GetName()))
				{
					mInventory[m.first] += m.second;
				}

				score += GC::SCORE_PLANT;
			}
		}
	}
}

void PlayState::Keyboard(const GameTimer& gt)
{

	//Player keyboard movement
	if (Input::Get().KeyHeld(GC::KEY_FW))
	{
		mPlayer.MoveUp(gt);
	}

	if (Input::Get().KeyHeld(GC::KEY_BK))
	{
		mPlayer.MoveDown(gt);
	}

	if (Input::Get().KeyHeld(GC::KEY_RT))
	{
		mPlayer.MoveRight(gt);
	}

	if (Input::Get().KeyHeld(GC::KEY_LT))
	{
		mPlayer.MoveLeft(gt);
	}

	//Player attack and harvest
	if (Input::Get().KeyReleased(GC::KEY_ATTACK))
	{
		mCombatController.PlayerAttack();
		HarvestByRadius();
	}

	//Switch to debug cam
	if (Input::Get().KeyReleased(GC::KEY_CAM))
	{
		mCamType = CAMERA_TYPE::DEBUG;

		//Set debug cam to game position to avoid getting lost
		mCameras.at(CAMERA_TYPE::DEBUG).SetPosition(mCameras.at(CAMERA_TYPE::GAME).GetPosition3f());

		//Update active camera
		GameApp::Get().SetActiveCamera(&mCameras.at(CAMERA_TYPE::DEBUG));
	}


	// Inventory

	// Scrolling through inventory keys Debug
	if (Input::Get().KeyReleased(VK_DOWN) & itemMenuOpen)
	{
		InventoryDown();
	}

	if (Input::Get().KeyReleased(VK_UP) & itemMenuOpen)
	{
		InventoryUp();
	}

	// Opens and closes item menu
	if (Input::Get().KeyReleased(GC::KEY_INVENTORY))
	{
		itemMenuOpen = !itemMenuOpen;
		std::swap(mInventoryLocation.first, mInventoryLocation.second);
	}

	// Use Item Key, change to something else, or only allow when on item/pause menu
	if (Input::Get().KeyReleased(GC::KEY_USEITEM) & itemMenuOpen)
	{
		ItemAction();
	}

	// Toggle Help
	if (Input::Get().KeyReleased(GC::KEY_HELP))
	{
		std::swap(mHelpLocation.first, mHelpLocation.second);
		mHelpMessage.Activate("", 0.f);
	}

	// Trade
	if (Input::Get().KeyReleased(GC::KEY_TRADE))
	{
		if (FindNearestTraderInRadius())
		{
			//todo play sound
			assert(mpActiveTrader);
			GameApp::Get().ChangeState(GC::STATE_TRADE);
		}
		else
		{
			//todo sound fail sound
		}
	}

	// Loot
	if (Input::Get().KeyReleased('L'))
	{
		if (FindNearestBuildingInRadius())
		{
			//todo play sound
			assert(mpActiveBuilding);

			++mInventory["Ham"];
			++mInventory["Cheese"];
			inventoryPosition = mInventory.begin();
		}
		else
		{
			//todo sound fail sound
		}
	}
	
	// Pause
	if (Input::Get().KeyReleased(GC::KEY_PAUSE))
	{
		GameApp::Get().ChangeState("PauseMenu");
	}

	// Debug generate
	if (Input::Get().KeyReleased(GC::KEY_DEBUG_GENERATE)) //todo VOID NEEDS FIXING FIRST
	{
		ReGen();
	}

	if (Input::Get().KeyReleased(GC::KEY_DEBUG_KILL_ALL))
	{
		//mEnemies.clear();

		for (auto& e : mEnemies)
		{
			e.Reset();
			e.mEnabled = false;
			e.MoveOffScreen();
		}
	}
}

void PlayState::KeyboardDebug(const GameTimer& gt)
{

	const float dt = gt.DeltaTime();

	//dpad debug cam
	if (Input::Get().KeyHeld(GC::KEY_FW))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Walk(GC::ZOOM_SPEED * dt);
	}

	if (Input::Get().KeyHeld(GC::KEY_BK))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Walk(-GC::ZOOM_SPEED * dt);
	}

	if (Input::Get().KeyHeld(GC::KEY_RT))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Strafe(GC::MOVE_SPEED * dt);
	}

	if (Input::Get().KeyHeld(GC::KEY_LT))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Strafe(-GC::MOVE_SPEED * dt);
	}

	if (Input::Get().KeyHeld(GC::KEY_RAISE))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Elevate(GC::MOVE_SPEED * dt);
	}

	if (Input::Get().KeyHeld(GC::KEY_LOWER))
	{
		mCameras.at(CAMERA_TYPE::DEBUG).Elevate(-GC::MOVE_SPEED * dt);
	}

	// Switch to game camera
	if (Input::Get().KeyReleased(GC::KEY_CAM))
	{
		mCamType = CAMERA_TYPE::GAME;
		//Update active camera
		GameApp::Get().SetActiveCamera(&mCameras.at(CAMERA_TYPE::GAME));
	}

	// Debug random enemy pos
	if (Input::Get().KeyReleased(GC::KEY_DEBUG_ENEMY_POS))
	{
		std::for_each(mEnemies.begin(), mEnemies.end(), [](Enemy& e)
			{
				e.mEnabled = true;
				e.Reset();
				e.SetRandomPosition();
			});
	}
}

void PlayState::Gamepad(const GameTimer& gt)
{

	// Left stick
	mPlayer.Move(gt, Input::Get().LeftStickXZ() * GC::MOVE_SPEED);

	// Attack
	if (Input::Get().GamePad().rightTrigger == ButtonState::HELD)
	{
		mCombatController.PlayerAttack();
	}

	if (Input::Get().GamePad().start == ButtonState::PRESSED)
	{
		GameApp::Get().ChangeState("PauseMenu");
	}

	if (Input::Get().GamePad().y == ButtonState::PRESSED)
	{
		itemMenuOpen = !itemMenuOpen;
	}

	if (Input::Get().GamePad().dpadUp == ButtonState::PRESSED)
	{
		InventoryUp();
	}

	if (Input::Get().GamePad().dpadDown == ButtonState::PRESSED)
	{
		InventoryDown();
	}

	if (Input::Get().GamePad().a == ButtonState::PRESSED)
	{
		ItemAction();
	}

}

bool PlayState::TraderStoryComplete()
{
	int count = 0;

	for (auto& t : mTraders)
	{
		if (t.CompletedStoryItem())
		{
			++count;
		}
	}

	return count >= 3; // todo constant to header
}

void PlayState::Reset()
{
	score = 0;		//Reset Score
	playerName = "";

	mPlayer.Reset();
	mCombatController.Reset();
	mPlayerWeapon.Reset();

	mPlayerHealthBar.SetMinMax(0, GC::PLAYER_MAX_HEALTH); 
	mPlayerHealthBar.SetValue(mPlayer.health);
	mPlayerHealthBar.SetPosition({ 200.0f, 20.0f }); // todo add values to constants.h
	mPlayerHealthBar.SetValue(GC::PLAYER_MAX_HEALTH);

	//Reset Help & Inventory menus
	const DirectX::XMINT2 s = GameApp::Get().GetClientSizeU2(); //todo on resize
	SimpleMath::Vector2 v = GameApp::Get().GetClientSize();

	//Help Menu
	mHelpLocation.second.y = v.y - (v.y / 3);
	mHelpLocation.second.x = 10;
	mHelpLocation.first.y = v.y + 64;
	mHelpLocation.first.x = 10;

	//Inventory Menu
	mInventoryLocation.second.x = v.x - (v.x / 3);
	mInventoryLocation.second.y = 10;

	mInventoryLocation.first.x = v.x + 64;
	mInventoryLocation.first.y = 10;

	//Start Message
	mMessage.mText.center = true;
	mMessage.mText.position = v / 2.0f;
	mMessage.mText.color = DirectX::Colors::Red;
	mMessage.Activate("Start", 1.f);

	//Reset Inventory
	mInventory.clear();
	++mInventory["Radio"];
	++mInventory["Glowing Seeds"];
	++mInventory["Nail Bat"];
	inventoryPosition = mInventory.begin();

	//Reset help text
	mHelpMessage.mText.string = "";
	mTradeHelpMessage.mText.string = "";

	//Reset Traders
	float x = -10.5f;
	const int biggestGap = 4;
	float z = 5.0f;
	for (auto& t : mTraders)
	{
		if (!t.GetIfStoryItem())		//Keeps position of car the same
		{
			x += 1.0f + (rand() % biggestGap);
			z = 5.0f + (rand() % biggestGap);
			t.SetPos({ x, 0.0f, z });
		}

		t.ResetTrader();		//Resets quests/trades for each trader
	}

	//Reset Plants
	for (auto& p : mPlants)
	{
		p.SetPos(DirectX::XMFLOAT3(0.0f, -200.f, 0.0f));
	}
	mPlants.clear();

	//Reset Tiles
	ReGen();

	//Reset Buildings // TODO: TEMP

	//mBuildings.clear();
	for (auto& b : mBuildings)
	{
		x += 1.0f + (rand() % biggestGap);
		z = 5.0f + (rand() % biggestGap);
		b.SetPos({ x, 0.0f, z });
	}

	//Reset Lighting
	GameApp::Get().mStoryIndex = 0;
	areas = 0;
	timeCycle = 1;
	timeChange = 0.0f;
	GameApp::Get().GetMainPassCB()->AmbientLight = GC::DAWN_COLOUR; // dawn
	GameApp::Get().GetMainPassCB()->Lights[1].Strength = GC::DAWN_STRENGTH;

	// TODO: (REMEMBER) LOOK AT BELOW FOR ENEMY RESETTING (MAY OR MAY NOT BE NEEDED)
	//Reset Enemies
	size_t enemyAmount = mEnemies.size() - 1;
	while (enemyAmount > 1)
	{
		mEnemies.at(enemyAmount - 1).MoveOffScreen();
		mEnemies.erase(mEnemies.begin() + (enemyAmount - 1));
		enemyAmount--;
	}

	for (auto& e : mEnemies)
	{
		e.mEnabled = true;
		e.Reset();

		for (auto t : mTraders)
			while (e.CheckCollision(e.GetPos(), t.GetPos()) ||
				DirectX::SimpleMath::Vector3::Distance(mPlayer.GetPos(), e.GetPos()) < (GC::ENEMYTYPE1_RANGE + 2.f))	//Prevents enemies from spawning inside a trader
				e.SetRandomPosition();
	}

}

void PlayState::GetName()
{
	ifstream fin("Data/Leaderboard.txt", ios::out);
	if (fin.fail())
		assert(fin.fail());
	else
	{
		std::vector<std::string> lines(6);
		//Get first 6 lines from the txt file
		for (size_t i = 0; i < lines.size(); i++)
			std::getline(fin, lines.at(i));

		playerName = lines.at(5);
		lines.pop_back();

		ofstream fout("Data/Leaderboard.txt", ios::out);
		if (fout.fail())
			assert(fout.fail());
		else
		{
			for (size_t i = 0; i < lines.size(); i++)		//Outputs back into the txt file
				if (i >= 4)
					fout << lines.at(i);
				else
					fout << lines.at(i) << endl;

			fout.close();
		}
	}
}

void PlayState::StoreScore()
{
	//Stores score to txt file
	std::vector<int> scores(5);
	std::vector<std::string> lines(5);

	ifstream fin;
	fin.open("Data/Leaderboard.txt", ios::out);

	if (fin.fail())		//Cannot find the txt file / It doesn't exist
	{
		ofstream fout("Data/Leaderboard.txt");		//Create the file
		if (fout.fail())
			assert(fout.fail());
	}
	else
	{
		//Get first 6 lines from the txt file
		for (size_t i = 0; i < lines.size(); i++)
			std::getline(fin, lines.at(i));

		//Assign player name which is stored on the final line
		lines.push_back(playerName + "." + std::to_string(score));

		//Separate scores from names for each line
		for (size_t i = 0; i < lines.size() - 1; i++)
			scores.at(i) = std::stoi(lines.at(i).substr(lines.at(i).find(".") + 1));

		std::string previousLine;

		ofstream fout("Data/Leaderboard.txt", ios::out);
		if (fout.fail())
			assert(fout.fail());
		else
		{
			for (size_t i = scores.size() - 1; i < -1; i--)		//Reorders score table if positions change
			{
				if (score >= scores.at(i))
				{
					previousLine = lines.at(i);
					lines.at(i) = lines.at(5);
					if (i < 4)
						lines.at(i + 1) = previousLine;
				}
			}

			for (size_t i = 0; i < scores.size(); i++)		//Outputs back into the txt file
				if (i >= 4)
					fout << lines.at(i);
				else
					fout << lines.at(i) << endl;

			fout.close();
		}
	}
}