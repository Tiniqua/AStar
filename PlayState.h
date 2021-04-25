#pragma once
#include "State.h"
//#include "GameObject.h"
#include "CombatController.h"
#include "Common/Camera.h"
#include "Tile.h"
#include "Item.h"
#include <array>
#include "SpriteManager.h"
#include <unordered_map>
#include "Experience.h"
#include "Constants.h"
#include "Trader.h"
#include "Plant.h"
#include "Building.h"
#include <math.h>
#include "Bar.h"
using namespace std;

class PlayState : public State
{
	Player mPlayer;
	PlayerWeapon mPlayerWeapon;
	std::vector<Trader> mTraders;
	std::vector<Enemy> mEnemies;
	std::vector<Plant> mPlants;
	std::vector<Building> mBuildings;

	Inventory mInventory;
	const InventoryUnordered* mpDropItems;
	

	// Trader player is focused on
	Trader* mpActiveTrader = nullptr;
	Inventory::iterator inventoryPosition = mInventory.begin();			//Used to select items from the Inventory (Increments using down key, decrements using up key, loops back around when over inventory size)
	bool itemMenuOpen = false;				//Debug: only allow item selection when on a menu(either pause or an item menu)
	
	// TODO: (REMEMBER) TUTORIAL STUFF HERE
	bool shownAttackTutorial = false;
	bool shownPlantTutorial = false;
	bool shownInventoryUseTutorial = false;

	TileManager mTileManager;
	Tile mTile;
	CombatController mCombatController;

	enum CAMERA_TYPE
	{
		GAME = 0,
		DEBUG = 1,
		SIZE //Always last
	};

	CAMERA_TYPE mCamType = CAMERA_TYPE::GAME;
	std::array<Camera, CAMERA_TYPE::SIZE> mCameras;

	Bar mPlayerHealthBar;

	Panel mInventoryPanel;
	// swaps on toggle and lerps on update
	std::pair<SimpleMath::Vector2, SimpleMath::Vector2> mInventoryLocation;
	Text mInventoryText;

	Text mScoreText;
	Text mScoreTextShadow;

	std::pair<SimpleMath::Vector2, SimpleMath::Vector2> mHelpLocation;
	Panel mHelpPanel;
	Text mHelpText;
	// "Level Up", "You can't go there"
	FadeText mMessage;

	FadeText mHelpMessage;
	FadeText mTradeHelpMessage;
	DeltaTimer revolvingHintTimer;
	int revolvingHintPosition = 3;

	void InitializeTraders();
	bool FindNearestTraderInRadius();
	void StoreScore();
	void GetName();

	Enemy Spawn(std::string enemyType);

	float timeChange = 0.0f; // used to change time
	int timeCycle = 1; // dawn, noon, evening, pitch | changes with area & set number of minutes passed (2?)
	// pitch has constant enemy spawns up to a number(3? increases with each full cycle or every 2 cycles), 
	// evening has 1 large set of enemies (3)
	// noon has 1 small set of enemies (2)
	// dawn has 1 small set of enemies (2)
	bool newEnemy; // create a new enemy to replace the one that has been destroyed.

	int score = 0;
	int areas = 0; 
	std::string playerName = "";

	// Building player is focused on
	Building* mpActiveBuilding = nullptr;
	
	void InitializeBuildings();
	bool FindNearestBuildingInRadius();

public:
	
	PlayState();



	virtual void Initialize() override;
	virtual void Update(const GameTimer& gt)				override;
	virtual void Draw(const GameTimer& gt)					override;
	virtual void Reset()									override;

	virtual void OnMouseDown(WPARAM btnState, int x, int y) override;
	virtual void OnMouseUp(WPARAM btnState, int x, int y)	override;
	virtual void OnMouseMove(WPARAM btnState, int x, int y) override;
	virtual void OnKeyboardInput(const GameTimer& gt)		override;
	virtual void OnResume()									override;
	// Not to be confused with pausing the game
	virtual void OnPause()									override;

	void reInitialize();
	void eGen(bool fill); // bool is only needed for pitch
	void timeSet();

	void Controls(const GameTimer& gt);

	const std::string Loot(const InventoryUnordered* mpDropItems);

	Player* GetPlayer()
	{
		return &mPlayer;
	}

	Trader* GetTrader()
	{
		return mpActiveTrader;
	}

	Inventory* GetInventory()
	{
		return &mInventory;
	}

	Inventory::iterator* GetInventoryPos()
	{
		return &inventoryPosition;
	}

	Building* GetBuilding()
	{
		return mpActiveBuilding;
	}

	std::vector<Building> GetBuildings();

	void InventoryUp();
	void InventoryDown();
	void ItemAction();


	// Only use after changes have been made of inventory. Removes items with 0 quantity.
	void CleanInventory(Inventory& inv);

	const float GetRandomVoicePitch() const
	{
		return GC::VOICE_PITCH[rand()%GC::VOICE_PITCH_COUNT];
	}

	const float GetRandomFootstepPitch() const
	{
		return GC::FOOTSTEP_PITCH[rand() % GC::FOOTSTEP_PITCH_COUNT];
	}

	const float CAM_ANGLE = 45.0f;
	const DirectX::XMFLOAT3 CAM_OFFSET{ 0.0f,10.0f,-10.0f }; 
	Experience mExperience;

	void ReGen(); // REGENERATE TILES

	void UiUpdate(const GameTimer& gt);
	// Returns if planted successful
	bool CreatePlant();
	void UpdatePlayer(const GameTimer& gt);
	void UpdatePlants(const GameTimer& gt);
	void UpdateTiles(const GameTimer& gt);
	void UpdateEnemies(const GameTimer& gt);
	void HarvestByRadius();

	void Keyboard(const GameTimer& gt);
	void KeyboardDebug(const GameTimer& gt);
	void Gamepad(const GameTimer& gt);
	bool TraderStoryComplete();

	int EnemiesRemaining()
	{
		int count = 0;
		for (auto& e : mEnemies)
		{
			if (e.mEnabled) ++count;
		}
		return count;
	}
};

