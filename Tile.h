#pragma once
#include <vector>
#include "GameObject.h"
#include <random>
#include <time.h>

struct mapData { // STRUCT FOR MAP TEXTURE AND TILE DATA
	int uvIndex; //texture index from texture buffer for tile 
};

class Tile : public GameObject
{

public:

	void Initialize(const std::string& renderItemName);
	void SetRandomPosition();
	void CreateTileWallHorizontal(int clumpSize, DirectX::XMFLOAT3& position, int count);
	void CreateTileWallVertical(int clumpSize, DirectX::XMFLOAT3& position, int count);
	DirectX::XMFLOAT3 SetRandom();
};


class GameTimer;
typedef std::vector<Tile> TileLine;
typedef std::vector<TileLine> TileGid; // 2D Tile Array

class TileManager 
{
	int mDimention = 32;
	const int dimSquare = mDimention * mDimention;
	std::vector<std::vector<mapData>> mMapData;
	const float mUnitSize = 1.0f;
	TileGid  mTileGrid;
	
	const float x = -0.5f*mWidth;
	const float z = -0.5f*mDepth;
	const float dx = mWidth / (mDimention - 1);
	const float dz = mDepth / (mDimention - 1);
	const std::string mRenderItemName = "Tiles";


	const int HAZARD1 = 2;
	const int HAZARD2 = 4;
	const int HAZARD3 = 1;
	int H1 = HAZARD1; // frequency of hazard spots to generate / health
	int H2 = HAZARD2; // frequency of hazard spots to generate / slow
	int H3 = HAZARD3;// frequency of hazard spots to generate / slip
	int H1MaxSize = 3; // maximum size of hazard spot / health - ADVANCED GENERATION
	int H2MaxSize = 3; // maximum size of hazard spot / slow - ADVANCED GENERATION
	int H3MaxSize = 3; // maximum size of hazard spot / slip - ADVANCED GENERATION
	int H1MinSize = 2; // minimum size of hazard spot / health - ADVANCED GENERATION
	int H2MinSize = 2; // minimum size of hazard spot / slow - ADVANCED GENERATION
	int H3MinSize = 2; // minimum size of hazard spot / slip - ADVANCED GENERATION
	int H1Chance = 10; // range out of dimsquare for generation chance / health - ADVANCED GENERATION
	int H2Chance = 10; // range out of dimsquare for generation chance / health - ADVANCED GENERATION
	int H3Chance = 10; // range out of dimsquare for generation chance / health - ADVANCED GENERATION
	int H1Drop = 16; // cripples generation chance by half if x is equal or below this / health - ADVANCED GENERTAION 
	int H2Drop = 16; // cripples generation chance by half if x is equal or below this / health - ADVANCED GENERTAION
	int H3Drop = 16; // cripples generation chance by half if x is equal or below this / health - ADVANCED GENERTAION
	// DEFAULT - TILES OF OWN TYPE DON'T OVERLAP = 0
	// ORDER IS: POISON THEN MUD THEN ICE
	// POISON - NOT OWN						(CAN'T COVER ANYTHING ELSE by design (first in generation))
	// MUD - NOT OWN, POISON, NOT ICE		(CAN COVER POISON)
	// ICE - NOT OWN, NOT POISON, NOT MUD	(CAN'T COVER ANYTHING ELSE)
	// 0 own tiles, 1 means tile before, 2 etc, NEED MORE LATER FOR SPECIFIC HAZARD ORDER
	int H2Priority = 0; // affects positioning of central based on other hazards / slow - ADVANCED GENERATION
	int H3Priority = 2; // affects positioning of central based on other hazards / slip - ADVANCED GENERATION

	int H1Dist = 5; // distance between other hazards from central spot / health - ADVANCED GENERATION
	int H2Dist = 3; // distance between other hazards from central spot / slow - ADVANCED GENERATION
	int H3Dist = 3; // distance between other hazards from central spot / slip - ADVANCED GENERATION
	int H1Random = 50; // randomness attributed to hazard generation / health - ADVANCED GENERATION
	int H2Random = 200; // randomness attributed to hazard generation / slow - ADVANCED GENERATION
	int H3Random = 150; // randomness attributed to hazard generation / slip - ADVANCED GENERATION
	int H1RandomDist = 2; // distance from regular gen / health - ADVANCED GENERATION
	int H2RandomDist = 3; // distance from regular gen / slow - ADVANCED GENERATION
	int H3RandomDist = 3; // distance from regular gen / slip - ADVANCED GENERATION

public:
	const float mWidth = 30.0f;
	const float mHeight = 30.0f;
	const float mDepth = 30.0f;
	int MaxGen;
	void Initialize();
	void Update(const GameTimer& gt);
	Tile& GetTile(int x, int y);
	void SetTile(int x, int y, const Tile& tile);
	int GetIndex(int x, int y); // maybe remove 
	// Allpies mMapData to mTileGrid
	void SetUV( InstanceData* id, int x, int y);

	void REGEN();

	// TODO: (REMEMBER) UPDATE TEXTURES FOR HAZARDS & OTHER TILES HERE WHEN THE LOAD ORDER IS CHANGED
	const int Haz1Tex = 4; // POISON
	const int Haz2Tex = 2; // MUD / SLOW
	const int Haz3Tex = 3; // ICE / SLIP
	int getDimentions();

};


/*

	normal	= 1
			= 2
	mud		= 3
	ice		= 5
	poison	= 6




*/