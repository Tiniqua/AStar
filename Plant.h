#pragma once


#include "GameObject.h"

class GameTimer;

/*
	Plant -> Grow -> Harvert -> Grow -> Harvest ...
	                         -> Dead
*/


class Plant : public GameObject
{
	std::string name;
	enum Stage{DEAD, GROW,HARVEST}; // grow increases size. harvest caps scale at 1
	Stage mStage = Stage::GROW;
	float growRate = 0.3f;
	float regrowChance = 0.5f;
	float scale = 0.2f;
	float color = 1.0f; // flashes when harvestable
public:
	Plant(const PlantData& plantName);
	// apply scale/growth
	void Update(const GameTimer& gt);
	// Returns true if harvest was successful
	bool Harvest();
	float GetScale();
	std::string GetStage();
	
	const std::string& GetName();
	


};

