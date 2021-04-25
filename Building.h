#pragma once
#include <utility>
#include "Constants.h"
#include "GameObject.h"


// NPCS
class Building : public GameObject
{
	std::string buildingType;
	std::string buildingRiskLevel;

	//const InventoryUnordered* mRequestItems;
	const InventoryUnordered* mRewardItems;

	bool Looted = false;
	bool IsStoryItem = false;

public:
	Building(const BuildingData& bd); // abandoned-infested-occupied etc, risk:low,medium,high (chances to spawn enemys or environmental dmg)
	bool LootBuilding(Inventory& inventory); // WIP




	//std::string GetBuildingType();
	//std::string GetBuildingRiskLevel();
};
