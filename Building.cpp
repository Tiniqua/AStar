#include "Building.h"
#include "Constants.h"
#include <random>

Building::Building(const BuildingData& bd)
{
	buildingType = bd.first;
	buildingRiskLevel = bd.second;
}

bool Building::LootBuilding(Inventory& inventory)
{
	if (Looted == false)
	{
		// adds items
		for (auto& i : *mRewardItems)
		{
			inventory[i.first] += i.second;
		}

		// change state to reflect looting completed
		Looted = true;
		// swap dialog to avoid logic

		return true;
	}
	else
	{
		// Already looted
		return false;
	}

}

bool Looted()
{
	return !Looted;
}
	
//bool GiveLoot(Inventory& inventory)
//{
//
//}



