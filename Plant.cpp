#include <stdlib.h>  
#include <cmath>
#include "Plant.h"
#include "GameApp.h"

Plant::Plant(const PlantData & pd)
{
	name = pd.first;
	growRate = pd.second;
}

void Plant::Update(const GameTimer & gt)
{
	switch (mStage)
	{
	case GROW:
		scale += growRate * gt.DeltaTime(); 
		if (scale >= 1.0f)
			mStage = Stage::HARVEST;
		break;
	case HARVEST:
		color = sinf(gt.TotalTime());
		break;
	case DEAD:
		break;
	}

	SetScale(scale);
}

bool Plant::Harvest()
{
	if (mStage == Stage::HARVEST)
	{
		const float outcome = static_cast <float> (rand()) / static_cast <float> (RAND_MAX); // 0.0f - 1.0f

		scale = 0.2f;
		SetScale(scale);
		if (outcome <= regrowChance)
		{
			mStage = Stage::GROW;
			color = 1.0f;
		}
		else
		{
			mStage = Stage::DEAD;
			color = 0.5f;
		}
		return true;
	}
	else
	{
		return false;
	}
}

float Plant::GetScale()
{
	return scale;
}

std::string Plant::GetStage()
{
	switch (mStage)
	{
	case Stage::DEAD:		return "Dead"; break;
	case Stage::GROW: 		return "Growing"; break;
	case Stage::HARVEST:	return "Ready to Harvest"; break;
	default: return "Error";
	}
}

const std::string& Plant::GetName()
{
	return name;
}
