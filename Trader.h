#pragma once
#include <utility>
#include "Constants.h"
#include "GameObject.h"

// NPCS
class Trader : public GameObject
{
	const InventoryUnordered* mRequestItems;
	const InventoryUnordered* mRewardItems;
	// Swap when traded
	DialogPair mDialog;
	size_t mPitchIndex;
	bool traded = false;
	bool mIsStoryItem = false;

public:

	Trader(const std::string& dialog, const std::string& requestList, const std::string& rewardItems);

	bool CanTrade()
	{
		return !traded;
	}
	// Adds rewards to parameter inventory. Returns false if already traded
	bool GiveRewards(Inventory& inventory)
	{
		if (CanTrade())
		{
			// adds items
			for (auto& i : *mRewardItems)
			{
				inventory[i.first] += i.second;
			}

			// change state to reflect trader completed
			traded = true;
			// swap dialog to avoid logic

			mDialog.first.swap(mDialog.second);

			return true;
		}
		else
		{
			// Already traded
			return false;
		}
	}

	// for player to search for in inventory
	const InventoryUnordered* GetRequestItems()
	{
		return mRequestItems;
	}

	const InventoryUnordered* GetRewardItems()
	{
		return mRewardItems;
	}
	// Returns dialog before and after trading.
	const std::string GetDialog() const
	{
		return mDialog.first;
	}

	const std::string GetRewardAsString()
	{
		std::ostringstream s;

		s << "  Reward";

		std::for_each(mRewardItems->begin(), mRewardItems->end(), [&s](auto& inv)
		{
			s << "\n    ";
			s << inv.first;
			s << "  Qty ";
			s << inv.second;
		});

		return s.str();
	}

	const std::string GetRequestAsString()
	{
		std::ostringstream s;

		s << "Request";

		std::for_each(mRequestItems->begin(), mRequestItems->end(), [&s](auto& inv)
		{
			s << "\n    ";
			s << inv.first;
			s << "  Qty ";
			s << inv.second;

		});

		return s.str();
	}

	float GetPitch()
	{
		return GC::VOICE_PITCH[mPitchIndex];
	}

	void SetAsStoryItem()
	{
		mIsStoryItem = true;
	}

	bool CompletedStoryItem()
	{
		return mIsStoryItem && !CanTrade();
	}

	bool GetIfStoryItem()
	{
		return mIsStoryItem;
	}

	void ResetTrader()
	{
		if (traded)	traded = !traded;		//Allows trader to be traded with again after reset
		mDialog.first.swap(mDialog.second);
	}
};

