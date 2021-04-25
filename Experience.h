#pragma once
#include <math.h>

// Experience and Level System
class Experience 
{
	// level threshold
	int mExp;
	int mThreshold;
	int mLevel;

	const int mExponent;
	const int mOffset;

	// Based on level parameter
	int CalcThreshold(int level)
	{
		return (int)pow(level, mExponent) + mOffset;
	}

public:
	Experience(const int& level = 0, const int& exponent = 2, const int& offset = 1, const int& exp = 0)
		:
		mLevel(level),
		mExponent(exponent),
		mOffset(offset),
		mExp(exp)
	{
		mThreshold = CalcThreshold(mLevel + 1);
	}


	void AddExp(const int& e)
	{
		mExp += e;
	}

	// Trigger for level up. Increases level
	bool HasLeveledUp()
	{
		if (mThreshold <= mExp)
		{
			//increase level and change threshold
			mThreshold = CalcThreshold(++mLevel);

			return true;
		}
		else
		{
			return false;
		}
	}

	const int& GetTheshold()
	{
		return mThreshold;
	}

	const int& GetExp()
	{
		return mExp;
	}

};

