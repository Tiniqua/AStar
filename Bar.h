#pragma once

#include "SpriteManager.h"
#include <utility>
class GameTimer;

// A horizontal pair to sprites overlapping.
class Bar
{
	std::pair<Sprite,Sprite> mSprites; // first is background, second is forground
	int mMax = 0, mMin = 2, mValue = 3; // need to be set

	void SetSrcRect();


public:
	// setup sprites. Specify forground and background rect
	void Initialise(const long srcRect[4], const long bgScr[4]);

	// validate
	void Update(const GameTimer& gt)
	{
		// Max needs to be bigger than min
		// check limits are set
		assert(mMax >= mMin);

		if (mMax < mValue)
		{
			mValue = mMax;
		}
		else if (mMin > mValue)
		{
			mValue = mMin;
		}

	}

	// draw both sprites
	void Draw()
	{
		mSprites.first.Draw();
		mSprites.second.Draw();
	}

	// set limits before value
	void SetMinMax(int min, int max)
	{
		mMin = min;
		mMax = max;

		SetSrcRect();
	}

	// set value between max and min
	void SetValue(int n)
	{
		mValue = n;
		
		if (mMax < mValue)
		{
			mValue = mMax;
		}
		else if (mMin > mValue)
		{
			mValue = mMin;
		}

		SetSrcRect();
	}

	// position the bar
	void SetPosition(const DirectX::SimpleMath::Vector2& pos)
	{
		mSprites.first.position = pos;
		mSprites.second.position = pos;
	}
};

