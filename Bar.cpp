#include "Bar.h"
#include "Constants.h"

void Bar::Initialise(const long fgSrc[4], const long bgScr[4])
{
	
	mSprites.first.Initialise("uiTex");
	mSprites.second.Initialise("uiTex");
	
	mSprites.first.sourceRectangle = { bgScr[0] ,bgScr[1] ,bgScr[2] ,bgScr[3] };
	mSprites.second.sourceRectangle = { fgSrc[0] ,fgSrc[1] ,fgSrc[2] ,fgSrc[3] };

}

void Bar::SetSrcRect()
{
	// calculate percentage bar full
	float decimal = (float)(mValue- mMin) / (mMax - mMin);
	// apply scale for ui element
	long right = (long)( decimal * GC::BAR_SIZE);
	// apply to src rect 
	mSprites.second.sourceRectangle.right = GC::BAR_MIN +  right;
}
