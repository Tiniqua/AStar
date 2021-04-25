#include "Tile.h"

int TileManager::getDimentions()
{
	return mDimention;
}

void TileManager::Initialize()
{
	MaxGen = mDimention; // set up for tilefinder in playstate for player hazard detection

	srand(static_cast<int>(time(NULL)));

	std::random_device dev;
	std::mt19937 rng(dev()); // seed the generator

	std::uniform_int_distribution<int> grid(0,dimSquare); // 0 to grid size, placement for central hazard
	std::uniform_int_distribution<int> gen1(H1MinSize, H1MaxSize); // size of hazard spot
	std::uniform_int_distribution<int> gen2(H2MinSize, H2MaxSize); // size of hazard spot
	std::uniform_int_distribution<int> gen3(H3MinSize, H3MaxSize); // size of hazard spot

	std::vector<std::vector<int>> coords(mDimention);
	for (int u = 0; u < mDimention; u++) {
		coords[u].resize(mDimention);
	} // setup coords array to grid size
	
	std::vector<std::vector<int>> origin(mDimention); // used to store central tiles only during the first spawn
	for (int u = 0; u < mDimention; u++) {
		origin[u].resize(mDimention);
	} // setup coords array to grid size

	// select positions for central hazard spots at random and store in vector
	// use distance here as well

	// hazard type 1 - find hazard spots and check not within distance of other hazard
	while (H1 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				//int r = rand() % dimSquare; // favors the lower numbers
				int r = grid(rng);  // slightly better
				// IF WITHIN THE LOWER SPECTRUM (LEFT) THEN CUT MAX IN HALF TO LOWER CHANCES OF SELECTION
				if ((r <= int(H1Chance / 2) && H1 > 0 && i <= H1Drop) || (r <= H1Chance && H1 > 0 && i > H1Drop)) { 
					// if tile is selected
					// DISTANCE CHECK HERE
					// main is central square (number to square)
					//			  o
					//	 o		 ooo
					//	ooo		ooooo
					//	 o		 ooo
					//			  o
					bool SAFE = true;
					int Cycles = (H1Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false; 
						if (odd == true) { // larger loop
							for (int x = 0; x < (H1Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								int checkx = (i - H1Dist + Xcycle + x);
								int checky = (o + Ycycle - x);
								if ((i - H1Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H1Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention)
									if (coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
										SAFE = false;
							}
							Xcycle++;
						} // X ISNT CALCULATED PROPERLY, H1DIST + 1 ISN'T WORKING
						if (odd == false) { // smaller loop
							for (int x = 0; x < H1Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								int checkx = (i - H1Dist + Xcycle + x);
								int checky = (o + Ycycle - x);
								if ((i - H1Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H1Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention)
									if (coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
										SAFE = false;
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("grassTex", L"Data/Textures/grass.dds"); // 6
						coords[i][o] = Haz1Tex;
						origin[i][o] = Haz1Tex; // 
						H1 -= 1;
					}
				}
			}
		}
		//H1 -= 1;
	}

	// hazard type 2 - find hazard spots and check not within distance of other hazard
	while (H2 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				//int r = rand() % dimSquare;
				int r = grid(rng); // causes worse problems
				if ((r <= int(H2Chance / 2) && H2 > 0 && i <= H2Drop) || (r <= H2Chance && H2 > 0 && i > H2Drop)) { 
					// if tile is selected
					bool SAFE = true;
					int Cycles = (H2Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false;
						if (odd == true) { // larger loop
							for (int x = 0; x < (H2Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								if ((i - H2Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H2Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) {
									if (H2Priority == 0) // own tiles
										if (coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex)
											SAFE = false;
									if (H2Priority == 1) // prevent covering poison
										if ((coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) || 
											(coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Xcycle++;
						}
						if (odd == false) { // smaller loop
							for (int x = 0; x < H2Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								if ((i - H2Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H2Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) {
									if (H2Priority == 0) // own tiles
										if (coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex)
											SAFE = false;
									if (H2Priority == 1) // can't cover poison
										if ((coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) || 
											(coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("mudTex", L"Data/Textures/LostMeadow_dirt.dds"); // 7
						coords[i][o] = Haz2Tex;
						origin[i][o] = Haz2Tex;
						H2 -= 1;
					}
				}
			}
		}
	}

	// hazard type 3 - find hazard spots and check not within distance of other hazard
	while (H3 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				//int r = rand() % dimSquare;
				int r = grid(rng); // causes a worse problem
				if ((r <= int(H3Chance / 2) && H3 > 0 && i <= H3Drop) || (r <= H3Chance && H3 > 0 && i > H3Drop)) { 
					// if tile is selected
					bool SAFE = true;
					int Cycles = (H3Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false;
						if (odd == true) { // larger loop
							for (int x = 0; x < (H3Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								if ((i - H3Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H3Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) {
									if (H3Priority == 0) // can't cover own tiles
										if (coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex)
											SAFE = false;
									if (H3Priority == 1) // can't cover mud
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex))
											SAFE = false;
									if (H3Priority == 2) // can't cover mud or poison
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Xcycle++;
						}
						if (odd == false) { // smaller loop
							for (int x = 0; x < H3Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								if ((i - H3Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H3Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) {
									if (H3Priority == 0) // own tiles
										if (coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
											SAFE = false;
									if (H3Priority == 1) // can't cover mud
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex))
											SAFE = false;
									if (H3Priority == 2) // can't cover mud or poison
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex))
											SAFE = false;
								}
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("iceTex", L"Data/Textures/ice.dds"); // 4
						coords[i][o] = Haz3Tex;
						origin[i][o] = Haz3Tex;
						H3 -= 1;
					}
				}
			}
		}
		//H1 -= 1;
	}

	// set range of hazard using min & max size and set the nearby tiles as such

	// COULD TRY TO IMPLEMENT RANDOMNESS HERE AS WELL, SAVE SPACE AND 
	
	for (int i = 0; i < mDimention; i++) {
		for (int o = 0; o < mDimention; o++) {
			// use origin to select central tiles & set all tiles within gen 
			if (origin[i][o] == Haz1Tex){ // if central tile found
				int haz1 = gen1(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz1) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz1 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz1 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz1 + Xcycle + x][o + Ycycle - x] = Haz1Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz1; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							int checkx = (i - haz1 + Xcycle + x);
							int checky = (o + Ycycle - x);
							if ((i - haz1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz1 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?
				
				int Rand1 = H1RandomDist + H1Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand1 * 2) + 1; 
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand1 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand1 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand1 + Xcycle + x][o + Ycycle - x] != Haz1Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H1Random)
										coords[i - Rand1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand1; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand1 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand1 + Xcycle + x][o + Ycycle - x] != Haz1Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H1Random)
										coords[i - Rand1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
			// HAZARD 2
			if (origin[i][o] == Haz2Tex) { // if central tile found
				int haz2 = gen2(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz2) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz2 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz2 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz2 + Xcycle + x][o + Ycycle - x] = Haz2Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz2; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							if ((i - haz2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz2 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?

				int Rand2 = H2RandomDist + H2Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand2 * 2) + 1;
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand2 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand2 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand2 + Xcycle + x][o + Ycycle - x] != Haz2Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H2Random)
										coords[i - Rand2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand2; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand2 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand2 + Xcycle + x][o + Ycycle - x] != Haz2Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H2Random)
										coords[i - Rand2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
			// HAZARD 3
			if (origin[i][o] == Haz3Tex) { // if central tile found
				int haz3 = gen3(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz3) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz3 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz3 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz3 + Xcycle + x][o + Ycycle - x] = Haz3Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz3; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							if ((i - haz3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz3 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?

				int Rand3 = H3RandomDist + H3Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand3 * 2) + 1;
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand3 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand3 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand3 + Xcycle + x][o + Ycycle - x] != Haz3Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H3Random)
										coords[i - Rand3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand3; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand3 + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand3 + Xcycle + x][o + Ycycle - x] != Haz3Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H3Random)
										coords[i - Rand3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
		}
	}

	// use randomness attributed to that hazard to activate / deactivate other tiles outside up to range of 2 & 
	// interior tiles, use gen value + 2


	// 4 = ICE, 5 = POISON/MARSH, 6 = MUD

	//setup data array
	for (int k = 0; k < mDimention; ++k)
	{
		mMapData.push_back(std::vector<mapData>());

		for (int j = 0; j < mDimention; ++j)
		{
			mMapData.back().push_back(mapData());

			if (coords[k][j] <= 0)
			{
				mMapData.back().back().uvIndex = (rand() % 2)/* + 1*/; // random plain tile
			}
			else
				mMapData.back().back().uvIndex = coords[k][j]; // set as hazard

			//if (k == 0 || j == 0 || k == 31 || j == 31) { // DEBUG CODE, NEEDS REMOVING AFTER TESTING FINISHED
			//	mMapData.back().back().texIndex = 5;
			//} // 4 = ICE, 5 = POISON/MARSH, 6 = MUD
			//else {
			//	mMapData.back().back().texIndex = rand() % 2; // TODO: NOTE: TEXTURE IS RANDOMIZED HERE
			//}
		}
	}
	// TODO: not sure but tiles may be getting saved into tilegrid twice (as tile & gameobject) if possible

	// create tiles
	mTileGrid.resize(mDimention);
	for (int k = 0; k < mDimention; ++k)
	{
		TileLine tl;
		tl.resize(mDimention);
	
		for (int j = 0; j < mDimention; ++j)
		{
			tl.at(j).Initialize(mRenderItemName); //Creates instances

			// Position instanced along a 3D grid.
			tl.at(j).mpInstance->World = DirectX::XMFLOAT4X4(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				x + j * dx, 0.0f, z + k * dz, 1.0f);
			
			// TODO: (REMEMBER) A SLIGHT ERROR HERE, J & K SHOULD BE SWAPPED, BUT I THINK IT HELPS A LITTLE LIKE THIS
			//Selects random index from array of textures WIP - will be using auto generated clumping 2d array



			SetUV(tl.at(j).mpInstance, j, k);
			//tl.at(j).mpInstance->MaterialIndex = mMapData[j][k].texIndex; // set tile texture to struct data
		}

		mTileGrid.at(k) = tl;
	
	}
}

void TileManager::REGEN() {
	// FIXED ABOVE ISSUE BY USING RAND INSTEAD OF OTHER, TEMP FIX AND SHOULD BE LOOKED INTO LATER IF POSSIBLE
	srand(static_cast<int>(time(NULL)));
	std::mt19937 rng(static_cast<int>(time(NULL))); // seed the generator - 

	H1 = HAZARD1;
	H2 = HAZARD2;
	H3 = HAZARD3;

	for (int j = 0; j < mDimention; j++) {
		for (int k = 0; k < mDimention; k++) {

			mTileGrid.at(j).at(k).mpInstance->MaterialIndex = 0;
			// TODO: (REMEMBER) NUMBER USED MUSTN'T HAVE SIGNIFICANT MEANING FOR TEXTURES 
		}
	}

	std::uniform_int_distribution<int> grid(0, mDimention);
	std::uniform_int_distribution<int> gen1(H1MinSize, H1MaxSize); // uniform, unbiased
	std::uniform_int_distribution<int> gen2(H2MinSize, H2MaxSize); // uniform, unbiased
	std::uniform_int_distribution<int> gen3(H3MinSize, H3MaxSize); // uniform, unbiased

	std::vector<std::vector<int>> coords(mDimention);
	for (int u = 0; u < mDimention; u++) {
		coords[u].resize(mDimention);
	} // setup coords array to grid size

	std::vector<std::vector<int>> origin(mDimention); // used to store central tiles only during the first spawn
	for (int u = 0; u < mDimention; u++) {
		origin[u].resize(mDimention);
	} // setup coords array to grid size

	// select positions for central hazard spots at random and store in vector
	// use distance here as well

	// hazard type 1 - find hazard spots and check not within distance of other hazard
	while (H1 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				int r = rand() % dimSquare; // favors the lower numbers
				//int r = grid(rng);  // slightly better
				// IF WITHIN THE LOWER SPECTRUM (LEFT) THEN CUT MAX IN HALF TO LOWER CHANCES OF SELECTION
				if ((r <= int(H1Chance / 2) && H1 > 0 && i <= H1Drop) ||
					(r <= H1Chance && H1 > 0 && i > H1Drop)) {
					// if tile is selected
					// DISTANCE CHECK HERE
					// main is central square (number to square)
					//			  o
					//	 o		 ooo
					//	ooo		ooooo
					//	 o		 ooo
					//			  o
					bool SAFE = true;
					int Cycles = (H1Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false;
						if (odd == true) { // larger loop
							for (int x = 0; x < (H1Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								int checkx = (i - H1Dist + Xcycle + x);
								int checky = (o + Ycycle - x);
								if ((i - H1Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H1Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention)
									if (coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
										SAFE = false;
							}
							Xcycle++;
						} // X ISNT CALCULATED PROPERLY, H1DIST + 1 ISN'T WORKING
						if (odd == false) { // smaller loop
							for (int x = 0; x < H1Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								int checkx = (i - H1Dist + Xcycle + x);
								int checky = (o + Ycycle - x);
								if ((i - H1Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H1Dist + Xcycle + x) < mDimention && (o + Ycycle - x) < mDimention)
									if (coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
										SAFE = false;
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("grassTex", L"Data/Textures/grass.dds"); // 6
						coords[i][o] = Haz1Tex;
						origin[i][o] = Haz1Tex; // 
						H1 -= 1;
					}
				}
			}
		}
		//H1 -= 1;
	}

	// hazard type 2 - find hazard spots and check not within distance of other hazard
	while (H2 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				int r = rand() % dimSquare;
				//int r = grid(rng); // causes worse problems
				if ((r <= int(H2Chance / 2) && H2 > 0 && i <= H2Drop) ||
					(r <= H2Chance && H2 > 0 && i > H2Drop)) {
					// if tile is selected
					bool SAFE = true;
					int Cycles = (H2Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false;
						if (odd == true) { // larger loop
							for (int x = 0; x < (H2Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								if ((i - H2Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H2Dist + Xcycle + x) < mDimention &&
									(o + Ycycle - x) < mDimention) {
									if (H2Priority == 0) // own tiles
										if (coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex)
											SAFE = false;
									if (H2Priority == 1) // prevent covering poison
										if ((coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H1Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Xcycle++;
						}
						if (odd == false) { // smaller loop
							for (int x = 0; x < H2Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								if ((i - H2Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H2Dist + Xcycle + x) < mDimention &&
									(o + Ycycle - x) < mDimention) {
									if (H2Priority == 0) // own tiles
										if (coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex)
											SAFE = false;
									if (H2Priority == 1) // can't cover poison
										if ((coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H2Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("mudTex", L"Data/Textures/LostMeadow_dirt.dds"); // 7
						coords[i][o] = Haz2Tex;
						origin[i][o] = Haz2Tex;
						H2 -= 1;
					}
				}
			}
		}
	}

	// hazard type 3 - find hazard spots and check not within distance of other hazard
	while (H3 > 0) { // until all hazards have been placed repeat (may cause issue )
		for (int i = 0; i < mDimention; i++) { // x
			for (int o = 0; o < mDimention; o++) { // y
				// calculate random variable to figure out hazard spot
				int r = rand() % dimSquare;
				//int r = grid(rng); // causes a worse problem
				if ((r <= int(H3Chance / 2) && H3 > 0 && i <= H3Drop) ||
					(r <= H3Chance && H3 > 0 && i > H3Drop)) {
					// if tile is selected
					bool SAFE = true;
					int Cycles = (H3Dist * 2) + 1; // max number / for loop
					bool odd = true; // whether or not to run odd or even logic
					int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
					int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
					for (int d = 0; d < Cycles; d++) {
						bool set = false;
						if (odd == true) { // larger loop
							for (int x = 0; x < (H3Dist + 1); x++) {
								// xgrid = i -H1Dist + Xcycle + x
								// ygrid = o + Ycycle - x
								if ((i - H3Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H3Dist + Xcycle + x) < mDimention &&
									(o + Ycycle - x) < mDimention) {
									if (H3Priority == 0) // can't cover own tiles
										if (coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex)
											SAFE = false;
									if (H3Priority == 1) // can't cover mud
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex))
											SAFE = false;
									if (H3Priority == 2) // can't cover mud or poison
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex))
											SAFE = false;
								}
							}
							Xcycle++;
						}
						if (odd == false) { // smaller loop
							for (int x = 0; x < H3Dist; x++) {
								// xgrid = i(-H1Dist+1) + Xcycle + x
								// ygrid = o + Ycyle - x
								if ((i - H3Dist + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
									(i - H3Dist + Xcycle + x) < mDimention &&
									(o + Ycycle - x) < mDimention) {
									if (H3Priority == 0) // own tiles
										if (coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex)
											SAFE = false;
									if (H3Priority == 1) // can't cover mud
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex))
											SAFE = false;
									if (H3Priority == 2) // can't cover mud or poison
										if ((coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz1Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz2Tex) ||
											(coords[i - H3Dist + Xcycle + x][o + Ycycle - x] == Haz3Tex))
											SAFE = false;
								}
							}
							Ycycle++;
						}
						if (odd == true && set == false) {
							odd = false;
							set = true;
						}
						if (odd == false && set == false) {
							odd = true;
							set = true;
						}
					}

					if (SAFE == true) {
						//LoadTexture("iceTex", L"Data/Textures/ice.dds"); // 4
						coords[i][o] = Haz3Tex;
						origin[i][o] = Haz3Tex;
						H3 -= 1;
					}
				}
			}
		}
		//H1 -= 1;
	}

	// set range of hazard using min & max size and set the nearby tiles as such

	// COULD TRY TO IMPLEMENT RANDOMNESS HERE AS WELL, SAVE SPACE AND 

	for (int i = 0; i < mDimention; i++) {
		for (int o = 0; o < mDimention; o++) {
			// use origin to select central tiles & set all tiles within gen 
			if (origin[i][o] == Haz1Tex) { // if central tile found
				int haz1 = gen1(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz1) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz1 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz1 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz1 + Xcycle + x][o + Ycycle - x] = Haz1Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz1; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							int checkx = (i - haz1 + Xcycle + x);
							int checky = (o + Ycycle - x);
							if ((i - haz1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz1 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?

				int Rand1 = H1RandomDist + H1Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand1 * 2) + 1;
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand1 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand1 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand1 + Xcycle + x][o + Ycycle - x] != Haz1Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H1Random)
										coords[i - Rand1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand1; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand1 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand1 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand1 + Xcycle + x][o + Ycycle - x] != Haz1Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H1Random)
										coords[i - Rand1 + Xcycle + x][o + Ycycle - x] = Haz1Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
			// HAZARD 2
			if (origin[i][o] == Haz2Tex) { // if central tile found
				int haz2 = gen2(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz2) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz2 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz2 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz2 + Xcycle + x][o + Ycycle - x] = Haz2Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz2; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							if ((i - haz2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz2 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?

				int Rand2 = H2RandomDist + H2Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand2 * 2) + 1;
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand2 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand2 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand2 + Xcycle + x][o + Ycycle - x] != Haz2Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H2Random)
										coords[i - Rand2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand2; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand2 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand2 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand2 + Xcycle + x][o + Ycycle - x] != Haz2Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H2Random)
										coords[i - Rand2 + Xcycle + x][o + Ycycle - x] = Haz2Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
			// HAZARD 3
			if (origin[i][o] == Haz3Tex) { // if central tile found
				int haz3 = gen3(rng); // random between min & max
				// use for loop to set all tiles within gen1 to hazard
				int Cycles = (int(haz3) * 2) + 1; // max number / for loop
				bool odd = true; // whether or not to run odd or even logic
				int Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				int Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (haz3 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - haz3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz3 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz3 + Xcycle + x][o + Ycycle - x] = Haz3Tex; // sets coord to hazard
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < haz3; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							if ((i - haz3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - haz3 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								coords[i - haz3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
				// RANDOMNESS HERE
				// use H1RandomDist as max distance from central AND randomness is the range within dimsquare to use?

				int Rand3 = H3RandomDist + H3Dist; // set
				// use for loop to set all tiles within gen1 to hazard
				Cycles = (Rand3 * 2) + 1;
				odd = true; // whether or not to run odd or even logic
				Xcycle = 0; // used for x increase at start (increased after each whole odd cycle) 
				Ycycle = 0; // used for y increase at start (increased after each whole even cycle)
				for (int d = 0; d < Cycles; d++) {
					bool set = false;
					if (odd == true) { // larger loop
						for (int x = 0; x < (Rand3 + 1); x++) {
							// xgrid = i -H1Dist + Xcycle + x
							// ygrid = o + Ycycle - x
							if ((i - Rand3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand3 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand3 + Xcycle + x][o + Ycycle - x] != Haz3Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H3Random)
										coords[i - Rand3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
								}
						}
						Xcycle++;
					}
					if (odd == false) { // smaller loop
						for (int x = 0; x < Rand3; x++) {
							// xgrid = i(-H1Dist+1) + Xcycle + x
							// ygrid = o + Ycyle - x
							//int checkx = (i - haz1 + Xcycle + x); // DEBUG
							//int checky = (o + Ycycle - x); // DEBUG
							if ((i - Rand3 + Xcycle + x) >= 0 && (o + Ycycle - x) >= 0 &&
								(i - Rand3 + Xcycle + x) < mDimention &&
								(o + Ycycle - x) < mDimention) // error catcher
								if (coords[i - Rand3 + Xcycle + x][o + Ycycle - x] != Haz3Tex) { // not already a hazard
									// ADJUST HERE AS NEEDED
									int use = (rand() % dimSquare);
									if (use <= H3Random)
										coords[i - Rand3 + Xcycle + x][o + Ycycle - x] = Haz3Tex;
								}
						}
						Ycycle++;
					}
					if (odd == true && set == false) {
						odd = false;
						set = true;
					}
					if (odd == false && set == false) {
						odd = true;
						set = true;
					}
				}
			}
		}
	}

	//setup data array
	for (int k = 0; k < mDimention; ++k) {
		for (int j = 0; j < mDimention; ++j) {

			if (coords[k][j] <= 0)
				mMapData[k][j].uvIndex = (rand() % 2); // random plain tile
			else
				mMapData[k][j].uvIndex = coords[k][j]; // set as hazard



			SetUV(mTileGrid.at(j).at(k).mpInstance, k, j);
			//mTileGrid.at(j).at(k).mpInstance->MaterialIndex = mMapData[k][j].texIndex;

		}
	}
}

void TileManager::Update(const GameTimer & gt)
{
	
}

int TileManager::GetIndex(int x, int y) {
	return mMapData[x][y].uvIndex;
}

void TileManager::SetUV( InstanceData* id,  int x, int y)
{

	// gets uv from map data index with random
	DirectX::SimpleMath::Vector2 uv = GC::TILE_UV[mMapData[x][y].uvIndex];
	// random selection of particular tile
	uv += GC::TILE_UV_RANDOM[rand() % GC::NUM_TILE_UV_RAND];

	// UV to existing texture uv
	id->TexTransform._41 = uv.x;
	id->TexTransform._42 = uv.y;

}

Tile & TileManager::GetTile(int x, int y)
{
	return mTileGrid.at(y).at(x); 
}

void TileManager::SetTile(int x, int y, const Tile & tile)
{
	mTileGrid.at(y).at(x) = tile; 

}
//----------------INDIVIDUAL TILE CODE-----------------------------------------------//

void Tile::Initialize(const std::string& renderItemName)
{
	AddRenderItemInstance("Tiles");
}

void Tile::SetRandomPosition()
{
	int min = -15;
	int max = 15;
	std::random_device rd;								// only used once to initialise (seed) engine
	std::mt19937 rng(rd());								// random-number engine used (Mersenne-Twister in this case)
	std::uniform_int_distribution<int> uni(min, max);	// guaranteed unbiased

	float random_integer = (float)uni(rng);
	float random_integer2 = (float)uni(rng);


	mpInstance->World._41 = random_integer;
	mpInstance->World._42 = 4.0f; // height above ground level
	mpInstance->World._43 = random_integer2;

}

void Tile::CreateTileWallHorizontal(int clumpSize, DirectX::XMFLOAT3& position, int count)
{
	mpInstance->World._41 = position.x + count;
	mpInstance->World._42 = 2.0f; // height above ground level
	mpInstance->World._43 = position.z;
}

void Tile::CreateTileWallVertical(int clumpSize, DirectX::XMFLOAT3& position, int count)
{
	mpInstance->World._41 = position.x;
	mpInstance->World._42 = 2.0f; // height above ground level
	mpInstance->World._43 = position.z + count;
}


DirectX::XMFLOAT3 Tile::SetRandom()
{
	int min = -15;
	int max = 15;
	std::random_device rd;
	std::mt19937 rng(rd());
	std::uniform_int_distribution<int> uni(min, max);

	float random_integer = (float)uni(rng);
	float random_integer2 = (float)uni(rng);


	return DirectX::XMFLOAT3
	{
		random_integer,
		1,
		random_integer2
	};
}