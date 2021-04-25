#pragma once

#include <string>
#include <DirectXMath.h>
#include "Item.h"
#include <array>
#include <vector>
#include <map>



typedef std::pair<std::string, std::string> DialogPair;

typedef std::pair<std::string, float> PlantData;

typedef std::pair<std::string, std::string> BuildingData;

namespace GC
{
	//Keyboard keys

	const unsigned char
		// TODO: (REMEMBER) INPUT KEYS HERE
		KEY_FW = 'W',
		KEY_BK = 'S',
		KEY_RT = 'D',
		KEY_LT = 'A',

		// CAMERA ZOOM
		KEY_RAISE = 'O', // OUT
		KEY_LOWER = 'I', // IN

		// PLAYER ACTIONS
		KEY_INVENTORY = 0xD, // ENTER
		KEY_TRADE = 'Q', //TRADE
		KEY_USEITEM = 'E', // USE
		KET_TALK = 'T',
		KEY_ATTACK = 0x20,  //space

		KEY_LOOT = 'L',

		
		//KEY_PAUSE = 0xD, //enter
		//MENU / CAMERA & DEBUG
		KEY_PAUSE = 'P', 
		KEY_CAM = 0x70, // F1
		KEY_HELP = 'H',

		KEY_DEBUG_KILL_ALL = 'M',
		KEY_DEBUG_GENERATE = 'R', // used for map regeneration command
		KEY_DEBUG_ENEMY_POS = 'G',

		KEY_INPUT_NAME = 0xD

		;

	const size_t
		NUM_DIFF_RENDER_OBJS = 7 // for frame resource
		;

	const int
		PLAYER_LOW_HEALTH = 50,
		PLAYER_MAX_HEALTH = 100,

		ENEMYTYPE1_HEALTH = 30, // BARFER
		ENEMYTYPE1_ATTACK = 15,
		ENEMYTYPE2_HEALTH = 20, // RUNNER
		ENEMYTYPE2_ATTACK = 25,

		EXP_EXPONENT = 3,
		EXP_OFFSET = 5,
		EXP_DEFAULT = 5, // test value for increasing exp

		SCORE_PLANT = 150,
		SCORE_ENEMY = 250,
		SCORE_LVLUP = 100,

		HEAL_SMALL = 10,
		HEAL_MED = 50,
		HEAL_LARGE = 100,

		NO_TUTORIAL_VALUE = -999

		;

	const float
		ZOOM_SPEED = 20.0f,
		MOVE_SPEED = 5.0f,

		TRADER_RADIUS = 1.75f,
		BUILDING_RADIUS = 1.75f,

		PLAYER_RIGHTBOUND = 15.0f,
		PLAYER_LEFTBOUND = -15.0f,
		PLAYER_UPBOUND = 15.0f,
		PLAYER_DOWNBOUND = -15.0f,

		PLAYER_ATTACK_DURATION = 0.7f, // MAY NOT BE NEEDED
		PLAYER_ATTACK_DELAY = 0.7f,

		ENEMYTYPE1_RANGE = 8.0f,
		ENEMYTYPE1_ATTACK_DURATION = 3.0f,
		ENEMYTYPE1_ATTACK_DELAY = float(6.0f), // definitely the attack duration - cant actually be less than other
		ENEMYTYPE1_MAXSPEED = 1.0f,
		ENEMYTYPE1_DRAG = 1.0f,

		ENEMYTYPE2_RANGE = 6.0f,
		ENEMYTYPE2_ATTACK_DURATION = 8.0f,
		ENEMYTYPE2_ATTACK_DELAY = float(4.0f), // definitely the attack duration - cant actually be less than other on normal timer
		ENEMYTYPE2_MAXSPEED = 3.0f,
		ENEMYTYPE2_DRAG = 3.0f,

		ALL_ENEMY_MAXRANGE = 50.0f,

		WORLD_LEFT_BOUNDARY = -15.0f,
		WORLD_RIGHT_BOUNDARY = 15.0f,
		WORLD_TOP_BOUNDARY = 15.0f,
		WORLD_BOTTOM_BOUNDARY = -15.0f,

		// weapon rotation stuff
		WEAPONSTART = -40.0f,
		WEAPONEND = 40.0f,

		TILE_UV_INC = 1.0f / 8.0f // tile is 1/8 of texture atlus
		;
	// (NOTE) NO HELP MESSAGE AT CURRENT FOR CAMERA ZOOM
	//KEY_RAISE = 'O', // OUT
	//KEY_LOWER = 'I', // IN
	const std::string HELP_MESSAGES[9]
	{
		"Arrow Keys to scroll \n (E) Key to use an item",
		"Press (Q) to Trade",
		"Use the WASD keys to move and Space to attack",
		"Help is available using the (H) key",
		"Plant crops by selecting seeds from the inventory",
		"You can harvest crops when fully grown by attacking",
		"Try talking to a trader when near using the (Q) key",
		"Attack your enemies using the Spacebar",
		"Pause the game using the (P) key"
	};

	const std::string
		TUTORIAL_INTRO = "Welcome to the end of the world...",
		TUTORIAL_OBJECTIVE = "You must survive to reach SAFEHAVEN by the end of the day.\nKill all mutants in the area before you continue travelling",
		TUTORIAL_MOVE = "To move use the (W)(S)(A)(D) keys\nPause the game using the (P) key",
		TUTORIAL_TRADE = "To trade with other survivors, Press (Q) when near a trader.\n Trading allows you to get new weapons and items.",
		TUTORIAL_INVENTORY = "See what you have in your inventory (Enter)\nCycle through what you have (UP) (DOWN)\nSelect a weapon from the list and it will boost your attack\nClose the the inventory (Enter again)",
		TUTORIAL_ATTACK = "Attack your enemies using the Spacebar\nYou will automatically loot enemies you kill.\n Enemies can drop new weapons, healing items, or farming items.",
		TUTORIAL_USE = "Some items in the inventory can be used (E)",
		TUTORIAL_PLANT = "You have been damaged.\n Plant seeds from the inventory to grow healing items.\n Plants take time to grow so come back to it when its fully grown.";

	const float FOOTSTEP_PITCH[7]
	{
		-0.5f,
		-0.3f,
		-0.1f,
		0.0f,
		0.1f,
		0.3f,
		0.5f
	};
	const size_t FOOTSTEP_PITCH_COUNT = sizeof(GC::FOOTSTEP_PITCH) / sizeof(GC::FOOTSTEP_PITCH[0]);

	const float VOICE_PITCH[5]
	{
		-0.4f,
		-0.2f,
		0.0f,
		0.2f,
		0.4f
	};

	const size_t VOICE_PITCH_COUNT = sizeof(GC::VOICE_PITCH) / sizeof(GC::VOICE_PITCH[0]);

	const std::string
		GO_PLAYER = "Player",
		GO_ENEMY = "Enemy",
		GO_WEAPON = "Weapon",
		GO_TILE = "Tiles",
		GO_TRADER = "Trader",
		GO_POTATO = "Potato",
		GO_BUILDING = "Building", // maybe try switching to trader

		// TODO: (REMEMBER) IMPLEMENT ENEMY TYPES HERE
		// ?? = NOT IMPLEMENTED OR NEEDS REVIEWING
		ENEMY_TYPE_1 = "EnemyType1",
		// BASIC BARFING ENEMY (HOBBLES TOWARD PLAYER THEN BARFS, NO DAMAGE FROM CONTACT??)
		ENEMY_TYPE_2 = "EnemyType2",
		// CHARGER ENEMY (FAST, NO PARTICLE ATTACK)

		STATE_PLAY = "PlayState",
		STATE_TRADE = "TradeState",
		STATE_PAUSE = "PauseMenu",
		STATE_MAINMENU = "MainMenu",
	    STATE_HELP = "HelpMenu",
		STATE_GAMEOVER = "GameOver",
		STATE_VOLUME = "Volume",
		STATE_WIN = "WinState",
		STATE_NEW_AREA = "NewArea1",
		STATE_TUTORIAL = "Tutorial",

		TRADER_NAME_TEST = "TravellerTest",
		TRADER_NAME_1 = "Traveller1",
		TRADER_NAME_2 = "Traveller2",
		TRADER_NAME_3 = "Traveller3",
		TRADER_NAME_4 = "Traveller4",

		TRADER_OBJ_1 = "Car boot storage",
		TRADER_OBJ_2 = "Car seats",
		TRADER_OBJ_3 = "Car engine",

		PLANT_NAME_0 = "Sour Potato",
		PLANT_NAME_1 = "Worm Herb",
		PLANT_NAME_2 = "Skull Rose",

		BUILDING_TYPE_0 = "Abandoned",
		BUILDING_TYPE_1 = "Damaged",
		BUILDING_TYPE_2 = "Pristine",

		BUILDING_RISK_0 = "High",
		BUILDING_RISK_1 = "Medium",
		BUILDING_RISK_2 = "Low"
		;

	const PlantData
		PLANT_0 = { PLANT_NAME_0 , 0.25f }, // 4 secs
		PLANT_1 = { PLANT_NAME_1 , 0.125f }, // 8 secs
		PLANT_2 = { PLANT_NAME_2 , 0.0625f } //16 secs
	;

	const BuildingData
		BUILDING_0 = { BUILDING_TYPE_0 , BUILDING_RISK_0 },
		BUILDING_1 = { BUILDING_TYPE_1 , BUILDING_RISK_1 },
		BUILDING_2 = { BUILDING_TYPE_2 , BUILDING_RISK_2 }
	;

	

	//audio
	const std::string TRADER_TALK[6]
	{
		"trader01",
		"trader02",
		"trader03",
		"trader04",
		"trader05",
		"trader06",
	};


	const size_t TRADER_TALK_COUNT = sizeof(TRADER_TALK) / sizeof(TRADER_TALK[0]);

	const std::string TITLE = "Farming the Wastes";

	const std::string STORY_TEXT[4]
	{
		"It began in the 20th year of the 2nd millenium, on a tuesday, when the plague was \nreleased. The world quickly fell to ruin as it quickly spread & mutated moving between\nhosts, human, animal & plant alike. A few wars & civil uprisings later. And this is the\nworld as we now know it.\nTerrifying I know.",
		"Dawn turns to day...\n\n\n Make use of items you carry.\n Press 'I' to open the menu\n Up/Down Arrows to navigate items\n'U' to use item",
		"The sun hangs low in the sky...\n\n\n Plant seeds from the inventory 'U'.\n They will take a little time to grow.\n Harvest them when they have fully grown 'Space'",
		"And finally the darkness falls...\n\n\n"
	};

	const size_t STORY_TEXT_SIZE = sizeof(STORY_TEXT) / sizeof(STORY_TEXT[0]);

	const DirectX::XMFLOAT2
		MENU_BUTTON_PIVOT{ 0.5f,0.6f },
		MENU_BUTTON_DISTANCE{ 200.0f,100.0f }, // distance from pivot
		MENU_TITLE_POSITION{ 20.0f,20.0f },
		MENU_BODY_POSITION{ 20.0f,50.0f },


		// Add this to uv coords for texture on atlus
		TILE_UV_STONE{ 0.0f , 0.0f },
		TILE_UV_JUNK{ 0.0f , 0.5f },
		TILE_UV_DANGER{ 0.125f , 0.0f },
		TILE_UV_GRASS{ 0.125f , 0.5f },
		TILE_UV_SPIKES{ 0.25f , 0.0f },
		TILE_UV_NOTHING{ 0.25f , 0.5f },
		TILE_UV_EMPTY{ 0.375f , 0.0f },
		TILE_UV_BLANK{ 0.375f , 0.5f }

	;

	const DirectX::XMFLOAT2 TILE_UV[8] =
	{
		{ 0.0f , 0.0f }, // gravel
		{ 0.125f , 0.5f }, // grass
		{ 0.125f , 0.0f }, // danger
		{ 0.0f , 0.5f }, // junk
		{ 0.25f , 0.0f }, // spikes
		{ 0.25f , 0.5f }, // nothing
		{ 0.375f , 0.0f }, // nothing
		{ 0.375f , 0.5f } // nothing
	};


	const size_t NUM_TILE_UV_RAND = 4;

	// Add a random element to tile uvcoords for texture on atlus
	const DirectX::XMFLOAT2 TILE_UV_RANDOM[NUM_TILE_UV_RAND]{
		{	0.000f , 	0.000f },
		{	0.000f , 	0.125f },
		{	0.000f , 	0.250f },
		{	0.000f , 	0.375f }
	};

	// eg stoneTileUV +=  GC::TILE_UV_STONE  + GC::TILE_UV_RANDOM[rand() % GC::NUM_TILE_UV_RAND]
	// eg dangerTileUV += GC::TILE_UV_DANGER + GC::TILE_UV_RANDOM[rand() % GC::NUM_TILE_UV_RAND]


	const DirectX::XMFLOAT3
		DIRECTION_FWD({ 0.0f,	0.0f,	1.0f }),
		DIRECTION_BCK({ 0.0f,	0.0f,	-1.0f }),
		DIRECTION_RGT({ 1.0f,	0.0f,	0.0f }),
		DIRECTION_LFT({ -1.0f,	0.0f,	0.0f }),
		DIRECTION_RAISE({ 0.0f,	1.0f,	0.0f }),
		DIRECTION_LOWER({ 0.0f,	-1.0f,	0.0f })
		;
	const DirectX::XMFLOAT4
		BUTTON_DOWN_COLOR({ 0.7f, 0.7f, 0.7f, 1.0f }),
		BUTTON_UP_COLOR{ 1.0f, 1.0f, 1.0f, 1.0f },

		TEXT_DARK_COLOR{ 0.2f, 0.2f, 0.2f, 1.0f },
		TEXT_LIGHT_COLOR{ 1.0f, 1.0f, 1.0f, 1.0f }
	;


	const long
		// RECT
		PANEL_SRC[4]{ 192, 128,256,192 },

		BAR_MIN{ 32 }, // for source rect limits 0% - 100%
		BAR_MAX{ 160 }, // for source rect limits 0% - 100%
		BAR_BG[4]{ BAR_MIN,128,BAR_MAX,160 }, // Bar background
		BAR_GRN[4]{ BAR_MIN,160,BAR_MAX,192 },
		BAR_YLW[4]{ BAR_MIN,192,BAR_MAX,224 },
		BAR_RED[4]{ BAR_MIN,224,BAR_MAX,256 },
		BAR_SIZE{ BAR_MAX - BAR_MIN } //max - min.
	;

	const DirectX::XMFLOAT4
		DAWN_COLOUR({ 0.12f, 0.226f, 0.12f, 1.0f }),
		NOON_COLOUR({ 0.12f, 0.226f, 0.12f, 1.0f }),
		EVENING_COLOUR({ 0.12f, 0.226f, 0.44f, 1.0f }),
		PITCH_COLOUR({ 0.12f, 0.226f, 0.8f, 1.0f })
	;
	
	
	const DirectX::XMFLOAT3
		DAWN_STRENGTH({ 0.8f, 0.75f, 0.55f }),
		NOON_STRENGTH({ 1.0f, 0.9f, 0.6f }),
		EVENING_STRENGTH({ 0.7f, 0.55f, 0.4f }),
		PITCH_STRENGTH({ 0.1f, 0.1f, 0.1f })
	;



	const ItemMap ITEM_LIST = {
		{"Empty",			{ItemCategory::NONE,	0}}, // 
		{"Radio",			{ItemCategory::KEY_ITEM,  20}},
		{"Leadpipe",		{ItemCategory::WEAPON,  30}},
		{"Nail Bat",		{ItemCategory::WEAPON, 100}},
		{"Scrap Metal",		{ItemCategory::WEAPON,  10}},
		{"Glowing Seeds",	{ItemCategory::SEED,	200}},
		{"Car Engine Part",	{ItemCategory::KEY_ITEM,  200}},
		{"Car Key",			{ItemCategory::KEY_ITEM,  50}},
		{"Gas",				{ItemCategory::KEY_ITEM,  400}},
		{PLANT_NAME_0,		{ItemCategory::HARVESTED_PLANT, 100}},
		{PLANT_NAME_1,		{ItemCategory::HARVESTED_PLANT, 200}},
		{PLANT_NAME_2,		{ItemCategory::HARVESTED_PLANT, 350}},

	};

	const ItemLookup ITEM_LOOKUP_ENEMIES =
	{
		{ "EnemyType1",			{ {"Empty",1},{"Scrap Metal",4}, {"Glowing Seeds",2}}},
		{ "EnemyType2",			{ {"Empty",1},{"Glowing Seeds",3},{"Leadpipe",2}		}},
		{ "EnemyTypeBoss",		{ {"Car Engine Part",1}		,{"Glowing Seeds",3}		}},
	};

	//Trader requests. upto 3
	const ItemLookup ITEM_LOOKUP_REQUEST // TODO: (REMEMBER) TRADER STUFF HERE
	{
		{ TRADER_NAME_TEST,			{{"Glowing Seeds",3 }							}	},
		{ TRADER_NAME_1,			{{PLANT_NAME_0,2 }								}	}, // plants for seeds
		{ TRADER_NAME_2,			{{"Scrap Metal",2},								}   },
		{ TRADER_NAME_3,			{{"Leadpipe",1}, { "Scrap Metal",1}				}   },
		{ TRADER_NAME_4,			{{"Car Engine Part",1}, 						}   }, 
		{ TRADER_OBJ_1,				{{"Car Key",1}									}   }, // car middle
		{ TRADER_OBJ_2,				{{"Car Engine Part",3}							}   }, // car front
		{ TRADER_OBJ_3,				{{"Gas",5}										}   }, // car back
	};

	const ItemLookup ITEM_LOOKUP_REWARD 
	{
		{ TRADER_NAME_TEST,		{{PLANT_NAME_2,4 }									}	},
		{ TRADER_NAME_1,		{{"Glowing Seeds",5 }								}	}, // plants for seeds
		{ TRADER_NAME_2,		{{"Car Engine Part",5}, {"Leadpipe",2}				}	}, 
		{ TRADER_NAME_3,		{{"Nail Bat",1}	,{"Car Key",1}						}	},
		{ TRADER_NAME_4,		{{"Gas",5}											}	}, // no reward, charity
		{ TRADER_OBJ_1,			{													}   }, // car middle
		{ TRADER_OBJ_2,			{													}   }, // car front
		{ TRADER_OBJ_3,			{													}   }, // car back
	};

	const ItemLookup PLANT_LOOKUP_HARVEST
	{
		{PLANT_NAME_0, {{ PLANT_NAME_0,1}}},
		{PLANT_NAME_1, {{ PLANT_NAME_1,1}}},
		{PLANT_NAME_2, {{ PLANT_NAME_2,1}}},
	};


	const std::map<std::string, DialogPair> DIALOG_PAIR 
	{
		{ TRADER_NAME_TEST,	{"Got some rare things to trade, stranger","heh heh heh Thank you"	}},
		{ TRADER_NAME_1,	{"Trade with me","Now on your way"	}},
		{ TRADER_NAME_2,	{"I need some things you need some things","Now I need some peace and quiet and you need to leave"	}}, //
		{ TRADER_NAME_3,	{"Trade with me","Now on your way"	}},  
		{ TRADER_NAME_4, 	{"Trade with me","Now on your way"	}}, // no reward, charity
		{ TRADER_OBJ_3, 	{"There's lots of space to store weapons here.","Locked and loaded"	}},			// rear of car
		{ TRADER_OBJ_2, 	{"I have to load up on supplies for the journey","That should do it"}},			// middle of car
		{ TRADER_OBJ_1, 	{"I need to install these parts to get this moving","Purrs like a kitten"	}}	// front of car
	};


	const std::string RADIO_MESSAGES[3]
	{
		"The voice on the radio gives directions to Safe Haven. The message repeats.",
		"The radio picks up a suspicious distress call.",
		"An enthusiastic trader advertises on the radio."
	};

	const size_t RADIO_MESSAGES_SIZE = sizeof(RADIO_MESSAGES) / sizeof(RADIO_MESSAGES[0]);
}
