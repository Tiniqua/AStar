#pragma once
#include <array>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>

enum ItemCategory { NONE, WEAPON, HARVESTED_PLANT, KEY_ITEM, SEED };

class Item
{
public:
	ItemCategory category;
	int itemSellPrice;			//Could be used if trading is in the game
};

typedef std::map<std::string, Item>							ItemMap;
//item name and quantity.  ++items["x"] adds one to.
typedef std::map<std::string, int>							Inventory;

// for look up tables
// Unordered for item drops rarity. For enemy drops and traders 
typedef std::unordered_map<std::string, int>				InventoryUnordered;
//Looks up from Item item through strings
typedef std::unordered_map<std::string, InventoryUnordered> ItemLookup;

