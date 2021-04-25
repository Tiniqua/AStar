#pragma once
#include <vector>
#include "tile.h";
using namespace std;

class Algorithm
{
private:
	struct sNode
	{
		bool Obstacle = false; // obstacle / unpassable
		bool Visited = false; // node visited
		float GlobalGoal; // distance to goal so far
		float LocalGoal; // alternative distance
		int x; // x position
		int y; // y position
		vector<sNode*>Neighbours; // connections to other neighbours
		sNode* parent; // node connect to this node
	};

	sNode *nodes = nullptr;
	int mapWidth; // TODO: Link to Tile.H
	int mapHeight;

	sNode *nodeStart = nullptr;
	sNode *nodeEnd = nullptr;

public:
	void Initialize();
	void Update();
	void Solve();
	

};
