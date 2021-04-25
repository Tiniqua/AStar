#pragma once
using namespace std;
#include <vector>
#include "tile.h"
#include "AStar.h"
#include "GameObject.h"
#include "PlayState.h"

void Algorithm::Initialize()
{
	PlayState ps;
	std::vector<Building> buildingList = ps.GetBuildings();
	std::vector<Enemy> enemyList = ps.GetEnemies();
	Player player = ps.GetPlayerPos();

	TileManager tm;
	mapHeight = tm.Height;
	mapWidth = tm.Width;
	int mDimention = tm.getDimentions();
	
	nodes = new sNode[mDimention * mDimention]; // pull reference from Tile.H
	for (int x = 0; x < mapHeight; x++) // pull reference from GameObject
	{
		for (int y = 0; y < mapWidth; y++)
		{
			nodes[y * mapWidth + x].x = tm.GetTile(x,y).GetPos().x; // may need x and y swapping
			nodes[y * mapWidth + x].y = tm.GetTile(x,y).GetPos().y;
			nodes[y * mapWidth + x].parent = nullptr;
			nodes[y * mapWidth + x].Visited = false;


			for (auto& b : buildingList)
			{
				for (auto& e : enemyList)
				{
					if (tm.GetTile(x, y).CheckCollision(b.GetPos(), e.GetPos()) == true)//tile has building or trader)
					{
						nodes[y * mapWidth + x].Obstacle = true;
					}
					else
					{
						nodes[y * mapWidth + x].Obstacle = false;
					}
				}
			}
			
		}
	}

	for (int x = 0; x <mapHeight; x++) 
	{
		for (int y = 0; y < mapWidth; y++)
		{
			if(y > 0)
				nodes[y * mapWidth + x].Neighbours.push_back(&nodes[y - 1 * mapWidth + (x + 0)]); // north node of this node 
			if(y < mapHeight - 1)
				nodes[y * mapWidth + x].Neighbours.push_back(&nodes[y + 1 * mapWidth + (x + 0)]); // south of this node
			if (x > 0)
				nodes[y * mapWidth + x].Neighbours.push_back(&nodes[y + 0 * mapWidth + (x - 1)]); // west of this node
			if (x < mapHeight - 1)
				nodes[y * mapWidth + x].Neighbours.push_back(&nodes[y + 0 * mapWidth + (x + 1)]); // east of this node
		}
	}

	 

}

void Algorithm::Update()
{
	nodeStart = &nodes[(mapHeight / 2) * mapWidth + 1]; // temp start goal will be set to enemy spawn location
	nodeEnd = &nodes[(mapHeight / 2) * mapWidth + mapWidth - 1]; // temp end goal will be set to player location in update

	if (nodeEnd != nullptr)
	{
		sNode* p = nodeEnd;
		while (p->parent != nullptr)
		{
			p = p->parent; // works backwards from end point to all parents until it reaches the start node where it has no parent and is a nullptr
		}
	}

	Solve();
}

void Algorithm::Solve()
{
	for (int x = 0; x < mapHeight; x++)
	{
		for (int y = 0; y < mapWidth; y++)
		{
			nodes[y * mapWidth + x].Visited = false;
			nodes[y * mapWidth + x].GlobalGoal = INFINITY;
			nodes[y * mapWidth + x].LocalGoal = INFINITY;
			nodes[y * mapWidth + x].parent = nullptr;
		}
	}

	auto distance = [](sNode* a, sNode* b) // pythagoras lambda function to calculate distance to next node
	{
		return sqrtf((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y));
	};

	auto heuristic = [distance](sNode* a, sNode* b) // use for tinkering on algorithm
	{
		return distance(a, b);
	};

	sNode* nodeCurrent = nodeStart; //setup start node
	nodeStart->LocalGoal = 0.0f;
	nodeStart->GlobalGoal = heuristic(nodeStart, nodeEnd);

	list<sNode*> listUntestedNodes; //setup nodes not yet tested and add start
	listUntestedNodes.push_back(nodeStart);

	while (!listUntestedNodes.empty()) // completed
	{
		listUntestedNodes.sort([](const sNode* lhs, const sNode* rhs) { return lhs->GlobalGoal < rhs->GlobalGoal; }); // sort list on globalgoal value lowest to highest

		while (!listUntestedNodes.empty() && listUntestedNodes.front()->Visited) // removes nodes that have been visited
			listUntestedNodes.pop_front();

		if (listUntestedNodes.empty())
		{
			break; // if the list is empty stop the loop 
		}

		nodeCurrent = listUntestedNodes.front(); // best solution at the front 
		nodeCurrent->Visited = true; // prevents searching again

		for (auto nodeNeighbour : nodeCurrent->Neighbours) // check nodes neighbours
		{
			if (!nodeNeighbour->Visited && nodeNeighbour->Obstacle == 0) // if not visited and not an obstacle add to untested nodes list
			{
				listUntestedNodes.push_back(nodeNeighbour);
			}

			// Calculate the neighbours potential lowest parent distance
			float PossiblyLowerGoal = nodeCurrent->LocalGoal + distance(nodeCurrent, nodeNeighbour); // current node local + distance
			if (PossiblyLowerGoal < nodeNeighbour->LocalGoal) // if current local goal + distance is less than neighbour distance
			{
				nodeNeighbour->parent = nodeCurrent; // update neighbour
				nodeNeighbour->LocalGoal = PossiblyLowerGoal; 
				nodeNeighbour->GlobalGoal = nodeNeighbour->LocalGoal + heuristic(nodeNeighbour, nodeEnd); // set neighbours heuristic
			}
		}
	}
}