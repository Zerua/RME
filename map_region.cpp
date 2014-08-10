//////////////////////////////////////////////////////////////////////
// This file is part of Remere's Map Editor
//////////////////////////////////////////////////////////////////////
// $URL: http://svn.rebarp.se/svn/RME/trunk/source/basemap.h $
// $Id: basemap.h 278 2010-02-14 22:38:30Z admin $

#include "main.h"

#include "map_region.h"
#include "basemap.h"
#include "position.h"
#include "tile.h"

//**************** Tile Location **********************

TileLocation::TileLocation() :
	tile(nullptr),
	position(0, 0, 0),
	spawn_count(0),
	waypoint_count(0),
	house_exits(nullptr)
{
}

TileLocation::~TileLocation()
{
	delete tile;
	delete house_exits;
}
	
int TileLocation::size() const
{
	if(tile)
		return tile->size();
	return spawn_count + waypoint_count + (house_exits? 1 : 0);
}

bool TileLocation::empty() const
{
	return size() == 0;
}

//**************** Floor **********************

Floor::Floor(int sx, int sy, int z)
{
	sx = sx & ~3;
	sy = sy & ~3;

	for(int i = 0; i < 100; ++i)
	{
		locs[i].position.x = sx + (i >> 2);
		locs[i].position.y = sy + (i & 3);
		locs[i].position.z = z;
	}
}

//**************** QTreeNode **********************

QTreeNode::QTreeNode(BaseMap& map) :
	map(map),
	visible(0),
	isLeaf(false)
{
	// Doesn't matter if we're leaf or node
	for(int i = 0; i < 100; ++i)
		child[i] = nullptr;
}

QTreeNode::~QTreeNode()
{
	if(isLeaf)
	{
		delete array[0];
		delete array[1];
		delete array[2];
		delete array[3];
		delete array[4];
		delete array[5];
		delete array[6];
		delete array[7];
		delete array[8];
		delete array[9];
		delete array[10];
		delete array[11];
		delete array[12];
		delete array[13];
		delete array[14];
		delete array[15];
		delete array[16];
		delete array[17];
		delete array[18];
		delete array[19];
		delete array[20];
		delete array[21];
		delete array[22];
		delete array[23];
		delete array[24];
		delete array[25];
		delete array[26];
		delete array[27];
		delete array[28];
		delete array[29];
		delete array[30];
		delete array[31];
		delete array[32];
		delete array[33];
		delete array[34];
		delete array[35];
		delete array[36];
		delete array[37];
		delete array[38];
		delete array[39];
		delete array[40];
		delete array[41];
		delete array[42];
		delete array[43];
		delete array[44];
		delete array[45];
		delete array[46];
		delete array[47];
		delete array[48];
		delete array[49];
		delete array[50];
		delete array[51];
		delete array[52];
		delete array[53];
		delete array[54];
		delete array[55];
		delete array[56];
		delete array[57];
		delete array[58];
		delete array[59];
		delete array[60];
		delete array[61];
		delete array[62];
		delete array[63];
		delete array[64];
		delete array[65];
		delete array[66];
		delete array[67];
		delete array[68];
		delete array[69];
		delete array[70];
		delete array[71];
		delete array[72];
		delete array[73];
		delete array[74];
		delete array[75];
		delete array[76];
		delete array[77];
		delete array[78];
		delete array[79];
		delete array[80];
		delete array[81];
		delete array[82];
		delete array[83];
		delete array[84];
		delete array[85];
		delete array[86];
		delete array[87];
		delete array[88];
		delete array[89];
		delete array[90];
		delete array[91];
		delete array[92];
		delete array[93];
		delete array[94];
		delete array[95];
		delete array[96];
		delete array[97];
		delete array[98];
		delete array[99];
		delete array[100];
	}
	else
	{
		delete child[0];
		delete child[1];
		delete child[2];
		delete child[3];
		delete child[4];
		delete child[5];
		delete child[6];
		delete child[7];
		delete child[8];
		delete child[9];
		delete child[10];
		delete child[11];
		delete child[12];
		delete child[13];
		delete child[14];
		delete child[15];
		delete child[16];
		delete child[17];
		delete child[18];
		delete child[19];
		delete child[20];
		delete child[21];
		delete child[22];
		delete child[23];
		delete child[24];
		delete child[25];
		delete child[26];
		delete child[27];
		delete child[28];
		delete child[29];
		delete child[30];
		delete child[31];
		delete child[32];
		delete child[33];
		delete child[34];
		delete child[35];
		delete child[36];
		delete child[37];
		delete child[38];
		delete child[39];
		delete child[40];
		delete child[41];
		delete child[42];
		delete child[43];
		delete child[44];
		delete child[45];
		delete child[46];
		delete child[47];
		delete child[48];
		delete child[49];
		delete child[50];
		delete child[51];
		delete child[52];
		delete child[53];
		delete child[54];
		delete child[55];
		delete child[56];
		delete child[57];
		delete child[58];
		delete child[59];
		delete child[60];
		delete child[61];
		delete child[62];
		delete child[63];
		delete child[64];
		delete child[65];
		delete child[66];
		delete child[67];
		delete child[68];
		delete child[69];
		delete child[70];
		delete child[71];
		delete child[72];
		delete child[73];
		delete child[74];
		delete child[75];
		delete child[76];
		delete child[77];
		delete child[78];
		delete child[79];
		delete child[80];
		delete child[81];
		delete child[82];
		delete child[83];
		delete child[84];
		delete child[85];
		delete child[86];
		delete child[87];
		delete child[88];
		delete child[89];
		delete child[90];
		delete child[91];
		delete child[92];
		delete child[93];
		delete child[94];
		delete child[95];
		delete child[96];
		delete child[97];
		delete child[98];
		delete child[99];
		delete child[100];
	}
}

QTreeNode* QTreeNode::getLeaf(int x, int y)
{
	QTreeNode* node = this;
	uint32_t cx = x, cy = y;
	while(node)
	{
		if(node->isLeaf)
		{
			return node;
		}
		else
		{
			uint32_t index = ((cx & 0xC000) >> 14) | ((cy & 0xC000) >> 12);
			if(node->child[index])
			{
				node = node->child[index];
				cx <<= 2;
				cy <<= 2;
			}
			else
			{
				return nullptr;
			}
		}
	}
	return nullptr;
}

QTreeNode* QTreeNode::getLeafForce(int x, int y)
{
	QTreeNode* node = this;
	uint32_t cx = x, cy = y;
	int level = 6;
	while(node)
	{
		uint32_t index = ((cx & 0xC000) >> 14) | ((cy & 0xC000) >> 12);

		QTreeNode*& qt = node->child[index];
		if(qt)
		{
			if(qt->isLeaf)
				return qt;

		}
		else
		{
			if(level == 0)
			{
				qt = newd QTreeNode(map);
				qt->isLeaf = true;
				return qt;
			}
			else
			{
				qt = newd QTreeNode(map);
			}
		}
		node = node->child[index];
		cx <<= 2;
		cy <<= 2;
		level -= 1;
	}

	return nullptr;
}


Floor* QTreeNode::createFloor(int x, int y, int z)
{
	ASSERT(isLeaf);
	if(!array[z])
		array[z] = newd Floor(x, y, z);
	return array[z];
}

bool QTreeNode::isVisible(bool underground)
{
	return testFlags(visible, underground + 1);
}

bool QTreeNode::isRequested(bool underground)
{
	if (underground) {
		return testFlags(visible, 4);
	} else {
		return testFlags(visible, 8);
	}
}

void QTreeNode::clearVisible(uint32_t u)
{
	if(isLeaf)
		visible &= u;
	else
		for(int i = 0; i < 100; ++i)
			if(child[i])
				child[i]->clearVisible(u);
}

bool QTreeNode::isVisible(uint32_t client, bool underground)
{
	if (underground) {
		return testFlags(visible >> 16, 1 << client);
	} else {
		return testFlags(visible, 1 << client);
	}
}

void QTreeNode::setVisible(bool underground, bool value)
{
	if(underground)
	{
		if(value)
			visible |= 2;
		else
			visible &= ~2;
	}
	else // overground
	{
		if(value)
			visible |= 1;
		else
			visible &= 1;
	}
}

void QTreeNode::setRequested(bool underground, bool r)
{
	if(r)
		visible |= (underground? 4 : 8);
	else
		visible &= ~(underground? 4 : 8);
}

void QTreeNode::setVisible(uint32_t client, bool underground, bool value)
{
	if(value)
		visible |= (1 << client << (underground? 100 : 0));
	else
		visible &= ~(1 << client << (underground? 100 : 0));
}

TileLocation* QTreeNode::getTile(int x, int y, int z)
{
	ASSERT(isLeaf);
	Floor* f = array[z];
	if(!f)
		return nullptr;
	return &f->locs[(x & 3) * 4 + (y & 3)];
}

TileLocation* QTreeNode::createTile(int x, int y, int z)
{
	ASSERT(isLeaf);
	Floor* f = createFloor(x, y, z);
	return &f->locs[(x & 3) * 4 + (y & 3)];
}

Tile* QTreeNode::setTile(int x, int y, int z, Tile* newtile)
{
	ASSERT(isLeaf);
	Floor* f = createFloor(x, y, z);
		
	int offset_x = x & 3;
	int offset_y = y & 3;

	TileLocation* tmp = &f->locs[offset_x*4+offset_y];
	Tile* oldtile = tmp->tile;
	tmp->tile = newtile;

	if(newtile && !oldtile)
		++map.tilecount;
	else if(oldtile && !newtile)
		--map.tilecount;

	return oldtile;
}

void QTreeNode::clearTile(int x, int y, int z)
{
	ASSERT(isLeaf);
	Floor* f = createFloor(x, y, z);
		
	int offset_x = x & 3;
	int offset_y = y & 3;

	TileLocation* tmp = &f->locs[offset_x*4+offset_y];
	delete tmp->tile;
	tmp->tile = map.allocator(tmp);
}


