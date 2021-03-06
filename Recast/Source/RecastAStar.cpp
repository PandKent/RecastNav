#include <float.h>
#define _USE_MATH_DEFINES
#include <ctime>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

static bool buildAStarGridBaseMap(rcContext* ctx, int size, rcCompactHeightfield& chf, rcAStarGrid& asg)
{
	const int w = chf.width;
	const int h = chf.height;
	// size = 10;
	if (size > w || size>h || size <= 0)
	{
		ctx->log(RC_LOG_ERROR,"[Error][RecastAStar][buildAStarGrid] size > w or size > h s %f, w %f, h %f", size, w, h);
		return false;
	}

	//计算grid大小
	int gridsW = 0, gridsH = 0;
	//todo 这里脱裤子放屁了 晚点再改
	int rm = (int)floor(remainder(w,size));
	if (rm != 0)
	{
		gridsW = (int)floor(w/size);	
	}
	else
	{
		gridsW = w/size;
	}
	rm = (int)floor(remainder(h,size));
	if (rm != 0)
	{
		gridsH = (int)floor(h/size);
	}
	else
	{
		gridsH = h/size;
	}
	
	asg.height = gridsH;
	asg.width = gridsW;
	asg.cellSize = size;
	asg.gridsCount = gridsW * gridsH;
	asg.grids = (rcAStarCell*)rcAlloc(sizeof(rcAStarCell)*asg.gridsCount, RC_ALLOC_PERM);
	memset(asg.grids, 0, sizeof(rcAStarCell)*asg.gridsCount);
	
	for (int i = 0; i < gridsH; ++i)//纵
	{
		for (int j = 0; j < gridsW; ++j)//横
		{
			asg.grids[i * gridsW  + j].gridIndex = i * gridsW  + j;
			asg.grids[i * gridsW  + j].chfIndex = (i * w * size) + j * size;
		}
	}

	return true;
}

static bool detectAndBuildGrid(rcContext* ctx, int min, rcCompactHeightfield& chf, rcAStarGrid& asg)
{
	const int w = chf.width;
	const int h = chf.height;
	// min = 30;
	//todo 这里之后可以做分区多线程迭代 数据已经做了分区处理
	for (int grid = 0; grid < asg.gridsCount; ++grid)
	{
		rcAStarCell& cell = asg.grids[grid];
		int start = cell.chfIndex;
		int iterCount = asg.cellSize * asg.cellSize;
		int column = 0, row = 0;
		float minY = -1, maxY = -1, totalY = 0;
		for (int i = 0; i < iterCount; i++)
		{
			if (column == asg.cellSize) //换行
			{
				column = 0;
				row++;
			}
			int index = start + chf.width * row + column;
			
			column++;
			
			rcCompactCell& c = chf.cells[index];

			if (c.count != 0)
			{
				cell.chfCellCount ++ ;
				//高度计算
				float tmp = chf.spans[c.index].y * chf.cs;
				if (minY == -1 || maxY == -1)
				{
					minY = tmp;
					maxY = tmp;
				}
				
				if (tmp < minY)
				{
					minY = tmp;
				}
				else if (tmp > maxY)
				{
					maxY = tmp;
				}
				totalY += tmp;
			}
		}

		if (cell.chfCellCount < min)// < min
		{
			cell.walkable = false;
		}
		else
		{
			cell.walkable = true;
			float average = totalY/cell.chfCellCount;
			if(fabs((minY - average)) >= fabs(maxY - average))
			{
				cell.height = maxY; //趋近于高
			}
			else
			{
				cell.height = minY; //趋近于低
			}
		}
	}
	
	return true;
}

bool rcBuildAStarData(rcContext* ctx, int size, int min,
						rcCompactHeightfield& chf, rcAStarGrid& asg)
{
	if (!buildAStarGridBaseMap(ctx, size,chf,asg))
	{
		return false;
	}
	if (!detectAndBuildGrid(ctx,min,chf,asg))
	{
		return false;
	}
	return true;
}