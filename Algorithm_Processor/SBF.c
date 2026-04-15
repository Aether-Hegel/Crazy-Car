/**
 * @file SBF.c
 * @brief 推箱子路径规划算法实现
 * @author RT1064
 * @date 2026
 */

#include "SBF.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

/**
 * @brief 内部使用的访问标记数组
 * 
 * 用于BFS算法中标记已访问的格子
 * 静态变量,整个模块共享
 */
static uint8_t visited[GRID_WIDTH_MAX][GRID_HEIGHT_MAX];

/**
 * @brief 内部使用的父节点数组
 * 
 * 用于BFS算法中记录每个节点的父节点,用于路径回溯
 * parent[x][y] 表示从哪个节点到达(x,y)
 */
static GridPos parent[GRID_WIDTH_MAX][GRID_HEIGHT_MAX];

/**
 * @brief 四个方向的坐标偏移量(内部使用)
 * 
 * 用于计算四方向移动时的新坐标:
 * - 0: 上 {0, -1}
 * - 1: 下 {0, +1}
 * - 2: 左 {-1, 0}
 * - 3: 右 {+1, 0}
 */
static const GridPos dirOffset[4] = {
    {0, -1},  {0, 1},  {-1, 0},  {1, 0}
};

/**
 * @brief 四个方向的后方位置偏移量(内部使用)
 * 
 * 用于计算推箱子时小车应站的位置(箱子后方):
 * - 0: DIR_UP    -> {0, +1}  小车在箱子下方
 * - 1: DIR_DOWN  -> {0, -1}  小车在箱子上方
 * - 2: DIR_LEFT  -> {+1, 0}  小车在箱子右方
 * - 3: DIR_RIGHT -> {-1, 0}  小车在箱子左方
 */
static const GridPos behindOffset[4] = {
    {0, 1},   {0, -1}, {1, 0},   {-1, 0}
};

/**
 * @brief 方向偏移量数组(外部可访问)
 * 
 * 供外部模块使用,用于方向到坐标偏移的转换
 */
const GridPos SBF_DirOffset[4] = {
    {0, -1},  {0, 1},  {-1, 0},  {1, 0}
};

/**
 * @brief BFS算法使用的循环队列
 * 
 * 手动实现的循环队列,避免使用动态内存分配
 * 适用于嵌入式系统的固定内存需求
 */
typedef struct {
    GridPos data[PATH_MAX_LEN];   /**< 队列数据存储区 */
    uint8_t front;                 /**< 队首索引 */
    uint8_t rear;                   /**< 队尾索引 */
    uint8_t count;                 /**< 当前队列元素数量 */
} BFSQueue;

/**
 * @brief 初始化循环队列
 * 
 * 将队列置为空状态
 * 
 * @param q 指向队列结构体的指针
 * @return 无返回值
 */
static void BFSQueue_Init(BFSQueue *q) {
    q->front = 0;
    q->rear = 0;
    q->count = 0;
}

/**
 * @brief 检查队列是否为空
 * 
 * @param q 指向队列结构体的指针
 * @return uint8_t 
 *         - 1: 队列为空
 *         - 0: 队列非空
 */
static uint8_t BFSQueue_IsEmpty(BFSQueue *q) {
    return (q->count == 0);
}

/**
 * @brief 检查队列是否已满
 * 
 * @param q 指向队列结构体的指针
 * @return uint8_t 
 *         - 1: 队列已满
 *         - 0: 队列未满
 */
static uint8_t BFSQueue_IsFull(BFSQueue *q) {
    return (q->count >= PATH_MAX_LEN);
}

/**
 * @brief 入队操作
 * 
 * 将一个位置加入队列尾部
 * 
 * @param q 指向队列结构体的指针
 * @param pos 要加入的位置
 * @return uint8_t 
 *         - 1: 入队成功
 *         - 0: 队列已满,入队失败
 */
static uint8_t BFSQueue_Enqueue(BFSQueue *q, GridPos pos) {
    if (BFSQueue_IsFull(q)) return 0;
    q->data[q->rear] = pos;
    q->rear = (q->rear + 1) % PATH_MAX_LEN;
    q->count++;
    return 1;
}

/**
 * @brief 出队操作
 * 
 * 从队列头部取出一个位置
 * 
 * @param q 指向队列结构体的指针
 * @param pos 输出参数,存储取出的位置
 * @return uint8_t 
 *         - 1: 出队成功
 *         - 0: 队列为空,出队失败
 */
static uint8_t BFSQueue_Dequeue(BFSQueue *q, GridPos *pos) {
    if (BFSQueue_IsEmpty(q)) return 0;
    *pos = q->data[q->front];
    q->front = (q->front + 1) % PATH_MAX_LEN;
    q->count--;
    return 1;
}

void SBF_GridInit(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]) {
    /* 遍历整个网格地图,将所有单元格设置为空地 */
    for (uint8_t i = 0; i < GRID_WIDTH_MAX; i++) {
        for (uint8_t j = 0; j < GRID_HEIGHT_MAX; j++) {
            grid[i][j] = GRID_EMPTY;
        }
    }
}

uint8_t SBF_IsValidPos(GridPos pos, uint8_t width, uint8_t height) {
    /* 检查坐标是否在有效范围内: x和y都必须在[0, 边界)内 */
    return (pos.x >= 0 && pos.y >= 0 && pos.x < width && pos.y < height);
}

uint8_t SBF_SetObstacle(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos) {
    /* 先检查坐标有效性,再设置障碍物标记 */
    if (!SBF_IsValidPos(pos, GRID_WIDTH_MAX, GRID_HEIGHT_MAX)) return 0;
    grid[pos.x][pos.y] = GRID_OBSTACLE;
    return 1;
}

uint8_t SBF_SetTarget(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos) {
    /* 先检查坐标有效性,再设置目标位置标记 */
    if (!SBF_IsValidPos(pos, GRID_WIDTH_MAX, GRID_HEIGHT_MAX)) return 0;
    grid[pos.x][pos.y] = GRID_TARGET;
    return 1;
}

uint8_t SBF_SetBox(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos) {
    /* 先检查坐标有效性,再设置箱子标记 */
    if (!SBF_IsValidPos(pos, GRID_WIDTH_MAX, GRID_HEIGHT_MAX)) return 0;
    grid[pos.x][pos.y] = GRID_BOX;
    return 1;
}

uint8_t SBF_ClearCell(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos) {
    /* 先检查坐标有效性,再清除单元格为空地 */
    if (!SBF_IsValidPos(pos, GRID_WIDTH_MAX, GRID_HEIGHT_MAX)) return 0;
    grid[pos.x][pos.y] = GRID_EMPTY;
    return 1;
}

void SBF_GetPushBehindPos(GridPos boxPos, Direction pushDir, GridPos *behindPos) {
    /* 推动方向与后方位置的偏移量是对应的 */
    /* 例如:向上推箱子 -> 小车站在箱子下方 -> behindOffset[DIR_UP] = {0, +1} */
    if (pushDir >= 4) {
        behindPos->x = -1;
        behindPos->y = -1;
        return;
    }
    behindPos->x = boxPos.x + behindOffset[pushDir].x;
    behindPos->y = boxPos.y + behindOffset[pushDir].y;
}

uint8_t SBF_CanPush(GridPos boxPos, Direction pushDir, uint8_t width, uint8_t height) {
    /* 计算推箱子时小车应该站的位置(箱子后方) */
    GridPos behind;
    SBF_GetPushBehindPos(boxPos, pushDir, &behind);
    
    /* 检查小车位置是否在地图内 */
    if (!SBF_IsValidPos(behind, width, height)) return 0;
    
    /* 计算箱子被推动后的位置 */
    GridPos behindBehind;
    behindBehind.x = behind.x + dirOffset[pushDir].x;
    behindBehind.y = behind.y + dirOffset[pushDir].y;
    
    /* 检查箱子移动后的位置是否在地图内 */
    return SBF_IsValidPos(behindBehind, width, height);
}

Direction SBF_GetReverseDir(Direction dir) {
    /* 返回相反方向 */
    switch (dir) {
        case DIR_UP:    return DIR_DOWN;
        case DIR_DOWN:  return DIR_UP;
        case DIR_LEFT:  return DIR_RIGHT;
        case DIR_RIGHT: return DIR_LEFT;
        default:        return DIR_NONE;
    }
}

void SBF_GetPushDelta(Direction pushDir, int8_t *dx, int8_t *dy) {
    /* 根据推动方向设置坐标偏移量 */
    if (pushDir >= 4) {
        *dx = 0;
        *dy = 0;
        return;
    }
    *dx = dirOffset[pushDir].x;
    *dy = dirOffset[pushDir].y;
}

uint8_t SBF_FindPathToTarget(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                            uint8_t width, uint8_t height,
                            GridPos start, GridPos target,
                            GridPos *path, uint8_t *pathLength) {
    BFSQueue queue;
    
    /* 参数有效性检查 */
    if (!SBF_IsValidPos(start, width, height) || !SBF_IsValidPos(target, width, height)) {
        return 0;
    }
    /* 起点或终点不能是障碍物 */
    if (grid[start.x][start.y] == GRID_OBSTACLE) return 0;
    if (grid[target.x][target.y] == GRID_OBSTACLE) return 0;
    
    /* 初始化visited和parent数组 */
    for (uint8_t i = 0; i < width; i++) {
        for (uint8_t j = 0; j < height; j++) {
            visited[i][j] = 0;
            parent[i][j].x = -1;
            parent[i][j].y = -1;
        }
    }
    
    /* 初始化BFS队列,将起点加入 */
    BFSQueue_Init(&queue);
    BFSQueue_Enqueue(&queue, start);
    visited[start.x][start.y] = 1;
    
    /* BFS主循环:逐层扩展搜索 */
    while (!BFSQueue_IsEmpty(&queue)) {
        GridPos current;
        BFSQueue_Dequeue(&queue, &current);
        
        /* 到达目标,开始回溯路径 */
        if (current.x == target.x && current.y == target.y) {
            GridPos tempPath[PATH_MAX_LEN];
            uint8_t idx = 0;
            GridPos temp = target;
            
            /* 从目标点回溯到起点 */
            while (temp.x != -1 && temp.y != -1) {
                tempPath[idx++] = temp;
                if (idx >= PATH_MAX_LEN) break;
                temp = parent[temp.x][temp.y];
            }
            
            /* 反转路径:从起点到终点 */
            for (uint8_t i = 0; i < idx; i++) {
                path[i] = tempPath[idx - 1 - i];
            }
            *pathLength = idx;
            return 1;
        }
        
        /* 探索四个相邻方向 */
        for (uint8_t i = 0; i < 4; i++) {
            GridPos next = {current.x + dirOffset[i].x, current.y + dirOffset[i].y};
            
            /* 检查:坐标有效,未访问,不是障碍物 */
            if (SBF_IsValidPos(next, width, height) &&
                !visited[next.x][next.y] && 
                grid[next.x][next.y] != GRID_OBSTACLE) {
                
                /* 标记为已访问,记录父节点 */
                visited[next.x][next.y] = 1;
                parent[next.x][next.y] = current;
                BFSQueue_Enqueue(&queue, next);
            }
        }
    }
    
    /* 未找到路径 */
    return 0;
}

uint8_t SBF_CalcPushSequence(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                            uint8_t width, uint8_t height,
                            GridPos boxStart, GridPos target,
                            PushAction *sequence, uint8_t *seqLength) {
    GridPos currentBoxPos = boxStart;  /* 当前箱子位置 */
    uint8_t seqIdx = 0;                  /* 推动序列索引 */
    uint8_t maxIterations = width * height;  /* 最大迭代次数,防止死循环 */
    uint8_t iter = 0;
    
    /* 迭代直到箱子到达目标位置 */
    while ((currentBoxPos.x != target.x || currentBoxPos.y != target.y) && iter < maxIterations) {
        iter++;
        uint8_t bestDir = 0;            /* 最佳推动方向 */
        int16_t minDist = 32767;        /* 到目标的最小距离(平方) */
        
        /* 遍历四个方向,选择使箱子距离目标更近的方向 */
        for (uint8_t d = 0; d < 4; d++) {
            /* 检查该方向是否可以推动 */
            if (!SBF_CanPush(currentBoxPos, d, width, height)) continue;
            
            /* 计算推动后箱子的位置 */
            GridPos behind;
            SBF_GetPushBehindPos(currentBoxPos, d, &behind);
            
            /* 计算到目标的曼哈顿距离(使用平方避免开方) */
            GridPos newBoxPos = {currentBoxPos.x + dirOffset[d].x, currentBoxPos.y + dirOffset[d].y};
            int16_t dist = (target.x - newBoxPos.x) * (target.x - newBoxPos.x) + 
                          (target.y - newBoxPos.y) * (target.y - newBoxPos.y);
            
            /* 更新最佳方向(贪心策略) */
            if (dist < minDist) {
                minDist = dist;
                bestDir = d;
            }
        }
        
        /* 没有可用的推动方向 */
        if (minDist == 32767) return 0;
        
        /* 记录推动动作 */
        sequence[seqIdx].pos = currentBoxPos;
        sequence[seqIdx].pushDir = bestDir;
        seqIdx++;
        
        /* 更新箱子位置 */
        currentBoxPos.x += dirOffset[bestDir].x;
        currentBoxPos.y += dirOffset[bestDir].y;
    }
    
    *seqLength = seqIdx;
    /* 检查是否成功将箱子推到目标位置 */
    return (currentBoxPos.x == target.x && currentBoxPos.y == target.y) ? 1 : 0;
}

void SBF_CalcCarPathToPush(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], uint8_t width, uint8_t height,
                           GridPos carStart, GridPos boxPos, Direction pushDir,
                           GridPos *path, uint8_t *pathLength) {
    /* 计算小车应该站的位置(箱子后方) */
    GridPos behind;
    SBF_GetPushBehindPos(boxPos, pushDir, &behind);
    
    /* 使用BFS计算从当前位置到后方位置的路径 */
    SBF_FindPathToTarget(grid, width, height, carStart, behind, path, pathLength);
}

void SBF_CalcCarPathAfterPush(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                              GridPos boxPos, GridPos target,
                              GridPos *path, uint8_t *pathLength) {
    /* 推动后小车通常跟随箱子或移动到目标附近 */
    /* 这里简化为从箱子位置到目标的路径 */
    SBF_FindPathToTarget(grid, GRID_WIDTH_MAX, GRID_HEIGHT_MAX, boxPos, target, path, pathLength);
}

Direction SBF_GetDirFromDelta(GridPos from, GridPos to) {
    int8_t dx = to.x - from.x;  /* X方向变化 */
    int8_t dy = to.y - from.y;  /* Y方向变化 */
    
    /* 根据坐标变化确定方向 */
    if (dx == 0 && dy == -1) return DIR_UP;    /* 向上: y减小 */
    if (dx == 0 && dy == 1)  return DIR_DOWN;   /* 向下: y增大 */
    if (dx == -1 && dy == 0) return DIR_LEFT;   /* 向左: x减小 */
    if (dx == 1 && dy == 0)  return DIR_RIGHT; /* 向右: x增大 */
    return DIR_NONE;  /* 非相邻位置或相同位置 */
}
