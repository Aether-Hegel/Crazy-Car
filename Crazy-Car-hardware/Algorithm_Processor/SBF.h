/**
 * @file SBF.h
 * @brief 推箱子路径规划算法头文件
 * @author RT1064
 * @date 2026
 * 
 * 本模块用于智能车推箱子场景的路径规划:
 * - 基于BFS算法计算最短路径
 * - 支持小车推动箱子从起点到终点的路径规划
 * - 适用于RT1064等嵌入式平台
 */

#ifndef SBF_H
#define SBF_H

#include <stdint.h>

/**
 * @brief 网格地图最大尺寸定义
 * @{
 */
#define GRID_WIDTH_MAX  16   /**< 网格最大宽度 */
#define GRID_HEIGHT_MAX 12   /**< 网格最大高度 */
#define PATH_MAX_LEN    200  /**< 路径最大长度 */
/** @} */

/**
 * @brief 网格单元格类型定义
 * @{
 */
#define GRID_EMPTY      0   /**< 空地，小车和箱子可通行 */
#define GRID_OBSTACLE   1   /**< 障碍物，不可通行 */
#define GRID_BOX        2   /**< 箱子位置 */
#define GRID_TARGET     3   /**< 目标位置 */
/** @} */

/**
 * @brief 网格坐标结构体
 * 
 * 表示网格地图中的二维坐标位置
 */
typedef struct {
    int8_t x;  /**< X坐标(列索引) */
    int8_t y;  /**< Y坐标(行索引) */
} GridPos;

/**
 * @brief 方向枚举类型
 * 
 * 定义小车移动的四个基本方向
 */
typedef enum {
    DIR_UP    = 0,   /**< 向上移动(Y轴减小) */
    DIR_DOWN  = 1,   /**< 向下移动(Y轴增大) */
    DIR_LEFT  = 2,   /**< 向左移动(X轴减小) */
    DIR_RIGHT = 3,   /**< 向右移动(X轴增大) */
    DIR_NONE  = 4    /**< 无方向/停止 */
} Direction;

/**
 * @brief 小车状态结构体
 * 
 * 记录小车的位置和朝向
 */
typedef struct {
    GridPos pos;     /**< 小车当前位置 */
    Direction dir;   /**< 小车当前朝向 */
} CarState;

/**
 * @brief 推动动作结构体
 * 
 * 描述一次推箱子的动作
 */
typedef struct {
    GridPos pos;        /**< 箱子当前位置(推动前) */
    Direction pushDir;   /**< 推动方向 */
} PushAction;

/**
 * @brief 方向偏移量数组(外部可访问)
 * 
 * 用于将方向转换为坐标偏移量
 * - SBF_DirOffset[DIR_UP].x = 0,    SBF_DirOffset[DIR_UP].y = -1
 * - SBF_DirOffset[DIR_DOWN].x = 0,   SBF_DirOffset[DIR_DOWN].y = 1
 * - SBF_DirOffset[DIR_LEFT].x = -1,  SBF_DirOffset[DIR_LEFT].y = 0
 * - SBF_DirOffset[DIR_RIGHT].x = 1,  SBF_DirOffset[DIR_RIGHT].y = 0
 */
extern const GridPos SBF_DirOffset[4];

/**
 * @brief 初始化网格地图
 * 
 * 将整个网格地图初始化为空地(GRID_EMPTY)
 * 
 * @param grid  指向网格地图数组的指针
 *             - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * 
 * @return 无返回值
 * 
 * @note 初始化后所有单元格默认为空地,需要重新设置障碍物、箱子、目标等
 * 
 * @code
 * uint8_t grid[64][64];
 * SBF_GridInit(grid);  // 初始化为空地
 * @endcode
 */
void SBF_GridInit(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]);

/**
 * @brief 检查坐标是否在有效范围内
 * 
 * @param grid  指向网格地图数组的指针(未使用,可为NULL)
 * @param pos   要检查的坐标位置
 *             - 类型: GridPos
 * @param width 网格地图宽度
 *             - 类型: uint8_t
 *             - 范围: 1 ~ GRID_WIDTH_MAX
 * @param height 网格地图高度
 *             - 类型: uint8_t
 *             - 范围: 1 ~ GRID_HEIGHT_MAX
 * 
 * @return uint8_t 检查结果
 *             - 1: 坐标有效,在地图范围内
 *             - 0: 坐标无效,超出地图范围
 * 
 * @code
 * GridPos pos = {5, 3};
 * if (SBF_IsValidPos(pos, 20, 20)) {
 *     // 坐标有效
 * }
 * @endcode
 */
uint8_t SBF_IsValidPos(GridPos pos, uint8_t width, uint8_t height);

/**
 * @brief 设置障碍物
 * 
 * 将指定位置设置为障碍物(GRID_OBSTACLE),小车和箱子不可进入
 * 
 * @param grid  指向网格地图数组的指针
 *             - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param pos   障碍物位置
 *             - 类型: GridPos
 * 
 * @return uint8_t 操作结果
 *             - 1: 设置成功
 *             - 0: 设置失败,坐标超出范围
 * 
 * @code
 * SBF_SetObstacle(grid, (GridPos){5, 5});  // 在(5,5)位置设置障碍物
 * @endcode
 */
uint8_t SBF_SetObstacle(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos);

/**
 * @brief 设置目标位置
 * 
 * 将指定位置设置为目标位置(GRID_TARGET),用于标记箱子需要到达的位置
 * 
 * @param grid  指向网格地图数组的指针
 *             - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param pos   目标位置
 *             - 类型: GridPos
 * 
 * @return uint8_t 操作结果
 *             - 1: 设置成功
 *             - 0: 设置失败,坐标超出范围
 * 
 * @note 目标位置在推箱子过程中可被小车通行,但不能有障碍物
 * 
 * @code
 * SBF_SetTarget(grid, (GridPos){10, 10});  // 设置目标位置
 * @endcode
 */
uint8_t SBF_SetTarget(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos);

/**
 * @brief 设置箱子位置
 * 
 * 将指定位置设置为箱子(GRID_BOX),小车需要推动箱子到达目标位置
 * 
 * @param grid  指向网格地图数组的指针
 *             - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param pos   箱子位置
 *             - 类型: GridPos
 * 
 * @return uint8_t 操作结果
 *             - 1: 设置成功
 *             - 0: 设置失败,坐标超出范围
 * 
 * @note 一个地图只能有一个箱子,多次调用会移动箱子位置
 * 
 * @code
 * SBF_SetBox(grid, (GridPos){5, 3});  // 在(5,3)位置放置箱子
 * @endcode
 */
uint8_t SBF_SetBox(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos);

/**
 * @brief 清除单元格
 * 
 * 将指定位置恢复为空地(GRID_EMPTY)
 * 
 * @param grid  指向网格地图数组的指针
 *             - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param pos   要清除的位置
 *             - 类型: GridPos
 * 
 * @return uint8_t 操作结果
 *             - 1: 清除成功
 *             - 0: 清除失败,坐标超出范围
 * 
 * @code
 * SBF_ClearCell(grid, (GridPos){5, 5});  // 清除障碍物,恢复为空地
 * @endcode
 */
uint8_t SBF_ClearCell(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], GridPos pos);

/**
 * @brief 获取推箱子时小车需要站立的位置
 * 
 * 计算推动箱子时,小车应该在箱子后方的哪个位置
 * 
 * @param boxPos   箱子当前位置
 *                - 类型: GridPos
 * @param pushDir  推动方向
 *                - 类型: Direction
 * @param behindPos 输出的后方位置
 *                - 类型: GridPos*
 * 
 * @return 无返回值
 * 
 * @note 例如:箱子在(5,5),推动方向为DIR_UP,则后方位置为(5,6)
 * 
 * @code
 * GridPos behind;
 * SBF_GetPushBehindPos(boxPos, DIR_UP, &behind);  // 获取推箱子时小车应站位置
 * @endcode
 */
void SBF_GetPushBehindPos(GridPos boxPos, Direction pushDir, GridPos *behindPos);

/**
 * @brief 检查是否可以推动箱子
 * 
 * 判断从指定方向推动箱子是否可行(小车能站到箱子后方,且前方有空间)
 * 
 * @param boxPos  箱子当前位置
 *               - 类型: GridPos
 * @param pushDir 推动方向
 *               - 类型: Direction
 * @param width   网格地图宽度
 *               - 类型: uint8_t
 * @param height  网格地图高度
 *               - 类型: uint8_t
 * 
 * @return uint8_t 检查结果
 *             - 1: 可以推动
 *             - 0: 不可以推动(小车无法到达箱子后方或前方无空间)
 * 
 * @code
 * if (SBF_CanPush(boxPos, DIR_UP, 20, 20)) {
 *     // 可以从上方推动箱子
 * }
 * @endcode
 */
uint8_t SBF_CanPush(GridPos boxPos, Direction pushDir, uint8_t width, uint8_t height);

/**
 * @brief 获取相反方向
 * 
 * @param dir 输入方向
 *           - 类型: Direction
 * 
 * @return Direction 相反方向
 *         - DIR_UP <-> DIR_DOWN
 *         - DIR_LEFT <-> DIR_RIGHT
 *         - DIR_NONE <-> DIR_NONE
 * 
 * @code
 * Direction opposite = SBF_GetReverseDir(DIR_UP);  // 返回DIR_DOWN
 * @endcode
 */
Direction SBF_GetReverseDir(Direction dir);

/**
 * @brief BFS寻路算法
 * 
 * 使用广度优先搜索(BFS)算法计算从起点到目标的最短路径
 * 路径不经过障碍物,返回的路径包含起点和终点
 * 
 * @param grid        指向网格地图数组的指针
 *                    - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 *                    - 注意: GRID_OBSTACLE为不可通行区域
 * @param width       网格地图宽度
 *                    - 类型: uint8_t
 * @param height      网格地图高度
 *                    - 类型: uint8_t
 * @param start       起点位置
 *                    - 类型: GridPos
 * @param target      目标位置
 *                    - 类型: GridPos
 * @param[out] path   输出的路径数组,用于存储计算得到的路径点
 *                    - 类型: GridPos*
 *                    - 需保证数组长度 >= PATH_MAX_LEN
 * @param[out] pathLength 输出的路径长度
 *                    - 类型: uint8_t*
 *                    - 返回路径包含的坐标点数量
 * 
 * @return uint8_t 寻路结果
 *             - 1: 找到路径,path和pathLength已更新
 *             - 0: 未找到路径,起点或终点不可达
 * 
 * @note 使用四方向移动(上下左右),不支持斜向移动
 * @note 路径长度即为最短步数
 * 
 * @code
 * GridPos path[256];
 * uint8_t pathLen;
 * if (SBF_FindPathToTarget(grid, 20, 20, start, target, path, &pathLen)) {
 *     // 找到路径,pathLen为路径长度,path[0]为起点,path[pathLen-1]为终点
 *     for (int i = 0; i < pathLen; i++) {
 *         printf("(%d,%d) ", path[i].x, path[i].y);
 *     }
 * }
 * @endcode
 */
uint8_t SBF_FindPathToTarget(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                            uint8_t width, uint8_t height,
                            GridPos start, GridPos target,
                            GridPos *path, uint8_t *pathLength);

/**
 * @brief 计算推箱子序列
 * 
 * 计算将箱子从起点推到终点的所有推动动作序列
 * 采用贪心策略,每一步都选择使箱子距离目标更近的方向
 * 
 * @param grid        指向网格地图数组的指针
 *                    - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param width       网格地图宽度
 *                    - 类型: uint8_t
 * @param height       网格地图高度
 *                    - 类型: uint8_t
 * @param boxStart    箱子起始位置
 *                    - 类型: GridPos
 * @param target       目标位置
 *                    - 类型: GridPos
 * @param[out] sequence 输出的推动序列数组
 *                    - 类型: PushAction*
 *                    - 需保证数组长度 >= PATH_MAX_LEN
 * @param[out] seqLength 输出的推动序列长度
 *                    - 类型: uint8_t*
 *                    - 返回推动次数
 * 
 * @return uint8_t 计算结果
 *             - 1: 成功计算出推动序列
 *             - 0: 无法计算出有效的推动序列
 * 
 * @note 每个PushAction包含推动前的箱子位置和推动方向
 * @note sequence[i].pushDir表示箱子被推向哪个方向
 * 
 * @code
 * PushAction sequence[256];
 * uint8_t seqLen;
 * SBF_CalcPushSequence(grid, 20, 20, boxStart, target, sequence, &seqLen);
 * for (int i = 0; i < seqLen; i++) {
 *     printf("推动%d: 箱子在(%d,%d), 推向",
 *            i+1, sequence[i].pos.x, sequence[i].pos.y);
 *     switch (sequence[i].pushDir) {
 *         case DIR_UP:    printf("上\n"); break;
 *         case DIR_DOWN:  printf("下\n"); break;
 *         case DIR_LEFT:  printf("左\n"); break;
 *         case DIR_RIGHT: printf("右\n"); break;
 *     }
 * }
 * @endcode
 */
uint8_t SBF_CalcPushSequence(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                            uint8_t width, uint8_t height,
                            GridPos boxStart, GridPos target,
                            PushAction *sequence, uint8_t *seqLength);

/**
 * @brief 计算小车到达推动位置的路径
 * 
 * 计算小车从当前位置移动到推动箱子位置的路径
 * 小车需要先移动到箱子后方,然后准备推动
 * 
 * @param grid        指向网格地图数组的指针
 *                    - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param carStart    小车起始位置
 *                    - 类型: GridPos
 * @param boxPos      箱子当前位置
 *                    - 类型: GridPos
 * @param pushDir     推动方向
 *                    - 类型: Direction
 * @param[out] path   输出的路径数组
 *                    - 类型: GridPos*
 * @param[out] pathLength 输出的路径长度
 *                    - 类型: uint8_t*
 * 
 * @return 无返回值
 * 
 * @note 路径终点是小车应该站立的位置(箱子后方)
 * 
 * @code
 * GridPos path[256];
 * uint8_t pathLen;
 * SBF_CalcCarPathToPush(grid, carPos, boxPos, DIR_UP, path, &pathLen);
 * // 小车沿着path移动到箱子下方,然后向上推动箱子
 * @endcode
 */
void SBF_CalcCarPathToPush(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX], uint8_t width, uint8_t height,
                           GridPos carStart, GridPos boxPos, Direction pushDir,
                           GridPos *path, uint8_t *pathLength);

/**
 * @brief 计算推动后小车的移动路径
 * 
 * 计算推动箱子后小车应该移动到的位置(通常是箱子新位置)
 * 
 * @param grid        指向网格地图数组的指针
 *                    - 类型: uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX]
 * @param boxPos      箱子被推动后的位置
 *                    - 类型: GridPos
 * @param target      箱子最终目标位置
 *                    - 类型: GridPos
 * @param[out] path   输出的路径数组
 *                    - 类型: GridPos*
 * @param[out] pathLength 输出的路径长度
 *                    - 类型: uint8_t*
 * 
 * @return 无返回值
 * 
 * @note 通常小车推动后可以跟随箱子移动到目标附近
 * 
 * @code
 * GridPos path[256];
 * uint8_t pathLen;
 * SBF_CalcCarPathAfterPush(grid, newBoxPos, target, path, &pathLen);
 * @endcode
 */
void SBF_CalcCarPathAfterPush(uint8_t grid[GRID_WIDTH_MAX][GRID_HEIGHT_MAX],
                              GridPos boxPos, GridPos target,
                              GridPos *path, uint8_t *pathLength);

/**
 * @brief 根据坐标变化获取方向
 * 
 * 根据从一个位置到另一个位置的坐标变化,计算出方向
 * 
 * @param from 起始位置
 *            - 类型: GridPos
 * @param to   目标位置
 *            - 类型: GridPos
 * 
 * @return Direction 计算出的方向
 *         - DIR_UP:    to.y < from.y (向上)
 *         - DIR_DOWN:  to.y > from.y (向下)
 *         - DIR_LEFT:  to.x < from.x (向左)
 *         - DIR_RIGHT: to.x > from.x (向右)
 *         - DIR_NONE:  两位置不相邻或坐标相同
 * 
 * @note 只处理相邻两个格子之间的方向
 * 
 * @code
 * GridPos from = {5, 5};
 * GridPos to = {5, 6};
 * Direction dir = SBF_GetDirFromDelta(from, to);  // 返回DIR_DOWN
 * @endcode
 */
Direction SBF_GetDirFromDelta(GridPos from, GridPos to);

/**
 * @brief 获取推动方向的偏移量
 * 
 * 根据推动方向计算推动时坐标的变化量
 * 
 * @param pushDir 推动方向
 *              - 类型: Direction
 * @param[out] dx X方向偏移量输出
 *              - 类型: int8_t*
 * @param[out] dy Y方向偏移量输出
 *              - 类型: int8_t*
 * 
 * @return 无返回值
 * 
 * @code
 * int8_t dx, dy;
 * SBF_GetPushDelta(DIR_RIGHT, &dx, &dy);  // dx=1, dy=0
 * @endcode
 */
void SBF_GetPushDelta(Direction pushDir, int8_t *dx, int8_t *dy);

#endif
