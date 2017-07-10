#ifndef __IMAGE_H__
#define __IMAGE_H__

#include "include.h"
#include "image.h"


extern uint8   startgogo;
extern uint8   stopflag;
extern uint8   zhili;
extern uint8   wending;
extern uint8   biansu;
extern uint8   biancan;
extern uint8  zhuantou;
#define Img_H 60
#define Img_W 20

#define SROW 60//采集图像的高
#define SCOLUMN 160//采集图像的宽

#define PROW 60//处理图像的高
#define PCOLUMN 160//处理图像的宽

#define HEART PCOLUMN/2//赛道中心X坐标
//诸多参数
#define MAX_XDIS 10//线上相邻两点最大横向距离
#define MAX_LOST 10//最大断层点数
#define MIN_LINE_LENGTH 10//线段最短Y方向长度
#define MIN_CROSS_LENGTH 10//十字交叉最小长度
#define MAX_SEARCH_HEIGHT 40//最高赛道线底部Y方向坐标
#define CROSS_LENGTH 5

//变量声明
extern uint32 row;

extern uint8 processReady;
extern uint8 wantProcess;
//
extern uint8 Buffer[120];
extern uint8 ImageBuffer1[SROW][SCOLUMN];
extern uint8 ImageBuffer2[SROW][SCOLUMN];
extern uint8 *pSample;
extern uint8 *pProcess;
extern uint8 *pTemp;
extern int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd, xBase;//左线,右线和引导线的Y方向起始及终止点
extern int leftLine[PROW];
extern int rightLine[PROW];
extern int leadLine[PROW];
extern int ShowArray[PROW];
extern int *leady;

extern int A,C,BC,AB,T;
extern  float X;

extern int WIDTH;

extern int leftEdge1[PROW];
extern int rightEdge1[PROW];

//映射
extern uint8 processBuf1[PROW][PCOLUMN];
extern uint8 processBuf2[PROW][PCOLUMN];
extern uint8 *p1;
extern uint8 *p2;
extern int yw;
extern int offset_1;
extern int offset_2;
extern int dOffset;
extern int oldOffset;
extern int newOffset;
extern float Cuvre[4];
int m_sqrt(int x);
void prosseleadLine();

extern float CircleRate;

extern const uint8 mapX[PROW][PCOLUMN];
extern const uint8 mapY[PROW][PCOLUMN];
extern const uint8 leftEdge[PROW];
extern const uint8 rightEdge[PROW];
extern const uint8 weightTable[PROW];
extern int leadlength;
extern int LineType;

void StaticThreshold(uint8 tr,uint8 *src);//静态二值化
//二值化部分
uint8 Peak(uint8 *src);//双峰法动态阈值
uint8 Otsu(uint8 *src);//大津法动态阈值

void imageProcess(uint8  *src);//图像处理总程序
extern uint8 tr,staticTr;


void Erosion(uint8 *src) ;

int zhuantouprosses(uint8 *pSrc);

/*/
extern int stopflag;
extern uint32 blackHoleNum;
//预防
extern uint8 notStopFlag;
extern uint8 needStaticFlag;*/

//寻线部分
void RecordBWChange(uint8 *src,uint8 *pSrc);//记录跳变沿
//void Record(uint8 *src,uint8 *pSrc);//差分法记录跳变沿
int SearchBaseLines(uint8 *pSrc);//两种边缘搜线
void EdgeTrace(uint8 *src, int line[]);//跟踪边缘寻线
void CompleteLine(int line[]);//补线
int RemoveNoise(int mode);//去除噪点
int StartRecognize(uint8 *src);//起始线识别
//十字交叉部分
void CrossRecognize(uint8 *src,uint8 *pSrc);//十字交叉总函数
int GetBiasDownRAngle(int line[], int start, int end);//斜入十字确定下直角
int GetBiasUpRAngle(int line[], int start, int end);//斜入十字确定上直角
float *GetLSMatchingLine(int line[], int start, int end);//计算最小二乘拟合直线
int JudgeLineType(int line[], int start, int end);//判断线段类型
int *ReverseSearchLine(uint8 *pSrc, int line[], int start, float a, float b, int direction);//反向双边寻线
int GetStraightDownRAngle(uint8 *src, int line[], int start, int end, int direction);//直入十字确定下直角
int GetStraightUpRAngle(uint8 *src, int line[], int start, int end, int direction);//直入十字确定上直角
void CrossMakeUpLine(int line[], int start, int end);//十字交叉补线
float CuvreControl(int start,int end);
float circle(int row,int height);
int m_sqrt(int x);
int zwzfilter(int *p);
int AAGAFilter(int *src);
//引导线部分
void CalculateLeadLine(void);//计算引导线

//液晶部分
void ArraySetValue(uint8 *src,uint8 value);//数组统一
extern void LCD_line_display(Site_t site);
void DFS(int x,int y);
int CarpetSearch(int row);
#endif