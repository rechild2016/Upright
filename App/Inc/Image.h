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

#define SROW 60//�ɼ�ͼ��ĸ�
#define SCOLUMN 160//�ɼ�ͼ��Ŀ�

#define PROW 60//����ͼ��ĸ�
#define PCOLUMN 160//����ͼ��Ŀ�

#define HEART PCOLUMN/2//��������X����
//������
#define MAX_XDIS 10//���������������������
#define MAX_LOST 10//���ϲ����
#define MIN_LINE_LENGTH 10//�߶����Y���򳤶�
#define MIN_CROSS_LENGTH 10//ʮ�ֽ�����С����
#define MAX_SEARCH_HEIGHT 40//��������ߵײ�Y��������
#define CROSS_LENGTH 5

//��������
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
extern int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd, xBase;//����,���ߺ������ߵ�Y������ʼ����ֹ��
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

//ӳ��
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

void StaticThreshold(uint8 tr,uint8 *src);//��̬��ֵ��
//��ֵ������
uint8 Peak(uint8 *src);//˫�巨��̬��ֵ
uint8 Otsu(uint8 *src);//��򷨶�̬��ֵ

void imageProcess(uint8  *src);//ͼ�����ܳ���
extern uint8 tr,staticTr;


void Erosion(uint8 *src) ;

int zhuantouprosses(uint8 *pSrc);

/*/
extern int stopflag;
extern uint32 blackHoleNum;
//Ԥ��
extern uint8 notStopFlag;
extern uint8 needStaticFlag;*/

//Ѱ�߲���
void RecordBWChange(uint8 *src,uint8 *pSrc);//��¼������
//void Record(uint8 *src,uint8 *pSrc);//��ַ���¼������
int SearchBaseLines(uint8 *pSrc);//���ֱ�Ե����
void EdgeTrace(uint8 *src, int line[]);//���ٱ�ԵѰ��
void CompleteLine(int line[]);//����
int RemoveNoise(int mode);//ȥ�����
int StartRecognize(uint8 *src);//��ʼ��ʶ��
//ʮ�ֽ��沿��
void CrossRecognize(uint8 *src,uint8 *pSrc);//ʮ�ֽ����ܺ���
int GetBiasDownRAngle(int line[], int start, int end);//б��ʮ��ȷ����ֱ��
int GetBiasUpRAngle(int line[], int start, int end);//б��ʮ��ȷ����ֱ��
float *GetLSMatchingLine(int line[], int start, int end);//������С�������ֱ��
int JudgeLineType(int line[], int start, int end);//�ж��߶�����
int *ReverseSearchLine(uint8 *pSrc, int line[], int start, float a, float b, int direction);//����˫��Ѱ��
int GetStraightDownRAngle(uint8 *src, int line[], int start, int end, int direction);//ֱ��ʮ��ȷ����ֱ��
int GetStraightUpRAngle(uint8 *src, int line[], int start, int end, int direction);//ֱ��ʮ��ȷ����ֱ��
void CrossMakeUpLine(int line[], int start, int end);//ʮ�ֽ��油��
float CuvreControl(int start,int end);
float circle(int row,int height);
int m_sqrt(int x);
int zwzfilter(int *p);
int AAGAFilter(int *src);
//�����߲���
void CalculateLeadLine(void);//����������

//Һ������
void ArraySetValue(uint8 *src,uint8 value);//����ͳһ
extern void LCD_line_display(Site_t site);
void DFS(int x,int y);
int CarpetSearch(int row);
#endif