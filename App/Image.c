#include "include.h"
#include "image.h"
#include "MK60_gpio.h"

#include "arm_math.h"

#define ColConnect(a,b) (a|b)!=255				//列连通 1连通
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//行连通 0为连通


uint8   stopflag=0;
uint8   startgogo=0;
//uint8   zhili=0;
uint8   wending=0;
uint8   biansu=0;
uint8   biancan=0;
uint8  zhuantou=0;

//求连通图
unsigned char ConGraph[Img_H][Img_W] = { 0 };//连通图
uint8 IsConnect[Img_H]={0};
extern uint8 img[60][20];
//采集用的
//uint8 Buffer[120];
//双数组采集用的
uint8 processReady=0;
uint8 wantProcess=0;
//采集用的
uint8 ImageBuffer1[SROW][SCOLUMN];
uint8 ImageBuffer2[SROW][SCOLUMN];

uint8 *pSample=ImageBuffer1[0];
uint8 *pProcess=ImageBuffer2[0];
uint8 *pTemp=ImageBuffer1[0];
//曲线
int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd;//左线,右线和引导线的Y方向起始及终止点
int LineType;  //曲线类型
int leftLine[PROW];
int rightLine[PROW];
int leadLine[PROW];
int ShowArray[PROW];//上位机显示数组
int WIDTH=30;//记录赛道宽度
int leadlength;
int midpoint_before = HEART,midpoint_before_E = HEART;

int offset_1;//此处的offset_1  不可更改变量名为offset，野火库的关键字啊，不然怎么死的都不知道
int offset_2;
int dOffset;
int oldOffset;
int newOffset;
float Cuvre[4];

float CircleRate=0;


uint8 processBuf1[PROW][PCOLUMN];//解压后处理前的
uint8 processBuf2[PROW][PCOLUMN];//处理后的
uint8 *p1=processBuf1[0];
uint8 *p2=processBuf2[0];

/***********************************/
//数组重置数值
void ArraySetValue(uint8 *src,uint8 value)
{
    uint8 i=0,j=0;
    for(i=0;i<PROW;i++)
    {
        for(j=0;j<PCOLUMN;j++)
        {
            *(src+i*PCOLUMN+j)=value;
        }
    }
} 
//加权递推平均滤波
int AAGAFilter(int *src)
{ 
 
   int i=0,Value=0;
   int sum=0,CoeSum=1830;
   int Coefficient[60]; 

   for(i=1;i<61;i++) //  求和
   {
      Coefficient[i]=i;
   }
   
   for(i=0;i<60;i++)
  {
    if(*(src+i)==0){
     CoeSum+=-60+i;
    }
    else
    sum+=*(src+i) * Coefficient[59-i];
   }
    Value=sum/CoeSum;
    return(Value);  
}

/**************************************************************************************/
/*****************************************寻找双线***********************************/
//记录跳变沿
void RecordBWChange(uint8 *src,uint8 *pSrc)//P1 P2
{
    int num1, num2;
   ArraySetValue(pSrc, 255); //重组数组，P2全给0 
    for (int i = 0; i < PROW; i++)//每一行
    {
       for (int j = leftEdge[i]; j <  rightEdge[i]; j++)
        {
            num1 = *(src+i*PCOLUMN+j);
            num2 = *(src+i*PCOLUMN+j+1);
            if (num1 == 0 && num2 == 255)//0是黑，255白
            {
                *(pSrc+i*PCOLUMN+j)= 1;
            }
            else if (num1 == 255 && num2 == 0)
            {
                *(pSrc+i*PCOLUMN+j+1)= 1;
            }
        }
       
 
    }
}



/*****/
//由跳变沿记录数组初步寻找左右线
int leftHalf[PCOLUMN/2];
int rightHalf[PCOLUMN/2];
int leftHalf0[PCOLUMN/2];
int rightHalf0[PCOLUMN/2];

int SearchBaseLines(uint8 *pSrc)//P2
{
    //变量定义
    int num, t1, leftCnt, rightCnt;
    int i,j,m,n;
    //清空之前的寻线数据
    leftYStart = -1; rightYStart = -1; leftYEnd = -1; rightYEnd = -1; leadYStart = -1; leadYEnd = -1;
    zhuantou=0;
    for (i = 0; i < PROW; i++)
    {
        leftLine[i] = 0;
        rightLine[i] = 0;
        leadLine[i] = 0;
    }

    //先搜索最下方若干(40)行，指现实中的行数
    for (i = 0; i < MAX_SEARCH_HEIGHT; i++)
    {
        //向左搜索
        leftCnt = 0;//找到的点数
        for (j = HEART; j > leftEdge[PROW - 1 - i]; j--)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                leftHalf[leftCnt++] = j;//第几个点的列数
                //break;
            }
        }

        //向右搜索
        rightCnt = 0;
        for (j = HEART; j < rightEdge[PROW - 1 - i]; j++)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                rightHalf[rightCnt++] = j;
				//break;
            }
        }
		
        //统计搜索结果
        if (leftCnt == 0 && rightCnt!=0)//右边有线左边没线
        {
            rightYStart = i;
            rightYEnd = i;
            rightLine[i] = rightHalf[0];//第i行最里边的跳变的列数
        }
        else if (rightCnt == 0 && leftCnt != 0)//左边有线右边没线
        {
            leftYStart = i;
            leftYEnd = i;
            leftLine[i] = leftHalf[0];//第i行最里边的跳变的列数
        }
        else if (leftCnt != 0 && rightCnt != 0)//左右都有线
        {
            for (m = 0; m < leftCnt; m++)
            {
                for (n = 0; n < rightCnt; n++)
                {
                    t1 = rightHalf[n] - leftHalf[m];//左右点间距
                    //1cm  为1.3个点
                    //感觉此方法得进行校正后使用  
                    if (t1 < WIDTH + 8 && t1 > WIDTH - 8)//间距在赛道间距范围内  
                    {
                        leftYStart = i;
                        leftYEnd = i;
                        leftLine[i] = leftHalf[m];
                        rightYStart = i;
                        rightYEnd = i;
                        rightLine[i] = rightHalf[n];
                        break;
                    }
                }
                if (leftYStart != -1 && rightYStart != -1)//找到赛道，跳出循环
                {
                    break;
                }
            }
		  
        }
        //搜到离开
        if (leftYStart != -1 || rightYStart != -1)//其中一边搜索到
        {
            if (leftYStart != -1)//如果左边搜到
            {
                if (rightYStart != -1)
                {
                    //两边同时搜到
                    break;
                }
                else
                {
                    //只搜到左边
                    EdgeTrace(pSrc, leftLine);//跟踪左线
                    if (leftYEnd != leftYStart)
                    {
                        CompleteLine(leftLine);//进行补线
                        break;
                    }
                }
            }
            else
            {
                //只搜到右边
                EdgeTrace(pSrc, rightLine);//跟踪右线
                if (rightYEnd != rightYStart)
                {
                    CompleteLine(rightLine);//进行补线
                    break;
                }
            }
        }
    }//所有行搜索结束

    //未搜到丢弃
    if (leftYStart == -1 && rightYStart == -1)
    {
        return -1;
    }
	
    //搜到双边则判断距离是否合格
    if (leftYStart != -1 && rightYStart != -1)
    {
        EdgeTrace(pSrc, leftLine);//跟踪左线
        CompleteLine(leftLine);//补左线
        EdgeTrace(pSrc, rightLine);//跟踪右线
        CompleteLine(rightLine);//补右线
    }
	
    //只搜到左边则先左边跟踪边缘
    else if (leftYStart != -1 && rightYStart == -1)
    {
        //对左线的每一行向右寻线,计算校正后的距离,以此作为黑点是否合格的依据
        for (i = leftYStart; i <= leftYEnd; i++)
        {
            for (j = leftLine[i] + 1; j <rightEdge[i]; j++)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                if (num == 1)
                {
                    if (j - leftLine[i] < WIDTH + 10 && j - leftLine[i] > WIDTH - 10)
                    {
                        rightYStart = i;
                        rightYEnd = i;
                        rightLine[i] = j;
                        break;
                    }
                }
            }
            if (rightYStart != -1)
            {
                EdgeTrace(pSrc, rightLine);//跟踪右线
                CompleteLine(rightLine);//补右线
                break;
            }
        }
    }
    //只搜到右边则先右边跟踪边缘
    else if (leftYStart == -1 && rightYStart != -1)
    {
        //对左线的每一行向右寻线,计算校正后的距离,以此作为黑点是否合格的依据
        for (i = rightYStart; i <= rightYEnd; i++)
        {
            for (j = rightLine[i] - 1; j >leftEdge[i] ; j--)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                if (num == 1)
                {
                    if (rightLine[i] - j < WIDTH + 10 && rightLine[i] - j > WIDTH - 10)
                    {
                        leftYStart = i;
                        leftYEnd = i;
                        leftLine[i] = j;
                        break;
                    }
                }
            }
            if (leftYStart != -1)
            {
                EdgeTrace(pSrc, leftLine);//跟踪左线
                CompleteLine(leftLine);//补左线
                break;
            }
        }
    }
    return 1;
}


/*********************************************/
//跟踪边缘寻线
void EdgeTrace(uint8 *pSrc, int line[])//P2  左线数组
{
    int xMid, xStart, xEnd, num, lost,i,j;
    //左线
    if (line == leftLine)
    {
        //开始由右向左边缘左跟踪
        lost = 0;
        xMid = line[leftYStart];
        for (i = leftYStart + 1; i < PROW; i++)
        {
            //若是上一行找到才以上一行为基准
            if (line[i - 1] != 0)
            {
                xMid = line[i - 1];
            }
			
            //开始与结束
            xStart = xMid + MAX_XDIS;
            if (xStart > rightEdge[PROW - 1 - i]) { xStart = rightEdge[i]; }//不超过校正后的边界
			
            xEnd = xMid - MAX_XDIS;
            if (xEnd < leftEdge[PROW - 1 - i]) { xEnd = leftEdge[i]; }
			
            for (j = xStart; j >= xEnd; j--)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                //找到则记录并且清空丢点计数
                if (num == 1)
                {
                    leftYEnd = i;
                    line[i] = j;
                    lost = 0;
                    break;
                }
            }
            //未找到则丢点计数值加一
            if (leftYEnd != i)
            {
                lost++;
            }
            //丢点次数过多就退出
            if (lost > MAX_LOST)
            {
                return;
            }
        }//循环结束
    }

	
    //右线
    else if (line == rightLine)
    {
        //开始由右向左边缘左跟踪
        lost = 0;
        xMid = line[rightYStart];
        for (i = rightYStart + 1; i < PROW; i++)
        {
            //若是上一行找到才以上一行为基准
            if (line[i - 1] != 0)
            {
                xMid = line[i - 1];
            }
            //开始与结束
            xStart = xMid - MAX_XDIS;
            if (xStart < leftEdge[PROW - 1 - i]) { xStart = leftEdge[i]; }
            xEnd = xMid + MAX_XDIS;
            if (xEnd > rightEdge[PROW - 1 - i]) { xEnd = rightEdge[i]; }
            for (j = xStart; j <= xEnd; j++)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                //找到则记录并且清空丢点计数
                if (num == 1)
                {
                    rightYEnd = i;
                    line[i] = j;
                    lost = 0;
                    break;
                }
            }
            //未找到则丢点计数值加一
            if (rightYEnd != i)
            {
                lost++;
            }
            //丢点次数过多就退出
            if (lost > MAX_LOST)
            {
                return;
            }
        }
    }
}

//补线
void CompleteLine(int line[])
{
    int i, j, start = 0, end = PCOLUMN - 1, num, y1 = 0, y2 = 0, t1, t2;
    float slope;
    if (line == leftLine) { start = leftYStart; end = leftYEnd; }
    else if (line == rightLine) { start = rightYStart; end = rightYEnd; }
    else { start = leadYStart; end = leadYEnd; }
    //对不是0的上下两点按斜率补线
    for (i = start; i <= end; i++)
    {
        num = line[i];
        if (num == 0)//出现丢失点
        {
            if (y1 == 0)
            {
                y1 = i - 1;//y1为丢失前一点纵坐标
            }
        }
        if (num != 0 && y1 != 0)
        {
            y2 = i;//丢失后的第一点纵坐标
            t1 = line[y1];//横坐标
            t2 = line[y2];
            slope = ((float)t2 - t1) / ((float)y2 - y1);
            for (j = y1 + 1; j < y2; j++)
            {
                line[j] = (int)(t1 + slope * (j - y1) + 0.5);//0.5为四舍五入
            }
            y1 = 0;
            y2 = 0;
        }
    }
}


/*************************************************************************************/

/****************************************引导线提取***********************************/
//计算引导线
void CalculateLeadLine()
{
    int y1, y2, y3, y4, yl, yr;
    float divide;
    //只有左线(完全没右线)
    if (leftYStart != -1 && rightYStart == -1)
    {
        leadYStart = leftYStart;
        leadYEnd = leftYEnd;
        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            leadLine[i] = leftLine[i] + WIDTH / 2;
        }
    }
    //只有右线(完全没左线)
    else if (leftYStart == -1 && rightYStart != -1)
    {
        leadYStart = rightYStart;
        leadYEnd = rightYEnd;
        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            leadLine[i] = rightLine[i] - WIDTH / 2;
        }
    }
    //双线
    else if (leftYStart != -1 && rightYStart != -1)//左右都有线
    {
       
        //左盛右衰
        if (leftYEnd - rightYEnd > 18) 
        {
            //FM ( 1) ;    
            //DELAY_MS ( 150 ) ;
            //FM ( 0 ) ;
            leadYStart = leftYStart;
            leadYEnd = leftYEnd;
			
            for (int i = leadYStart; i <= leadYEnd; i++)
            {
                leadLine[i] = leftLine[i] + WIDTH / 2;
            }
        }
        //右盛左衰
        else if (rightYEnd - leftYEnd > 18)
        {
            //FM ( 1) ;    
            //DELAY_MS ( 150 ) ;
            //FM ( 0 ) ;
           
            leadYStart = rightYStart;
            leadYEnd = rightYEnd;
            for (int i = leadYStart; i <= leadYEnd; i++)
            {
                leadLine[i] = rightLine[i] - WIDTH / 2;
            }
        }
        //阴阳协调
        else
        {
            //FM ( 1) ;    
            //DELAY_MS ( 150 ) ;
            //FM ( 0 ) ;
           
            y1 = leftYStart<rightYStart?leftYStart:rightYStart;//S小
            y2 = leftYStart<rightYStart?rightYStart:leftYStart;//S大
            y3 = leftYEnd<rightYEnd?leftYEnd:rightYEnd;//E小
            y4 = leftYEnd<rightYEnd?rightYEnd:leftYEnd;//E大
            divide = (double)(y4 - y2) / (y3 - y2);
            leadYStart = y1;
            leadYEnd = (y3 + y4) / 2;
            for (int i = y1; i < y2; i++)//
            {
                if (y1 == leftYStart)
                {
                    leadLine[i] = leftLine[i] + WIDTH / 2;
                }
                else
                {
                    leadLine[i] = rightLine[i] - WIDTH / 2;
                }
            }
            for (int i = y2; i <= y3; i++)//看不懂，效果上来说，没问题
            {
                if (y4 == leftYEnd)
                {
                    yl = (int)(y2 + (i - y2) * divide);
                    yr = i;
                }
                else
                {
                    yl = i;
                    yr = (int)(y2 + (i - y2) * divide);
                }
                if(yl<PROW && yr<PROW)
                {
                    leadLine[(yl + yr) / 2] = (leftLine[yl] + rightLine[yr]) / 2;
                }
            }
            CompleteLine(leadLine);
        }
    }
}

//去除噪点
int RemoveNoise(int mode)
{ 
    if (mode == 1)
    {
        //线长判断
        if (leftYEnd - leftYStart < MIN_LINE_LENGTH)
        {
            leftYStart = -1;
        }
        if (rightYEnd - rightYStart < MIN_LINE_LENGTH)
        {
            rightYStart = -1;
        }
        if (leftYStart == -1 && rightYStart == -1) { return -1; }
        else { return 1; }
    }
    else if (mode == 2)
    {
        //线长判断
        if (leftYEnd - leftYStart < MIN_CROSS_LENGTH)
        {
            leftYStart = -1;
        }
        if (rightYEnd - rightYStart < MIN_CROSS_LENGTH)
        {
            rightYStart = -1;
        }
        //滤除超过线上相邻两点最大横向距离的线
        for(int i=leftYStart;i<leftYEnd;i++)
        {
            int num=leftLine[i+1]-leftLine[i];
            if(num>MAX_XDIS || num<-MAX_XDIS)
            {
                leftYEnd=i;
            }
        }
        for(int i=rightYStart;i<rightYEnd;i++)
        {
            int num=rightLine[i+1]-rightLine[i];
            if(num>MAX_XDIS || num<-MAX_XDIS)
            {
                rightYEnd=i;
            }
        }
        if (leftYStart == -1 && rightYStart == -1) { return -1; }
        else { return 1; }
    }
    return 1;
}
/*************************************************************************************/
/***************************************十字交叉识别**********************************/
//十字交叉总函数
void CrossRecognize(uint8 *src,uint8 *pSrc)//P1 P2
{
    int y2 = -1;
    //斜入十字判断
    //左线
    if (leftYStart != -1)
    {
        y2 = GetBiasDownRAngle(leftLine, leftYStart, leftYEnd);//判断是否是斜入十字
        if (y2 != -1)//是斜入十字
        {
            //完成左线 Y2为转折点
            float *ab = GetLSMatchingLine(leftLine, leftYStart, y2);
            ab[1] += WIDTH / 2;//平移
            int *lys = ReverseSearchLine(pSrc, leftLine, leftYStart, ab[0], ab[1], 1);
            if (lys[0] != -1 && lys[1] != -1)
            {
                leftYEnd = lys[1];
                CrossMakeUpLine(leftLine, leftYStart, leftYEnd);
            }
            else
            {
                leftYEnd = y2;
            }
            //兼济右线
            if (rightYStart != -1)
            {
                int *rys = ReverseSearchLine(pSrc, rightLine, rightYEnd, ab[0], ab[1], 2);
                if (rys[0] != -1 && rys[1] != -1)
                {
                    rightYEnd = rys[1];
                    GetBiasUpRAngle(rightLine, rys[0], rys[1]);
                    CrossMakeUpLine(rightLine, rightYStart, rightYEnd);
                }
            }
            return;
        }
    }
    //右线
    if (y2 == -1 && rightYStart != -1)
    {
        y2 = GetBiasDownRAngle(rightLine, rightYStart, rightYEnd);
        if (y2 != -1)
        {
            //完成右线
            float *ab = GetLSMatchingLine(rightLine, rightYStart, y2);
            ab[1] -= WIDTH / 2;
            int *rys = ReverseSearchLine(pSrc, rightLine, rightYStart, ab[0], ab[1], 2);
            if (rys[0] != -1 && rys[1] != -1)
            {
                rightYEnd = rys[1];
                CrossMakeUpLine(rightLine, rightYStart, rightYEnd);
            }
            else
            {
                rightYEnd = y2;
            }
            //兼济左线
            if (leftYStart != -1)
            {
                int *lys = ReverseSearchLine(pSrc, leftLine, leftYEnd, ab[0], ab[1], 1);
                if (lys[0] != -1 && lys[1] != -1)
                {
                    leftYEnd = lys[1];
                    GetBiasUpRAngle(leftLine, lys[0], lys[1]);
                    CrossMakeUpLine(leftLine, leftYStart, leftYEnd);
                }
            }
            return;
        }
    }
	
    //直入十字判断
    int lt, ly2 = -1, ry2 = -1;
    //左线
    if (leftYStart != -1)
    {
        ly2 = GetStraightDownRAngle(src, leftLine, leftYStart, leftYEnd, 1);
        if (ly2 != -1)
        {
            lt = JudgeLineType(leftLine, leftYStart, ly2);
            if (lt == 2)
            {
                return;
            }
            else 
            {
                for (int i = ly2 + 1; i <= leftYEnd; i++)
                {
                    leftLine[i] = 0;
                }
            }
            float* ab = GetLSMatchingLine(leftLine, leftYStart, ly2);
            ab[1] += WIDTH / 2;
            int *lys = ReverseSearchLine(pSrc, leftLine, leftYEnd, ab[0], ab[1], 1);
            if (lys[0] != -1 && lys[1] != -1)
            {
                leftYEnd = lys[1];
                GetStraightUpRAngle(src, leftLine, lys[0], lys[1], 1);
                CrossMakeUpLine(leftLine, leftYStart, leftYEnd);
            }
            else
            {
                leftYEnd = ly2;
            }
            //兼济右线
            if (rightYStart != -1)
            {
                ry2 = GetStraightDownRAngle(src, rightLine, rightYStart, rightYEnd, 2);
                int *rys = ReverseSearchLine(pSrc, rightLine, rightYEnd, ab[0], ab[1], 2);
                if (rys[0] != -1 && rys[1] != -1)
                {
                    rightYEnd = rys[1];
                    GetStraightUpRAngle(src, rightLine, rys[0], rys[1], 2);
                    CrossMakeUpLine(rightLine, rightYStart, rightYEnd);
                }
                else
                {
                    if (ry2 != -1) { rightYEnd = ry2; }
                }
            }
            return;
        }
    }
    //右线
    if (ly2 == -1 && rightYStart != -1)
    {
        ry2 = GetStraightDownRAngle(src, rightLine, rightYStart, rightYEnd, 2);
        if (ry2 != -1)
        {
            lt = JudgeLineType(rightLine, rightYStart, ry2);
            if (lt == 2)
            {
                return;
            }
            else
            {
                for (int i = ry2 + 1; i <= rightYEnd; i++)
                {
                    rightLine[i] = 0;
                }
            }
            float *ab = GetLSMatchingLine(rightLine, rightYStart, ry2);
            ab[1] -= WIDTH / 2;
            int *rys = ReverseSearchLine(pSrc, rightLine, rightYEnd, ab[0], ab[1], 2);
            if (rys[0] != -1 && rys[1] != -1)
            {
                rightYEnd = rys[1];
                GetStraightUpRAngle(src, rightLine, rys[0], rys[1], 2);
                CrossMakeUpLine(rightLine, rightYStart, rightYEnd);
            }
            else
            {
                rightYEnd = ry2;
            }
            //兼济左线
            if (leftYStart != -1)
            {
                ly2 = GetStraightDownRAngle(src, leftLine, leftYStart, leftYEnd, 1);
                int *lys = ReverseSearchLine(pSrc, leftLine, leftYEnd, ab[0], ab[1], 1);
                if (lys[0] != -1 && lys[1] != -1)
                {
                    leftYEnd = lys[1];
                    GetStraightUpRAngle(src, leftLine, lys[0], lys[1], 1);
                    CrossMakeUpLine(leftLine, leftYStart, leftYEnd);
                }
                else
                {
                    if (ly2 != -1) { leftYEnd = ly2; }
                }
            }
            return;
        }
    }
}

//斜入十字确定下直角
int GetBiasDownRAngle(int line[], int start, int end)//leftLine, leftYStart, leftYEnd
{
    //1为从下向上,-1为从上向下
    int m1, m2, mul;
    //直角检测(斜入十字)
    for (int i = start + CROSS_LENGTH; i <= end - CROSS_LENGTH; i++)
    {
        m1 = line[i] - line[i - CROSS_LENGTH];
        m2 = line[i + CROSS_LENGTH] - line[i];
        mul = m1 * m2;
        if (mul < -CROSS_LENGTH * CROSS_LENGTH)
        {
            //斜入十字,转折点后清零
            for (int j = i + 1; j <= end; j++)
            {
                line[j] = 0;
            }
           
            return i;
        }
    }
    return -1;
}

//斜入十字确定上直角
int GetBiasUpRAngle(int line[], int start, int end)
{
    //1为从下向上,-1为从上向下
    int m1, m2, mul;
    //直角检测(斜入十字)
    for (int i = end - CROSS_LENGTH; i >= start + CROSS_LENGTH; i--)
    {
        m1 = line[i] - line[i - CROSS_LENGTH];
        m2 = line[i + CROSS_LENGTH] - line[i];
        mul = m1 * m2;
        if (mul < -CROSS_LENGTH * CROSS_LENGTH)
        {
            //斜入十字
            for (int j = i - 1; j >= start; j--)
            {
                line[j] = 0;
            }
            return i;
        }
    }
    return -1;
}

//计算最小二乘拟合直线
float ls[2];
float *GetLSMatchingLine(int line[], int start, int end)//leftLine, leftYStart, y2
{
    int h1, sum1, sum2, sum3, sum4;//最小二乘法计算
    //最小二乘法拟合(横Y纵X)X为准确值，Y为测量值
    sum1 = 0; sum2 = 0; sum3 = 0; sum4 = 0;
    h1 = end - start + 1;
    for (int i = start; i <= end; i++)
    {
        sum1 += line[i];//y求和
        sum2 += PROW - 1 - i;//x求和
        sum3 += (PROW - 1 - i) * line[i];//x*y求和
        sum4 += (PROW - 1 - i) * (PROW - 1 - i);//x^2求和
    }
    ls[0] = (float)(h1 * sum3 - sum1 * sum2) / (h1 * sum4 - sum2 * sum2);//a
    ls[1] = (float)(sum1 / h1 - ls[0] * ((float)sum2 / h1));//b
    return ls;
}

//判断线段类型
float MAX_STRAIGHT_AREA = 7;
int JudgeLineType(int line[], int start, int end)
{
    float a, b, sum, dis, avDis;
    a = (float)(line[start] - line[end] + 1) / (end - start + 1);
    b = line[start] - a * (PROW - 1 - start);
    sum = 0;
    for (int i = start; i <= end; i++)
    {
        dis = (float)(fabs(a * (PROW - 1 - i) - line[i] + b) / sqrt(a * a + 1));
        sum += dis;
    }
    //直线返回1而曲线返回2
    avDis=sum / (end - start + 1);
    if (avDis <= MAX_STRAIGHT_AREA)
    {
        return 1;
    }
    else
    {
        return 2;
    }
}

//反向双边寻线
int ys[2];
int MIN_CON_POINT=5;
int *ReverseSearchLine(uint8 *pSrc, int line[], int start, float a, float b, int direction)
//p2, leftLine, leftYEnd, a, b, 1  
{
    int midTrue, midX, num, lastX = 0, conCnt = 0, lost = 0;
    ys[0]=-1;
    ys[1]=-1;
    //左边寻线
    if (direction == 1)
    {
        conCnt = 0;
        lost = 0;
        //直接边缘
        for (int i = PROW - 1; i > start + 5; i--)
        {
            midTrue = (int)(a * (PROW - 1 - i) + b);
            midX = midTrue;
            if (midX > rightEdge[PROW - 1 - i]) { midX = rightEdge[PROW - 1 - i]; }
            if (midX < leftEdge[PROW - 1 - i]) { midX = leftEdge[PROW - 1 - i]; }
            for (int j = midX; j > 0; j--)//从中间向左边扫描
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                if (num == 1)//有跳变点
                {
                    if (midTrue - j > WIDTH / 2 + 10)
                    {
                        break;
                    }
					//J满足条件，
                    lost = 0;
                    if (ys[1] == -1) { lastX = j; }
                    if (conCnt < MIN_CON_POINT)
                    {
                        line[i] = j;
                        conCnt++;
                    }
                    else
                    {
                        if(conCnt==MIN_CON_POINT){ys[1] = i;}
                        conCnt++;
                        if (j - lastX < MAX_XDIS && j - lastX > -MAX_XDIS) { line[i] = j; ys[0] = i; }
                    }
                    lastX = j;
                    break;
                }
            }
            if (lost != 0 && conCnt < MIN_CON_POINT) { conCnt = 0; }
            lost++;
            if (lost > MAX_LOST) { break; }
        }
    }
	
    else if (direction == 2)
    {
        conCnt = 0;
        lost = 0;
        //直接边缘
        for (int i = PROW - 1; i > start + 5; i--)
        {
            midTrue = (int)(a * (PROW - 1 - i) + b);
            midX = midTrue;
            if (midX > rightEdge[PROW - 1 - i]) { midX = rightEdge[PROW - 1 - i]; }
            if (midX < leftEdge[PROW - 1 - i]) { midX = leftEdge[PROW - 1 - i]; }
            for (int j = midX; j < PCOLUMN; j++)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                if (num == 1)
                {
                    if (j - midTrue > WIDTH / 2 + 10)
                    {
                        break;
                    }
                    lost = 0;
                    if (ys[1] == -1) { lastX = j; }
                    if (conCnt < MIN_CON_POINT)
                    {
                        line[i] = j;
                        conCnt++;
                    }
                    else
                    {
                        if(conCnt==MIN_CON_POINT){ys[1] = i;}
                        conCnt++;
                        if (j - lastX < MAX_XDIS && j - lastX > -MAX_XDIS) { line[i] = j; ys[0] = i; }
                    }
                    lastX = j;
                    break;
                }
            }
            if (lost != 0 && conCnt < MIN_CON_POINT) { conCnt = 0; }
            lost++;
            if (lost > MAX_LOST) { break; }
        }
    }
    return ys;
}

//直入十字确定下直角
int MAX_HORLINE_LENGTH = 15;
int GetStraightDownRAngle(uint8 *src, int line[], int start, int end, int direction)
{
    int pn=0,st=0;
    //左线
    if (direction == 1)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[i]; j > 0; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)
                {
                    pn++;
                }
                else
                {
                    break;
                }
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                //直入十字
                return i - 1;
            }
        }
    }
    //右线
    else if (direction == 2)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[i]; j < PCOLUMN; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)
                {
                    pn++;
                }
                else
                {
                    break;
                }
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                //直入十字
                return i - 1;
            }
        }
    }
    return -1;
}

//直入十字确定上直角
int GetStraightUpRAngle(uint8 *src, int line[], int start, int end, int direction)
{
    int pn=0,st=0;
    //左线
    if (direction == 1)
    {
        if(end-start<=2){st=end-1;}
        else{st=start+2;}
        for (int i = st; i >= start; i--)
        {
            pn = 0;
            for (int j = line[i]; j > 0; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)
                {
                    pn++;
                }
                else
                {
                    break;
                }
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                //直入十字
                for (int j = i; j >= start; j--)
                {
                    line[j] = 0;
                }
                return i + 1;
            }
        }
    }
    //右线
    else if (direction == 2)
    {
        if(end-start<=2){st=end-1;}
        else{st=start+2;}
        for (int i = st; i >= start; i--)
        {
            pn = 0;
            for (int j = line[i]; j < PCOLUMN; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)
                {
                    pn++;
                }
                else
                {
                    break;
                }
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                //直入十字
                for (int j = i; j >= start; j--)
                {
                    line[j] = 0;
                }
                return i + 1;
            }
        }
    }
    return -1;
}

//十字交叉补线
void CrossMakeUpLine(int line[], int start, int end)
{
    int i, j, num, y1 = 0, y2 = 0, t1, t2;
    double slope;
    //对不是0的上下两点按斜率补线
    for (i = start; i <= end; i++)
    {
        num = line[i];
        if (num == 0)
        {
            if (y1 == 0)
            {
                y1 = i - 1;
            }
        }
        if (num != 0 && y1 != 0)
        {
            y2 = i;
            t1 = line[y1];
            t2 = line[y2];
            slope = ((float)t2 - t1) / ((float)y2 - y1);
            for (j = y1 + 1; j < y2; j++)
            {
                line[j] = (int)(t1 + slope * (j - y1) + 0.5);
            }
            y1 = 0;
            y2 = 0;
        }
    }
}

/*********************/
//计算曲率1
  int A,C,BC,AB,T;
  float X;
float CuvreControl(int start,int end)
{

  if (start>end)
  {
  T=start;
  start=end;
  end=T;
  }
    
  A=leadLine[start];
  C=leadLine[end];
  BC=C-A;
  AB=end-start;
  X=(float)2.0*BC/(float)(AB*AB+BC*BC);
  return X;
}




//图像处理算法
/***********************************/
uint8 tr,staticTr;
void imageProcess(uint8 *src)//src校正后的图像
{
        int8 re;
        uint8 i;
        
        //寻双线
        RecordBWChange(src,p2);//记录跳变沿

        re=SearchBaseLines(p2);//寻双线
        CrossRecognize(p1,p2);
        CalculateLeadLine();//计算引导线 
        leadlength=leadYEnd-leadYStart;
        
        Cuvre[0]=CuvreControl(leadYStart+leadlength*2/3,leadYEnd);
        Cuvre[1]=CuvreControl(leadYStart+leadlength/3,leadYStart+leadlength*2/3-1);
        Cuvre[2]=CuvreControl(leadYStart,leadYStart+leadlength/3-1);
       // Cuvre[3]=(Cuvre[0]*7+Cuvre[1]*2+Cuvre[2])/(3.0*10.0); 
        midpoint_before=AAGAFilter(leadLine);//加权平均滤波
        
        if(abs(midpoint_before-midpoint_before_E)>30)
          midpoint_before=midpoint_before_E;
        else
          midpoint_before_E=midpoint_before;
        
        Site_t site;
        site.x = 110;
        site.y = 32;
        LCD_num_BC(site,midpoint_before, 3,BLUE,RED); 
        
        LineType=JudgeLineType(leadLine,leadYStart,leadYEnd);
 
        
}

uint32 leadnum0,leadnumhalf;
int half=0;
void LCD_line_display(Site_t site)
{
	half=(leadYStart+leadYEnd)/2;
  	leadnum0=leadLine[5]%10+(leadLine[5]%100)/10*10+(leadLine[5]%1000)/100*100;
	leadnumhalf=leadLine[half]%10+(leadLine[half]%100)/10*10+(leadLine[half]%1000)/100*100;
    site.x = 10;
    site.y = 16;
   /* LCD_num_BC(site,leftYStart, 2,BLUE,RED);    
	site.x = 10;
    site.y = 32;
    LCD_num_BC(site,leftYEnd, 2,BLUE,RED);
    site.x = 50;
    site.y = 16;
    LCD_num_BC(site,rightYStart, 2,BLUE,RED);
    site.x = 50;
    site.y = 32;
    LCD_num_BC(site,rightYEnd, 2,BLUE,RED);
	site.x = 30;
    site.y = 16;
    LCD_num_BC(site,leadYStart, 2,BLUE,RED);
    site.x = 30;
    site.y = 32;
    LCD_num_BC(site,leadYEnd, 2,BLUE,RED);
	site.x = 80;
    site.y = 16;
    LCD_num_BC(site,leadnum0, 3,BLUE,RED);
	site.x = 80;
    site.y = 32;
    LCD_num_BC(site,leadnumhalf, 3,BLUE,RED);*/
    for(site.y=60+leadYEnd;site.y>=60+leadYStart;(site.y)--)  
    {
        site.x=*(leadLine+PROW-site.y+60);
        site.x=site.x*0.8;
        LCD_point(site, GREEN);
    }
    for(site.y=leftYEnd+60;site.y>=leftYStart+60;(site.y)--)   
    {
        site.x=*(leftLine+PROW-site.y+60);
        site.x=site.x*0.8;
        LCD_point(site, RED);
    }
    for(site.y=rightYEnd+60;site.y>=rightYStart+60;(site.y)--)
    {
        site.x=*(rightLine+PROW-site.y+60);
        site.x=site.x*0.8;
        LCD_point(site, BLUE);
    }
    
}   
//深搜 从x,y位置开始递归往下搜 找到所有连通区域
//搜索顺序是正上、左上、右上、左、右、左下、右下、正下 
//左右连通判断 和 上下连通判断是不同的方法
void DFS(int x,int y)//完成
{
    if (x<0 || x>=Img_H || y<0 || y>=Img_W)return;//避免出界
    if (ConGraph[x][y] != 0 ) return;//已经搜索过，跳出递归
    ConGraph[x][y] = 1;//标记连通
    IsConnect[x]=1;
    if (x > 0)//往上方搜 
    {
        if (ColConnect(img[x][y], img[x - 1][y]))
            DFS(x - 1, y);
    }
    if (y > 0 )//左右搜
    {
        if (RowConnect(img[x][y-1], img[x][y]))
            DFS(x, y - 1);
    }
    if (y < Img_W-1)
    {
        if (RowConnect(img[x][y] , img[x][y + 1]))
            DFS(x, y + 1);
    }
    if (x < Img_H-1)//往下搜
    {
        if (ColConnect(img[x][y], img[x + 1][y]))
            DFS(x + 1, y);
    }
}

//找右跳变点 从白变位黑 0000 1111 返回跳变点位置
int RightJump(unsigned char a)
{
    int  i;
    if (a >= 0x0f )//左边存在黑点 在左边找跳变
    {
        a >>= 4;
        for (i = 0; i < 4; i++)
        {
            if ((a&(1 << i)) == 0)break;
        }
        i += 4;
    }
    else {//右边找跳变
        for (i = 0; i < 4; i++)
        {
            if ((a&(1 << i)) == 0)break;
        }
    }
    return 8 - i;
}

//找左跳变点  从黑变成白 1111 0000
int LeftJump(unsigned char a)
{
    int i = 0;
    if ((a & 0x0f) != 0x0f)//右边四个不都是1
    {
        for (i = 0; i < 4; i++)
        {
            if ((a >> i) & 0x01 == 1)break;
        }
        i = 8 - i;
    }
    else//在左边四个找到跳变点
    {
        a >>= 4;
        for (i = 0; i < 4; i++)
        {
          if ((a >> i) & 0x01 == 1)break;
        }
        i = 4 - i;
    }
    return i;
}

//地毯式搜索 用于前三行 
//找到这一行中最大的一段白线的中点
int CarpetSearch(int row)
{
    int ans;
    int len = 0, Maxlen = 0;
    int start,end,mid=-1;
    int i,j;
    for (i = 0; i < Img_W; i++)
    {
      if (img[row][i] != 0xff)//出现白点
      {
        start = LeftJump(img[row][i])+i*8;//标记起点
        for (j = i+1; j < Img_W; j++)
        {
          if (img[row][j] != 0x00)//出现黑点
              break;
        }
        end = RightJump(img[row][j]) + j * 8;
        ans = (int)((start + end + 0.5) / 2 );
        i = j;
        len = end - start;
        if (len > Maxlen) {
            mid = ans;
            Maxlen = len;
        }
      }
    }
    return mid;
}