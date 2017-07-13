#include "include.h"
#include "image.h"
#include "MK60_gpio.h"

#include "arm_math.h"

uint8   stopflag=0;
uint8   startgogo=0;
uint8   zhili=0;
uint8   wending=0;
uint8   biansu=0;
uint8   biancan=0;
uint8  zhuantou=0;

//采集用的
uint8 Buffer[120];
//双数组采集用的
uint8 processReady=0;
uint8 wantProcess=0;
//采集用的
uint8 ImageBuffer1[SROW][SCOLUMN];
uint8 ImageBuffer2[SROW][SCOLUMN];
//连通图
unsigned char ConGraph[60][20] = { 0 };//连通图
uint8 IsConnect[60]={0};
uint8 ConNum[60]={0};
//求连通图
extern uint8 img[60][20];
#define ColConnect(a,b) (a|b)!=255				//列连通 1连通
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//行连通 0为连通
uint8 *pSample=ImageBuffer1[0];
uint8 *pProcess=ImageBuffer2[0];
uint8 *pTemp=ImageBuffer1[0];
//环形
int Linemid_1[PROW]={0};
int xyz=-1;
int huan_sign=0;
//曲线
int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd;//左线,右线和引导线的Y方向起始及终止点
int LineType;  //曲线类型
int leftLine[PROW];
int rightLine[PROW];
//障碍物最低行
int block_hang=0;
int leadLine[PROW];
int ShowArray[PROW];//上位机显示数组
int Ping[5];  //平均滤波 后期可删
int WIDTH=36;//记录赛道宽度
int leadlength;
int midpoint_before = HEART,midpoint_before_E = HEART  ;
int Ruhuan_sign=0;
int offset_1;//此处的offset_1  不可更改变量名为offset，野火库的关键字啊，不然怎么死的都不知道
int offset_2;
int dOffset;
int oldOffset;
int newOffset;
float Cuvre[4];

float CircleRate=0;

uint8 STOP=0;

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
int     AAGAFilter(int *src)
{ 
 
   int i=0,Value=0;
   int sum=0,CoeSum=465;
   int Coefficient[30]; 

   for(i=1;i<31;i++) //  求和
   {
      Coefficient[i-1]=i;
   }
   
   for(i=0;i<30;i++)
  {
    if(*(src+i)<1){
     CoeSum+=-30+i;
    }
    else
    sum+=*(src+i) * Coefficient[29-i];
   }
    Value=sum/CoeSum;
    return(Value);  
}

/**********************************
//搜索边界
int leftEdge1[PROW]={0};
int rightEdge1[PROW]={0};

void Search_the_boundary(void)
{
	int i,j;
	for(i=59;i>=0;i--)//左边界
	{
		for(j=0;j<160;j++)
		{
			if(mapX[i][j]!=0)
			{
				leftEdge1[59-i]=j;
				break;
			}
		}
	}
	for(i=59;i>=0;i--)//右边界
	{
		for(j=159;j>=0;j--)
		{
			if(mapX[i][j]!=0)
			{
				rightEdge1[59-i]=j;
				break;
			}
		}
	}
        ;
}*/

/**************************************************************************************/
/*****************************************寻找双线***********************************/
//记录跳变沿
void RecordBWChange(uint8 *src,uint8 *pSrc)//P1 P2
{
        int num1, num2;
	int two_white_sign=0,two_black_sign=0,usefull_hang=0;
        block_hang=0;
   ArraySetValue(pSrc, 255); //重组数组，P2全给0 
    for (int i = 0; i < PROW; i++)//每一行
    {   two_white_sign=0;
        two_black_sign=0;
        
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
	    else if(num1==255&& num2 == 255)
		  	{
                //if(!two_black_sign)
			       two_white_sign++; 
				if(two_black_sign>6&&two_black_sign<15)
					{
					//白 黑 白
                                          
					two_black_sign=0;//  置位使得每一行只进来一次
                                        if(usefull_hang==0)
                                          block_hang=i;
                                        usefull_hang++;					                
					
				      }
            }
            else if(num1==0&& num2 == 0)
            {
             if(two_white_sign>1)
			   two_black_sign++;
		  	 } 

	   }
 
    }
   if(usefull_hang<10||usefull_hang>26)
     block_hang=0; 
             
	
}
///**************中位数滤波*********/
#define N 5
int zwzfilter(int leadlength)
{
         int sum=0;
         int buf[N],i,j,temp=0;
          for(int m=0;m<leadlength/5;m++)
          {
       
       for(i=0;i<N;i++)
       {
        buf[i]=leadLine[5*m+i];
       }
       for(i=0;i<N;i++)
       {
        Ping[i]=buf[i];
       }
       for(j=0;j<N-1;j++)
         for(i=0;i<N-j-1;i++)
         {
            if(buf[i]>buf[i+1])
            {
              temp= buf[i];
              buf[i]=buf[i+1];
              buf[i+1]=temp;
            }
         }
       
  
       sum+=buf[(N-1)/2];
          }
      return sum/(leadlength/5);
       
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
    int i,j,m,n,lie;
    //清空之前的寻线数据
    leftYStart = -1; rightYStart = -1; leftYEnd = -1; rightYEnd = -1; leadYStart = -1; leadYEnd = -1;
    zhuantou=0;
    for (i = 0; i < PROW; i++)
    {
        leftLine[i] = 0;
        rightLine[i] = 0;
        leadLine[i] = 0;
    }
	/*************
    int a=0;
    WIDTH[0]=0;
    for(i=0;i<PROW;i++)//搜索所有行，读取赛道宽度；
    {
        if(WIDTH[59]!=0)break;//已读取赛道宽度则退出       
        leftCnt = 0;//找到的点数
        for (j = HEART; j > 0; j--)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                leftHalf0[leftCnt++] = j;//第几个点的列数
            }
        }
        //向右搜索
        rightCnt = 0;
        for (j = HEART; j < 159; j++)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                rightHalf0[rightCnt++] = j;
            }
        }
        WIDTH[i]=rightHalf0[0]-leftHalf0[0];
        a=a+WIDTH[i];
        
    }
    a=a/60;
    num=0;*/
    //////////////************/
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

		    /*leftYStart = i;
		    leftYEnd = i;
		    leftLine[i] = leftHalf[0];
		    rightYStart = i;
		    rightYEnd = i;
		    rightLine[i] = rightHalf[0];*/
		     for (m = 0; m < leftCnt; m++)
            {
                for (n = 0; n < rightCnt; n++)
                {
                    t1 = rightHalf[n] - leftHalf[m];//左右点间距
                    //1cm  为1.3个点
                    //感觉此方法得进行校正后使用 
                    /*if(t1<WIDTH - 8)
                    {
                    	lie++;
						if(lie>=2)STOP=1;
                    }*/
                    
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
                if(processBuf1[line[j]][j]==0) /////////                    
                  line[j] =0;                 /////////2017.7.9加，判断是不是黑线，黑线不布线
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
    int y1, y2, y3, y4, yl, yr,leftLine_before=60,rightLine_before=100;
    float divide; 
    int sign=0;
    //吧左右线为零的地方补线
     for (int i = leftYStart; i <= leftYEnd; i++)
        {
              if(leftLine[i]==0)
              leftLine[i]=leftLine_before;
            
              else
              leftLine_before=leftLine[i];
        }                     
            for (int i = rightYStart; i <= rightYEnd; i++)
        {
        
               if(rightLine[i]==0)
              rightLine[i]=rightLine_before;
           
                else
              rightLine_before=rightLine[i];  
      
    
        }  
        //只有左线(完全没右线)      
            
    if (leftYStart != -1 && rightYStart == -1)
    {   
        leadYStart = leftYStart;
        leadYEnd = leftYEnd;
        

        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            
          //检测左线右边是不是黑线，是黑线就1  白线 -1
          for(int j=2;j<7;j++)
          {
           if(processBuf1[PROW-1-i][leftLine[i]-j]==0)
                sign++;
           else 
                sign--;
          
          }
          
          leadLine[i] = leftLine[i] +sign/abs(sign)*WIDTH / 2;
          sign=0;  
        }
        
        
    }
    //只有右线(完全没左线)
    else if (leftYStart == -1 && rightYStart != -1)
    {
        leadYStart = rightYStart;
        leadYEnd = rightYEnd;
        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            
          ////////////////////////////
          for(int j=2;j<7;j++)
          {
           if(processBuf1[PROW-1-i][rightLine[i]+j]==255)
                sign++;
           else 
                sign--;
          
          }
          
          leadLine[i] = rightLine[i] + sign/abs(sign)*WIDTH / 2;
          sign=0;
          
          ///////////////////////
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
              for(int j=2;j<7;j++)
           {
           if(processBuf1[PROW-1-i][leftLine[i]-j]==0||processBuf1[PROW-1-i][leftLine[i]+j]==255)
                sign++;
           else 
                sign--;
          
           }
          
          leadLine[i] = leftLine[i] +sign/abs(sign)*WIDTH / 2;
          sign=0; 
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
                for(int j=2;j<7;j++)
          {
           if(processBuf1[PROW-1-i][rightLine[i]+j]==255||processBuf1[PROW-1-i][rightLine[i]-j]==0)
                sign++;
           else 
                sign--;
          
          }
          
          leadLine[i] = rightLine[i] + sign/abs(sign)*WIDTH / 2;
          sign=0; 
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
                        ////////////////////////////
               
          for(int j=2;j<7;j++)
           {
           if(processBuf1[PROW-1-i][leftLine[i]-j]==0||processBuf1[PROW-1-i][leftLine[i]+j]==255)
                sign++;
           else 
                sign--;
            
           
           
           }
          
          leadLine[i] = leftLine[i] +sign/abs(sign)*WIDTH / 2;
          sign=0; 
          /////////////////////// 
              }
                else
                {
              ////////////////////
                   for(int j=2;j<7;j++)
                {
            if(processBuf1[PROW-1-i][rightLine[i]+j]==255||processBuf1[PROW-1-i][rightLine[i]-j]==0)
                sign++;
            else 
                sign--;
          
                 }
          
          leadLine[i] = rightLine[i] + sign/abs(sign)*WIDTH / 2;
          sign=0; 
          ////////////////////////
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
                    ////////////////////////////
               for(int j=2;j<7;j++)
            {
           
             if(processBuf1[PROW-1-i][rightLine[i]+j]==255&&processBuf1[PROW-1-i][leftLine[i]-j]==255&&processBuf1[PROW-1-i][rightLine[i]-j]==0&&processBuf1[PROW-1-i][leftLine[i]+j]==0)            
             sign++;
            else 
             sign--;          
             } 
              if(sign>0)
              {
                leadLine[(yl + yr) / 2] = leftLine[yl] - WIDTH / 2;
              
             
              }
               else
                leadLine[(yl + yr) / 2] = (leftLine[yl] + rightLine[yr]) / 2;
               
               sign=0;
                }
           ////////////////////////////////////////////////     
            }
            CompleteLine(leadLine);
        }
    }
}
//深搜 从x,y位置开始递归往下搜 找到所有连通区域
//搜索顺序是正上、左上、右上、左、右、左下、右下、正下 
//左右连通判断 和 上下连通判断是不同的方法
void DFS(int x,int y)//完成
{
    if (x<0 || x>=60 || y<0 || y>=20)return;//避免出界
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
    if (y < 20-1)
    {
        if (RowConnect(img[x][y] , img[x][y + 1]))
            DFS(x, y + 1);
    }
    if (x < 60-1)//往下搜
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
    for (i = 0; i < 20; i++)
    {
      if (img[row][i] != 0xff)//出现白点
      {
        start = LeftJump(img[row][i])+i*8;//标记起点
        for (j = i+1; j < 20; j++)
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
float MAX_STRAIGHT_AREA = 1.01;
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
//环形弯的处理
void HuangXing(void)
{int i,j,num=0,sum=0;
  for(i=59;i>20;i--)
  {
     for (j = rightEdge[i]; j >leftEdge[i] ; j--)
     {
     
       if(processBuf1[59][j]==255||processBuf1[58][j]==255||processBuf1[57][j]==255||processBuf1[55][j]==255)
       {   
       if(i<30)
       {
       if(processBuf1[i][j]==255)
        {sum++;
         continue;
         }
       }
       else
       {
        if(processBuf1[i][j]==0)
        { sum++;
         continue;
         }
       }
       }
        
     } 
     if (sum==0)
       {num++;
        sum=0;
        } 
 
  }
      Site_t site;
        site.x = 110;
        site.y = 10;
        LCD_num_BC(site,num, 3,BLUE,RED); 

}

/************************************************************/
int GetStraightDownRAngle_2(uint8 *src, int line[], int start, int end, int direction)
	//P2 leftLine, leftYStart, leftYEnd, 1
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
            for (int j = line[end]; j < HEART; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//无跳变
                {
					pn++;
                }
                else if(pn>2)
                {
			       	return i - 1;
					
                }
                
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                
                return  -1;
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
            for (int j = line[end]; j > HEART; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//无跳变
                {
					pn++;
                }
                else if(pn>2)
                {
			       	return i - 1;
					
                }
                
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                
                return  -1;
            }
        }
    }
    return -1;
}

/************************************************************/
int GetStraightDownRAngle_1(uint8 *src, int line[], int start, int end, int direction)
	//P2 leftLine, leftYStart, leftYEnd, 1
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
            for (int j = line[end]; j < HEART; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//无跳变
                {
					pn++;
                }
                else if(pn<=2)
                {
			       	return i - 1;
					
                }
                
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                
                return  -1;
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
            for (int j = line[end]; j > HEART; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//无跳变
                {
					pn++;
                }
                else if(pn<=2)
                {
			       	return i - 1;
					
                }
                
            }
            if (pn > MAX_HORLINE_LENGTH)
            {
                
                return  -1;
            }
        }
    }
    return -1;
}

//*************判断十字或环形**************/
//    0为直线入环
//    1为直线入十字
//    2为斜入环
//    3为斜入十字
//    -1为均不是
int Crossorring(uint8 *src,uint8 *pSrc)
{
	
	int i,j,k,num,sum=0;
	int midY=-1;
	float a,b,c,d;
	int  ly1 = -1, ry1 = -1;
	int Linemid;
	int  ly2 = -1, ry2 = -1;
	int endmix,endmax,startmix,startmax;
	//直入
	if (leftYStart != -1&&rightYStart != -1)
        {
        //ly2 = GetStraightDownRAngle(src, leftLine, leftYStart, leftYEnd, 1);
		//ry2 = GetStraightDownRAngle(src, rightLine, rightYStart, rightYEnd, 2);
		if(leftYEnd>=rightYEnd)
    	{
    		  endmax=leftYEnd;
		  endmix=rightYEnd;
    	}
		else
		{
		endmax=rightYEnd;
		endmix=leftYEnd;
		}
		if(leftYStart>=rightYStart)
    	{ 
    		startmax=leftYStart;
		startmix=rightYStart;
    	}
		else
		{
			startmax=rightYStart;
			startmix=leftYStart;
		}
		
		if(endmix-startmax<=3)return -1;
                if(endmix>=30)return -1;
		if((endmax-endmix)>18)return -1;
		ly2 = GetStraightDownRAngle_2(src, leftLine, leftYStart, leftYEnd, 1);
		ry2 = GetStraightDownRAngle_2(src, rightLine, rightYStart, rightYEnd, 2);
                ly1 = GetStraightDownRAngle_1(src, leftLine, leftYStart, leftYEnd, 1);
		ry1 = GetStraightDownRAngle_1(src, rightLine, rightYStart, rightYEnd, 2);
		
		if ((ly2 != -1 && ry2 != -1)||(ly1 != -1 && ry1 != -1))//存在下直角
		{
			for(i=startmax;i<=endmix-3;i++)
			{
				Linemid_1[i]=(int)(leftLine[i]+rightLine[i])/2;
			}
			//float *kv = GetLSMatchingLine(Linemid_1, startmax,endmix-4);
                        //c=kv[0];d=kv[1];
            if((endmix-3-startmax)<=2)return -1;
            c=1.0*(Linemid_1[endmix-3]-Linemid_1[startmax])/((endmix-3)-startmax);
			d=Linemid_1[endmix-3];
			sum=0;
			for(k=20;k<=25;k++)
			{
				Linemid=(int)(c*k+d);
				for(i=Linemid-3;i<Linemid+3;i++)
				{
					num = *(src+(PROW - 1 - endmix-k)*PCOLUMN+i);
					if(num == 0)//有跳变
					{sum++;}
					if(sum>=10)return 0;//直入圆环
				}
			}
			if((endmax!=PROW - 1)&&(sum<3))
			{
				return 1;//直入直角
			}
		}	
	}
	/*else if(leftYStart != -1&&rightYStart == -1)//只有左线
	{		
		midY=GetBiasDownRAngleX(leftLine, leftYStart, leftYEnd);
		if(midY!=-1)
		{
			
			float *abf = GetLSMatchingLine(leftLine, leftYStart, midY);
            a=abf[0];
			float *abb = GetLSMatchingLine(leftLine, midY+1, leftYEnd);
            b=abb[0];
			Angle=a*b;
			if(Angle<-3)return 2;//斜入环
			else if(-3<Angle<0) return 3;//斜入十字
			else return -1;
                       
		}
		else return -1;
	}*/
	/*else if(leftYStart == -1&&rightYStart != -1)//只有右线
	{
		midY=GetBiasDownRAngle(rightLine, rightYStart, rightYEnd);
		if(midY!=-1)
		{
			
			float *abf = GetLSMatchingLine(rightLine, rightYStart, midY);
            a=abf[0];
			float *abb = GetLSMatchingLine(rightLine, midY+1, rightYEnd);
            b=abb[0];
			Angle=a*b;
			if(Angle<-3)return 2;//斜入环
			else if(-3<Angle<0) return 3;//斜入十字
			else return -1;                       
		}
		else return -1;
	}*/
	else return -1;
}




//图像处理算法
/***********************************/
uint8 tr,staticTr;
void imageProcess(uint8 *src)//src校正后的图像
{
        int8 re;
        uint8 i;
        
        int test[5];       

        //寻双线
        RecordBWChange (src,p2);//记录跳变沿

        
        re=SearchBaseLines(p2);//寻双线
        
        //if(re==-1) ArraySetValue(p2,255);
         // re=RemoveNoise(1);//移除噪音
        //if(re==-1) ArraySetValue(p2,255);
        //十字交叉
        //引导线
        //re=RemoveNoise(2);//移除噪音
        //if(re==-1) ArraySetValue(p2,255);
        CrossRecognize(p1,p2);
        
       
        //xyz=Crossorring(p1,p2);
        CalculateLeadLine();//计算引导线 
        ///////处理环形弯
        huan_sign=judgebigring(20);
        if(huan_sign<20&&huan_sign>15)
        {  
            DELAY_MS ( 10 ) ;          
            for(int i=0;i<20;i++)
              leadLine[i]=leftEdge[i]+WIDTH / 3;    
        }
        
       ////////处理障碍
        block_hang=block_avoid();
        if(block_hang>10)
        {
           // FM (1) ;
            DELAY_MS ( 40 ) ;
           // FM (0) ;
        
    
        }
      
        leadlength=leadYEnd-leadYStart;
        Cuvre[0]=CuvreControl(leadYStart+leadlength*2/3,leadYEnd);
        Cuvre[1]=CuvreControl(leadYStart+leadlength/3,leadYStart+leadlength*2/3-1);
        Cuvre[2]=CuvreControl(leadYStart,leadYEnd);
        midpoint_before=AAGAFilter(leadLine);//加权平均滤波
        // midpoint_before=zwzfilter(leadlength);
       
        if(abs(midpoint_before-midpoint_before_E)>30)
        midpoint_before=midpoint_before_E;
        else
        {
          midpoint_before=midpoint_before*0.5+midpoint_before_E*0.5;
          midpoint_before_E=midpoint_before;
        }
       
        
        Site_t site;
        site.x = 110;
        site.y = 32;
        LCD_num_BC(site,midpoint_before, 3,BLUE,RED); 
        
        LineType=JudgeLineType(leadLine,leadYStart,leadYEnd);
          
        
}

int block_avoid(void)
{int i,j;
 int white_sign=0,two_black_sign=0,usefull_hang=0,wide_hang=0;
    if(leadlength<45||(leftYEnd-leftYStart<40)||(rightYEnd-rightYStart)<40)
		return 0;
    
   for(i=block_hang+2;i<block_hang+5;i++)
     {    wide_hang=leftLine[i]-leftLine[i]-3;
	 for(j=leftLine[block_hang]+1;j<rightLine[block_hang];j++)
	 	{
                     if(processBuf1[i][j]==255)
		  	{
                         white_sign++;                          
                        }
                }
          if(white_sign>=wide_hang)
          {
           usefull_hang++;
           if(usefull_hang>1)
	   	return block_hang;
           white_sign=0;
          }
	   	 
    }
   return 0;

}




void Crossorring_huanxing(void)
{ 
  int i,j;
 /*// int leadLine_T[PROW];
  for(i=0;i<40;i++)
  {
    for(j=leftEdge[i]-2;j<80;j++)
       {
            if(processBuf1[i][j]==0&&processBuf1[i][j+1]==255)
            {leadLine[i] =j+ WIDTH / 2;
            break;
              }
      
        } 
  */
   // leftYStart = -1; rightYStart = -1; leftYEnd = -1; rightYEnd = -1; leadYStart = -1; leadYEnd = -1;
    if(((rightYEnd-leftYStart)+(leadYEnd-rightYStart)+(leftYEnd-leadYStart))<6&&leadYEnd<16&&rightYEnd<16&&leftYEnd<16&&leadYEnd>5&&rightYEnd>5&&leftYEnd>5)     
    { 
      if((leftYStart!=-1)&&(rightYStart!=-1)&&(leftYEnd !=-1)&&(rightYEnd!= -1)&&(leadYStart!= -1)&&(leadYEnd!= -1))
      {midpoint_before=60;
       DELAY_MS ( 60 ) ;
           FM (1) ;
           DELAY_MS ( 100 ) ;
           FM (0) ;
      }
    } 
    
    
    /*
      for(i=0;i<PROW;i++)
    {if(processBuf1[i][80]&&processBuf1[i][79])
      
      
    }
  */
       
 

}
///////////判断大圆环，有全黑的行数
////////
////  black_hang_up  :从下面找到的第一行全黑上面第black_hang向下找五行，有四行全黑则判断为环形
int judgebigring(int black_hang_up)
{
	int plot_black_L=0,plot_white_L=0,all_black=0,all_white=0,parts_black=0,wide_hang=0,black_hang=0,i;
        int plot_black_R=0,plot_white_R=0,all_black_hang=0;   
        int plot_sign;
        //加判断条件，节约时间  最顶行为全黑，最低行为全白
        for(uint8 y=leftEdge[1]+1;y<rightEdge[1]-1; y++)
        {
          
        if (processBuf1[1][y]==255)//0是黑，255白 
          {
            plot_sign++;
            if(plot_sign>4)
             return 0;                       
          } 
        }
        for(uint8 y=leftEdge[58]+1;y<rightEdge[58]-1; y++)
        {
          
        if (processBuf1[58][y]==0)//0是黑，255白 
          {
               plot_sign++;
            if(plot_sign>4)
             return 0;                      
          } 
        }
       //////////////////////////////////////////
        
	for (i = 0; i < PROW-10; i++)//每一行
    {
         wide_hang=rightEdge[PROW-1-i]-leftEdge[PROW-1-i]-3;
        for (uint8 l = HEART,r=HEART;l>leftEdge[PROW-1-i]+1,r<rightEdge[PROW-1-i]-1; l--,r++)
        {      
          if (processBuf1[PROW-1-i][HEART]==0)//0是黑，255白 
          {
            if (processBuf1[PROW-1-i][l]==0)
           {
                 plot_black_L++;
                                 
           }
           if(processBuf1[PROW-1-i][r]==0)
           {
              plot_black_R++;
           }
           if (processBuf1[PROW-1-i][l]==255)
           {
                 plot_white_L++;
             
           }
           if(processBuf1[PROW-1-i][r]==255)
           {     plot_white_R++;
           
           }
          }
           else
             break;
          if(((plot_black_L+plot_black_R)>=wide_hang/4)&&(plot_white_L>=2)&&(plot_white_R>=2))   ////中间出现黑色，两边是白色
           {
           parts_black++;
            }
           if((plot_black_L+plot_black_R)>=wide_hang)
           {
            all_black++;
           if(parts_black>=3)
           {black_hang=i;    //出现全黑的行数
            break;
            }
           
           }
        }
         plot_white_L=0;
         plot_black_R=0;
         plot_black_L=0;
         plot_white_R=0;
           if(black_hang==i&&i!=0)
             break;                   
        }
        ///////////////////////////////上面是找中间黑，两边白 ，下面是满足上面条件后，检验上面 black_hang行是不是存在几行全黑
        if(black_hang==i&&i!=0)
     {
         int black_all=0,plot_black=0,all_black_hang=0; 
        for(int j=black_hang+black_hang_up;j>black_hang+black_hang_up-5;j--)
        {  
          wide_hang=rightEdge[PROW-1-j]-leftEdge[PROW-1-j]-3;
          for (uint8 l = HEART,r=HEART;l>leftEdge[PROW-1-j]+1,r<rightEdge[PROW-1-j]-1; l--,r++)
        {      
          if (processBuf1[PROW-1-j][HEART]==0)
          {
           if (processBuf1[PROW-1-j][l]==0)               
           {
                 black_all++;                                 
           }
           if(processBuf1[PROW-1-j][r]==0)//0是黑，255白 )
           {
             black_all++;
           }
           
           
          }
           else
             break;
          if(black_all>=wide_hang)
           {
            all_black_hang++;
            
           }
        }        
        black_all=0;
        
        }
        if(all_black_hang>=3)
               return  black_hang;
           else
               return  0;
        
     }
     return  0;
}

uint32 leadnum0,leadnumhalf;
int half=0;
void LCD_line_display(Site_t site)
{
	half=(leadYStart+leadYEnd)/2;
  	leadnum0=leadLine[5]%10+(leadLine[5]%100)/10*10+(leadLine[5]%1000)/100*100;
	leadnumhalf=leadLine[half]%10+(leadLine[half]%100)/10*10+(leadLine[half]%1000)/100*100;

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