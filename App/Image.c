#include "Image.h"
#include "common.h"
#include "include.h"

#define ColConnect(a,b) (a|b)!=255				//列连通 1连通
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//行连通 0为连通
#define ImgMap(x,y) (imgbuff[(x)*10+(y)/8]>>(7-(y)%8))&0x01

unsigned char ConGraph[Img_H][Img_W] = { 0 };//连通图
uint8 IsConnect[Img_H]={0};
uint8 ConNum[Img_H]={0};

extern uint8 img[Img_H][Img_W];
extern uint8 imgbuff[CAMERA_SIZE];

 Point  MidLine[Img_H] = { 0 };
 
int Hamming_weight(unsigned char n)		//统计 1 bits 的个数
{
	int count;
	for (count = 0; n; count++)
		n &= n - 1;
	return count;
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
    int start, end,mid=0;
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

int LeftSearch(unsigned char *a, int start) //向左搜索第一个跳变位置 
{
      int i=start;
      for(;i>=0;--i)
      {
        if(a[i]!=0x00)
          return (i*8+Hamming_weight(a[i]));
      }
      return 0;
}

int RightSearch(unsigned char *a, int start)
{
	int i=start;
	for(;i<Img_W;++i)
	{
          if(a[i]!=0x00)
            return (i*8+8-Hamming_weight(a[i]));
	}
	return 80;
}
//搜中线
int Search(unsigned char *MidLine)
{
    unsigned char mid=4,last=4;
    int i,j;
    int len=0;
    MidLine[0] =(LeftSearch(img[Img_H-1],mid)+RightSearch(img[Img_H-1],mid))/2;
    for(i=Img_H-2;i>0;--i)
    {
      last=MidLine[Img_H-i-2]/8;
      
      if(img[i][last]!=0xff)//中点不为黑色 
      {
              MidLine[Img_H-i-1]=(LeftSearch(img[i],last)+RightSearch(img[i],last))/2;
              //从上一行中线位置往两侧找 
      }
      else//中间出现黑色 
      {
        break;         
      }
            
    }
    len=Img_H-i;
    return len;
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

/***************************************
    根据斜率寻找中点位置
**************************************/
int SlopeSearch(int *t,Point *Mid)
{
    int x0, y0,x1,y1;//搜索的起点
    float k, k1;
    Point left, right;
    int i,fx;
    x0 = 2 * Mid[*t - 1].x - Mid[*t - 2].x;
    y0 = 2 * Mid[*t - 1].y - Mid[*t - 2].y;
    x1 = Mid[*t - 1].x;
    y1 = Mid[*t - 1].y;
    if(y0<0 || y0>Img_H || x0<0 || x0>Img_W*8)
    {
        Mid[*t].x = CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
        (*t)++;
        Mid[*t].x = CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
        return 0;
    }
          
    if (ImgMap(Img_H - y0, x0) == 1)//如果搜索起点为黑点
    {
        Mid[*t].x=CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
        (*t)++;
        Mid[*t].x = CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
    }
    else 
    {
        if (x0 <x1 + 10 && x0>x1 - 10)//竖直向上，斜率为无穷 
        {//水平搜索
            for (i = x0 - 1; i >= 0; i--)//上边的交点
                if (ImgMap(Img_H - y0, i) == 1)break;
            left.x = i;
            left.y = y0;

            for (i = x0 + 1; i < Img_W * 8; i++)//下边的交点
                if (ImgMap(Img_H - y0, i) == 1)break;
            right.x = i;
            right.y = y0;
        }
        else if (y0 <y1 + 3 && y0>y1 - 3)//水平线 斜率为0
        {//竖直搜索
            for (i = y0 - 1; i >= 0; i--)//左边的交点
                if (ImgMap(Img_H - i, x0) == 1)break;
            left.x = x0;
            left.y = i;

            for (i = y0 + 1; i < Img_H; i++)//右边的交点
                if (ImgMap(Img_H - i, x0) == 1)break;
            right.x = x0;
            right.y = i;

        }
        else {
            k = (y1 - y0) / (x1 - x0);
            k1 = -1 / k;
            //y-y0=k1(x-x0) => y=k1(x-x0)+y0;
            for (i = x0 - 1; i >= 0; i--)//左边的交点
            {
                fx = (int)(k1*(i - x0) + 0.5 + y0);
                if (fx < 0 || fx >= Img_H)break;
                if (ImgMap(Img_H - fx, i) == 1)break;
            }
            left.x = i;
            left.y = fx;

            for (i = x0 + 1; i < Img_W * 8; i++)//右边的交点
            {
                fx = (int)(k1*(i - x0) + 0.5 + y0);
                if (fx < 0 || fx >= Img_H)break;
                if (ImgMap(Img_H - fx, i) == 1)break;
            }
            right.x = i;
            right.y = fx;
        }
        Mid[*t].x = (left.x + right.x) / 2 + 0.5;
        Mid[*t].y = (left.y + right.y) / 2 + 0.5;
    }
}

//获取中线
int GetMidLine(Point *MidPoint)
{
    int i,len;
   // uint8 x0, y0,x1,y1;//搜索的起点
     //遍历最下方三行
    for ( i = 1; i <= 3; i++)
    {
        MidPoint[i].x = CarpetSearch(Img_H - i);
        MidPoint[i].y = i;
    }
    for(;i<Img_H;i++)
    {
      SlopeSearch(&i, MidPoint);
      if (MidPoint[i].x < 0 || MidPoint[i].y < 0)break;  
      if(IsConnect[Img_H-1-i]==0)break;
    }
    len = i;
    for (i = 10; i < len-2; i++)//简单的滤波 应该没什么卵用
    {
        if (abs(MidPoint[i].x - MidPoint[i + 1].x) > 5)
        {
            MidPoint[i + 1].x = (MidPoint[i].x + MidPoint[i + 2].x) / 2;
            MidPoint[i + 1].y = (MidPoint[i].y + MidPoint[i + 2].y) / 2;
        }
    }
    return len;
}

/****************************************
  统计连每行有多少连通块
  只统计个数，不判断中间是否出现非连通区
****************************************/
uint8 CountRow(int row)
{
  int num=0;
  int i;
  for(i=0;i<Img_W;i++)
    if(ConGraph[row][i]==1)num++;
  
  return num;
}

/****************************************
统计连通图中的信息
每行有多少连通块
****************************************/
uint8 CountGraph(uint8* num)
{
  uint8 i,len;
  
  for(i=Img_H-1;i>0;i--)
  {
    num[i]=CountRow(i);
    if(num[i]>0)len++;
  }
  return len;
}

int ImageProcess()//图像处理程序
{
  int i,j;
  int len=0,x0=0;
  
  for(i=0;i<Img_H;i++)
    for(j=0;j<Img_W;j++)
    img[i][j]=imgbuff[i*10+j];
  //Correction();           //校正图像存到img中 
  
  memset(ConGraph, 0, 600);
  memset(IsConnect,0,60);
  memset(ConNum,0,sizeof(ConNum));
  
  //确定连通图
  for(i=1;i<10;i++)
  {
    x0=CarpetSearch(Img_H - i);
    if(x0>10 && x0<70)break;
  }
  
  DFS(59,(x0+7)/8);

  len=CountGraph(ConNum);//统计每一行连通块个数以及行数
  for(i=0;i<60;i++)//非连通区域填黑
  {
    for(j=0;j<10;j++)
    {
      if(ConGraph[i][j]==0)
        img[i][j]=0xff;  

      imgbuff[i*10+j]=img[i][j];//把图更新到显示数组中
    }
  }
  len=GetMidLine(MidLine);
  
  return len;
}
