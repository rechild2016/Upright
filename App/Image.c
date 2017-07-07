#include "Image.h"
#include "common.h"
#include "include.h"

#define ColConnect(a,b) (a|b)!=255				//����ͨ 1��ͨ
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//����ͨ 0Ϊ��ͨ
#define ImgMap(x,y) (imgbuff[(x)*10+(y)/8]>>(7-(y)%8))&0x01

unsigned char ConGraph[Img_H][Img_W] = { 0 };//��ͨͼ
uint8 IsConnect[Img_H]={0};
uint8 ConNum[Img_H]={0};

extern uint8 img[Img_H][Img_W];
extern uint8 imgbuff[CAMERA_SIZE];

 Point  MidLine[Img_H] = { 0 };
 
int Hamming_weight(unsigned char n)		//ͳ�� 1 bits �ĸ���
{
	int count;
	for (count = 0; n; count++)
		n &= n - 1;
	return count;
}

//��������� �Ӱױ�λ�� 0000 1111 ���������λ��
int RightJump(unsigned char a)
{
    int  i;
    if (a >= 0x0f )//��ߴ��ںڵ� �����������
    {
        a >>= 4;
        for (i = 0; i < 4; i++)
        {
            if ((a&(1 << i)) == 0)break;
        }
        i += 4;
    }
    else {//�ұ�������
        for (i = 0; i < 4; i++)
        {
            if ((a&(1 << i)) == 0)break;
        }
    }
    return 8 - i;
}

//���������  �Ӻڱ�ɰ� 1111 0000
int LeftJump(unsigned char a)
{
    int i = 0;
    if ((a & 0x0f) != 0x0f)//�ұ��ĸ�������1
    {
        for (i = 0; i < 4; i++)
        {
            if ((a >> i) & 0x01 == 1)break;
        }
        i = 8 - i;
    }
    else//������ĸ��ҵ������
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

//��̺ʽ���� ����ǰ���� 
//�ҵ���һ��������һ�ΰ��ߵ��е�
int CarpetSearch(int row)
{
    int ans;
    int len = 0, Maxlen = 0;
    int start, end,mid=0;
    int i,j;
    for (i = 0; i < Img_W; i++)
    {
      if (img[row][i] != 0xff)//���ְ׵�
      {
        start = LeftJump(img[row][i])+i*8;//������
        for (j = i+1; j < Img_W; j++)
        {
          if (img[row][j] != 0x00)//���ֺڵ�
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

int LeftSearch(unsigned char *a, int start) //����������һ������λ�� 
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
//������
int Search(unsigned char *MidLine)
{
    unsigned char mid=4,last=4;
    int i,j;
    int len=0;
    MidLine[0] =(LeftSearch(img[Img_H-1],mid)+RightSearch(img[Img_H-1],mid))/2;
    for(i=Img_H-2;i>0;--i)
    {
      last=MidLine[Img_H-i-2]/8;
      
      if(img[i][last]!=0xff)//�е㲻Ϊ��ɫ 
      {
              MidLine[Img_H-i-1]=(LeftSearch(img[i],last)+RightSearch(img[i],last))/2;
              //����һ������λ���������� 
      }
      else//�м���ֺ�ɫ 
      {
        break;         
      }
            
    }
    len=Img_H-i;
    return len;
}

//���� ��x,yλ�ÿ�ʼ�ݹ������� �ҵ�������ͨ����
//����˳�������ϡ����ϡ����ϡ����ҡ����¡����¡����� 
//������ͨ�ж� �� ������ͨ�ж��ǲ�ͬ�ķ���
void DFS(int x,int y)//���
{
    if (x<0 || x>=Img_H || y<0 || y>=Img_W)return;//�������
    if (ConGraph[x][y] != 0 ) return;//�Ѿ��������������ݹ�
    ConGraph[x][y] = 1;//�����ͨ
    IsConnect[x]=1;
    if (x > 0)//���Ϸ��� 
    {
        if (ColConnect(img[x][y], img[x - 1][y]))
            DFS(x - 1, y);
    }
    if (y > 0 )//������
    {
        if (RowConnect(img[x][y-1], img[x][y]))
            DFS(x, y - 1);
    }
    if (y < Img_W-1)
    {
        if (RowConnect(img[x][y] , img[x][y + 1]))
            DFS(x, y + 1);
    }
    if (x < Img_H-1)//������
    {
        if (ColConnect(img[x][y], img[x + 1][y]))
            DFS(x + 1, y);
    }
}

/***************************************
    ����б��Ѱ���е�λ��
**************************************/
int SlopeSearch(int *t,Point *Mid)
{
    int x0, y0,x1,y1;//���������
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
          
    if (ImgMap(Img_H - y0, x0) == 1)//����������Ϊ�ڵ�
    {
        Mid[*t].x=CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
        (*t)++;
        Mid[*t].x = CarpetSearch(Img_H - *t);
        Mid[*t].y = *t;
    }
    else 
    {
        if (x0 <x1 + 10 && x0>x1 - 10)//��ֱ���ϣ�б��Ϊ���� 
        {//ˮƽ����
            for (i = x0 - 1; i >= 0; i--)//�ϱߵĽ���
                if (ImgMap(Img_H - y0, i) == 1)break;
            left.x = i;
            left.y = y0;

            for (i = x0 + 1; i < Img_W * 8; i++)//�±ߵĽ���
                if (ImgMap(Img_H - y0, i) == 1)break;
            right.x = i;
            right.y = y0;
        }
        else if (y0 <y1 + 3 && y0>y1 - 3)//ˮƽ�� б��Ϊ0
        {//��ֱ����
            for (i = y0 - 1; i >= 0; i--)//��ߵĽ���
                if (ImgMap(Img_H - i, x0) == 1)break;
            left.x = x0;
            left.y = i;

            for (i = y0 + 1; i < Img_H; i++)//�ұߵĽ���
                if (ImgMap(Img_H - i, x0) == 1)break;
            right.x = x0;
            right.y = i;

        }
        else {
            k = (y1 - y0) / (x1 - x0);
            k1 = -1 / k;
            //y-y0=k1(x-x0) => y=k1(x-x0)+y0;
            for (i = x0 - 1; i >= 0; i--)//��ߵĽ���
            {
                fx = (int)(k1*(i - x0) + 0.5 + y0);
                if (fx < 0 || fx >= Img_H)break;
                if (ImgMap(Img_H - fx, i) == 1)break;
            }
            left.x = i;
            left.y = fx;

            for (i = x0 + 1; i < Img_W * 8; i++)//�ұߵĽ���
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

//��ȡ����
int GetMidLine(Point *MidPoint)
{
    int i,len;
   // uint8 x0, y0,x1,y1;//���������
     //�������·�����
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
    for (i = 10; i < len-2; i++)//�򵥵��˲� Ӧ��ûʲô����
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
  ͳ����ÿ���ж�����ͨ��
  ֻͳ�Ƹ��������ж��м��Ƿ���ַ���ͨ��
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
ͳ����ͨͼ�е���Ϣ
ÿ���ж�����ͨ��
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

int ImageProcess()//ͼ�������
{
  int i,j;
  int len=0,x0=0;
  
  for(i=0;i<Img_H;i++)
    for(j=0;j<Img_W;j++)
    img[i][j]=imgbuff[i*10+j];
  //Correction();           //У��ͼ��浽img�� 
  
  memset(ConGraph, 0, 600);
  memset(IsConnect,0,60);
  memset(ConNum,0,sizeof(ConNum));
  
  //ȷ����ͨͼ
  for(i=1;i<10;i++)
  {
    x0=CarpetSearch(Img_H - i);
    if(x0>10 && x0<70)break;
  }
  
  DFS(59,(x0+7)/8);

  len=CountGraph(ConNum);//ͳ��ÿһ����ͨ������Լ�����
  for(i=0;i<60;i++)//����ͨ�������
  {
    for(j=0;j<10;j++)
    {
      if(ConGraph[i][j]==0)
        img[i][j]=0xff;  

      imgbuff[i*10+j]=img[i][j];//��ͼ���µ���ʾ������
    }
  }
  len=GetMidLine(MidLine);
  
  return len;
}
