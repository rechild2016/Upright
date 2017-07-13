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

//�ɼ��õ�
uint8 Buffer[120];
//˫����ɼ��õ�
uint8 processReady=0;
uint8 wantProcess=0;
//�ɼ��õ�
uint8 ImageBuffer1[SROW][SCOLUMN];
uint8 ImageBuffer2[SROW][SCOLUMN];
//��ͨͼ
unsigned char ConGraph[60][20] = { 0 };//��ͨͼ
uint8 IsConnect[60]={0};
uint8 ConNum[60]={0};
//����ͨͼ
extern uint8 img[60][20];
#define ColConnect(a,b) (a|b)!=255				//����ͨ 1��ͨ
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//����ͨ 0Ϊ��ͨ
uint8 *pSample=ImageBuffer1[0];
uint8 *pProcess=ImageBuffer2[0];
uint8 *pTemp=ImageBuffer1[0];
//����
int Linemid_1[PROW]={0};
int xyz=-1;
int huan_sign=0;
//����
int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd;//����,���ߺ������ߵ�Y������ʼ����ֹ��
int LineType;  //��������
int leftLine[PROW];
int rightLine[PROW];
//�ϰ��������
int block_hang=0;
int leadLine[PROW];
int ShowArray[PROW];//��λ����ʾ����
int Ping[5];  //ƽ���˲� ���ڿ�ɾ
int WIDTH=36;//��¼�������
int leadlength;
int midpoint_before = HEART,midpoint_before_E = HEART  ;
int Ruhuan_sign=0;
int offset_1;//�˴���offset_1  ���ɸ��ı�����Ϊoffset��Ұ���Ĺؼ��ְ�����Ȼ��ô���Ķ���֪��
int offset_2;
int dOffset;
int oldOffset;
int newOffset;
float Cuvre[4];

float CircleRate=0;

uint8 STOP=0;

uint8 processBuf1[PROW][PCOLUMN];//��ѹ����ǰ��
uint8 processBuf2[PROW][PCOLUMN];//������
uint8 *p1=processBuf1[0];
uint8 *p2=processBuf2[0];

/***********************************/
//����������ֵ
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
//��Ȩ����ƽ���˲�
int     AAGAFilter(int *src)
{ 
 
   int i=0,Value=0;
   int sum=0,CoeSum=465;
   int Coefficient[30]; 

   for(i=1;i<31;i++) //  ���
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
//�����߽�
int leftEdge1[PROW]={0};
int rightEdge1[PROW]={0};

void Search_the_boundary(void)
{
	int i,j;
	for(i=59;i>=0;i--)//��߽�
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
	for(i=59;i>=0;i--)//�ұ߽�
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
/*****************************************Ѱ��˫��***********************************/
//��¼������
void RecordBWChange(uint8 *src,uint8 *pSrc)//P1 P2
{
        int num1, num2;
	int two_white_sign=0,two_black_sign=0,usefull_hang=0;
        block_hang=0;
   ArraySetValue(pSrc, 255); //�������飬P2ȫ��0 
    for (int i = 0; i < PROW; i++)//ÿһ��
    {   two_white_sign=0;
        two_black_sign=0;
        
       for (int j = leftEdge[i]; j <  rightEdge[i]; j++)
        {
            num1 = *(src+i*PCOLUMN+j);
            num2 = *(src+i*PCOLUMN+j+1);
            if (num1 == 0 && num2 == 255)//0�Ǻڣ�255��
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
					//�� �� ��
                                          
					two_black_sign=0;//  ��λʹ��ÿһ��ֻ����һ��
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
///**************��λ���˲�*********/
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
//�������ؼ�¼�������Ѱ��������
int leftHalf[PCOLUMN/2];
int rightHalf[PCOLUMN/2];
int leftHalf0[PCOLUMN/2];
int rightHalf0[PCOLUMN/2];
int SearchBaseLines(uint8 *pSrc)//P2
{
    //��������
    int num, t1, leftCnt, rightCnt;
    int i,j,m,n,lie;
    //���֮ǰ��Ѱ������
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
    for(i=0;i<PROW;i++)//���������У���ȡ������ȣ�
    {
        if(WIDTH[59]!=0)break;//�Ѷ�ȡ����������˳�       
        leftCnt = 0;//�ҵ��ĵ���
        for (j = HEART; j > 0; j--)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                leftHalf0[leftCnt++] = j;//�ڼ����������
            }
        }
        //��������
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
    //���������·�����(40)�У�ָ��ʵ�е�����
    for (i = 0; i < MAX_SEARCH_HEIGHT; i++)
    {
        //��������
        leftCnt = 0;//�ҵ��ĵ���
        for (j = HEART; j > leftEdge[PROW - 1 - i]; j--)
        {
            num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
            if (num == 1)
            {
                leftHalf[leftCnt++] = j;//�ڼ����������
                //break;
            }
        }

        //��������
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
		
        //ͳ���������
        if (leftCnt == 0 && rightCnt!=0)//�ұ��������û��
        {
            rightYStart = i;
            rightYEnd = i;
            rightLine[i] = rightHalf[0];//��i������ߵ����������
        }
        else if (rightCnt == 0 && leftCnt != 0)//��������ұ�û��
        {
            leftYStart = i;
            leftYEnd = i;
            leftLine[i] = leftHalf[0];//��i������ߵ����������
        }
        else if (leftCnt != 0 && rightCnt != 0)//���Ҷ�����
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
                    t1 = rightHalf[n] - leftHalf[m];//���ҵ���
                    //1cm  Ϊ1.3����
                    //�о��˷����ý���У����ʹ�� 
                    /*if(t1<WIDTH - 8)
                    {
                    	lie++;
						if(lie>=2)STOP=1;
                    }*/
                    
                    if (t1 < WIDTH + 8 && t1 > WIDTH - 8)//�����������෶Χ��  
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
                if (leftYStart != -1 && rightYStart != -1)//�ҵ�����������ѭ��
                {
                    break;
                }
            }
		  
        }
        //�ѵ��뿪
        if (leftYStart != -1 || rightYStart != -1)//����һ��������
        {
            if (leftYStart != -1)//�������ѵ�
            {
                if (rightYStart != -1)
                {
                    //����ͬʱ�ѵ�
                    break;
                }
                else
                {
                    //ֻ�ѵ����
                    EdgeTrace(pSrc, leftLine);//��������
                    if (leftYEnd != leftYStart)
                    {
                        CompleteLine(leftLine);//���в���
                        break;
                    }
                }
            }
            else
            {
                //ֻ�ѵ��ұ�
                EdgeTrace(pSrc, rightLine);//��������
                if (rightYEnd != rightYStart)
                {
                    CompleteLine(rightLine);//���в���
                    break;
                }
            }
        }
    }//��������������

    //δ�ѵ�����
    if (leftYStart == -1 && rightYStart == -1)
    {
        return -1;
    }
	
    //�ѵ�˫�����жϾ����Ƿ�ϸ�
    if (leftYStart != -1 && rightYStart != -1)
    {
        EdgeTrace(pSrc, leftLine);//��������
        CompleteLine(leftLine);//������
        EdgeTrace(pSrc, rightLine);//��������
        CompleteLine(rightLine);//������
    }
	
    //ֻ�ѵ����������߸��ٱ�Ե
    else if (leftYStart != -1 && rightYStart == -1)
    {
        //�����ߵ�ÿһ������Ѱ��,����У����ľ���,�Դ���Ϊ�ڵ��Ƿ�ϸ������
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
                EdgeTrace(pSrc, rightLine);//��������
                CompleteLine(rightLine);//������
                break;
            }
        }
    }
    //ֻ�ѵ��ұ������ұ߸��ٱ�Ե
    else if (leftYStart == -1 && rightYStart != -1)
    {
        //�����ߵ�ÿһ������Ѱ��,����У����ľ���,�Դ���Ϊ�ڵ��Ƿ�ϸ������
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
                EdgeTrace(pSrc, leftLine);//��������
                CompleteLine(leftLine);//������
                break;
            }
        }
    }
    return 1;
}


/*********************************************/
//���ٱ�ԵѰ��
void EdgeTrace(uint8 *pSrc, int line[])//P2  ��������
{
    int xMid, xStart, xEnd, num, lost,i,j;
    //����
    if (line == leftLine)
    {
        //��ʼ���������Ե�����
        lost = 0;
        xMid = line[leftYStart];
        for (i = leftYStart + 1; i < PROW; i++)
        {
            //������һ���ҵ�������һ��Ϊ��׼
            if (line[i - 1] != 0)
            {
                xMid = line[i - 1];
            }
			
            //��ʼ�����
            xStart = xMid + MAX_XDIS;
            if (xStart > rightEdge[PROW - 1 - i]) { xStart = rightEdge[i]; }//������У����ı߽�
			
            xEnd = xMid - MAX_XDIS;
            if (xEnd < leftEdge[PROW - 1 - i]) { xEnd = leftEdge[i]; }
			
            for (j = xStart; j >= xEnd; j--)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                //�ҵ����¼������ն������
                if (num == 1)
                {
                    leftYEnd = i;
                    line[i] = j;
                    lost = 0;
                    break;
                }
            }
            //δ�ҵ��򶪵����ֵ��һ
            if (leftYEnd != i)
            {
                lost++;
            }
            //�������������˳�
            if (lost > MAX_LOST)
            {
                return;
            }
        }//ѭ������
    }

	
    //����
    else if (line == rightLine)
    {
        //��ʼ���������Ե�����
        lost = 0;
        xMid = line[rightYStart];
        for (i = rightYStart + 1; i < PROW; i++)
        {
            //������һ���ҵ�������һ��Ϊ��׼
            if (line[i - 1] != 0)
            {
                xMid = line[i - 1];
            }
            //��ʼ�����
            xStart = xMid - MAX_XDIS;
            if (xStart < leftEdge[PROW - 1 - i]) { xStart = leftEdge[i]; }
            xEnd = xMid + MAX_XDIS;
            if (xEnd > rightEdge[PROW - 1 - i]) { xEnd = rightEdge[i]; }
            for (j = xStart; j <= xEnd; j++)
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                //�ҵ����¼������ն������
                if (num == 1)
                {
                    rightYEnd = i;
                    line[i] = j;
                    lost = 0;
                    break;
                }
            }
            //δ�ҵ��򶪵����ֵ��һ
            if (rightYEnd != i)
            {
                lost++;
            }
            //�������������˳�
            if (lost > MAX_LOST)
            {
                return;
            }
        }
    }
}

//����
void CompleteLine(int line[])
{
    int i, j, start = 0, end = PCOLUMN - 1, num, y1 = 0, y2 = 0, t1, t2;
    float slope;
    if (line == leftLine) { start = leftYStart; end = leftYEnd; }
    else if (line == rightLine) { start = rightYStart; end = rightYEnd; }
    else { start = leadYStart; end = leadYEnd; }
    //�Բ���0���������㰴б�ʲ���
    for (i = start; i <= end; i++)
    {
        num = line[i];
        if (num == 0)//���ֶ�ʧ��
        {
            if (y1 == 0)
            {
                y1 = i - 1;//y1Ϊ��ʧǰһ��������
            }
        }
        if (num != 0 && y1 != 0)
        {
            y2 = i;//��ʧ��ĵ�һ��������
            t1 = line[y1];//������
            t2 = line[y2];
            slope = ((float)t2 - t1) / ((float)y2 - y1);
            for (j = y1 + 1; j < y2; j++)
            {
                
                line[j] = (int)(t1 + slope * (j - y1) + 0.5);//0.5Ϊ��������
                if(processBuf1[line[j]][j]==0) /////////                    
                  line[j] =0;                 /////////2017.7.9�ӣ��ж��ǲ��Ǻ��ߣ����߲�����
            }
            y1 = 0;
            y2 = 0;
        }
    }
}


/*************************************************************************************/

/****************************************��������ȡ***********************************/
//����������
void CalculateLeadLine()
{
    int y1, y2, y3, y4, yl, yr,leftLine_before=60,rightLine_before=100;
    float divide; 
    int sign=0;
    //��������Ϊ��ĵط�����
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
        //ֻ������(��ȫû����)      
            
    if (leftYStart != -1 && rightYStart == -1)
    {   
        leadYStart = leftYStart;
        leadYEnd = leftYEnd;
        

        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            
          //��������ұ��ǲ��Ǻ��ߣ��Ǻ��߾�1  ���� -1
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
    //ֻ������(��ȫû����)
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
    //˫��
    else if (leftYStart != -1 && rightYStart != -1)//���Ҷ�����
    {
       
        //��ʢ��˥
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
        //��ʢ��˥
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
        //����Э��
        else
        {
            //FM ( 1) ;    
            //DELAY_MS ( 150 ) ;
            //FM ( 0 ) ;
           
            y1 = leftYStart<rightYStart?leftYStart:rightYStart;//SС
            y2 = leftYStart<rightYStart?rightYStart:leftYStart;//S��
            y3 = leftYEnd<rightYEnd?leftYEnd:rightYEnd;//EС
            y4 = leftYEnd<rightYEnd?rightYEnd:leftYEnd;//E��
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
            for (int i = y2; i <= y3; i++)//��������Ч������˵��û����
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
//���� ��x,yλ�ÿ�ʼ�ݹ������� �ҵ�������ͨ����
//����˳�������ϡ����ϡ����ϡ����ҡ����¡����¡����� 
//������ͨ�ж� �� ������ͨ�ж��ǲ�ͬ�ķ���
void DFS(int x,int y)//���
{
    if (x<0 || x>=60 || y<0 || y>=20)return;//�������
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
    if (y < 20-1)
    {
        if (RowConnect(img[x][y] , img[x][y + 1]))
            DFS(x, y + 1);
    }
    if (x < 60-1)//������
    {
        if (ColConnect(img[x][y], img[x + 1][y]))
            DFS(x + 1, y);
    }
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
    int start,end,mid=-1;
    int i,j;
    for (i = 0; i < 20; i++)
    {
      if (img[row][i] != 0xff)//���ְ׵�
      {
        start = LeftJump(img[row][i])+i*8;//������
        for (j = i+1; j < 20; j++)
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

//ȥ�����
int RemoveNoise(int mode)
{ 
    if (mode == 1)
    {
        //�߳��ж�
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
        //�߳��ж�
        if (leftYEnd - leftYStart < MIN_CROSS_LENGTH)
        {
            leftYStart = -1;
        }
        if (rightYEnd - rightYStart < MIN_CROSS_LENGTH)
        {
            rightYStart = -1;
        }
        //�˳�����������������������������
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
/***************************************ʮ�ֽ���ʶ��**********************************/
//ʮ�ֽ����ܺ���
void CrossRecognize(uint8 *src,uint8 *pSrc)//P1 P2
{
    int y2 = -1;
    //б��ʮ���ж�
    //����
    if (leftYStart != -1)
    {
        y2 = GetBiasDownRAngle(leftLine, leftYStart, leftYEnd);//�ж��Ƿ���б��ʮ��
        if (y2 != -1)//��б��ʮ��
        {
            //������� Y2Ϊת�۵�
            float *ab = GetLSMatchingLine(leftLine, leftYStart, y2);
            ab[1] += WIDTH / 2;//ƽ��
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
            //�������
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
    //����
    if (y2 == -1 && rightYStart != -1)
    {
        y2 = GetBiasDownRAngle(rightLine, rightYStart, rightYEnd);
        if (y2 != -1)
        {
            //�������
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
            //�������
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
	
    //ֱ��ʮ���ж�
    int lt, ly2 = -1, ry2 = -1;
    //����
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
            //�������
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
    //����
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
            //�������
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

//б��ʮ��ȷ����ֱ��
int GetBiasDownRAngle(int line[], int start, int end)//leftLine, leftYStart, leftYEnd
{
    //1Ϊ��������,-1Ϊ��������
    int m1, m2, mul;
    //ֱ�Ǽ��(б��ʮ��)
    for (int i = start + CROSS_LENGTH; i <= end - CROSS_LENGTH; i++)
    {
        m1 = line[i] - line[i - CROSS_LENGTH];
        m2 = line[i + CROSS_LENGTH] - line[i];
        mul = m1 * m2;
        if (mul < -CROSS_LENGTH * CROSS_LENGTH)
        {
            //б��ʮ��,ת�۵������
            for (int j = i + 1; j <= end; j++)
            {
                line[j] = 0;
            }
           
            return i;
        }
    }
    return -1;
}

//б��ʮ��ȷ����ֱ��
int GetBiasUpRAngle(int line[], int start, int end)
{
    //1Ϊ��������,-1Ϊ��������
    int m1, m2, mul;
    //ֱ�Ǽ��(б��ʮ��)
    for (int i = end - CROSS_LENGTH; i >= start + CROSS_LENGTH; i--)
    {
        m1 = line[i] - line[i - CROSS_LENGTH];
        m2 = line[i + CROSS_LENGTH] - line[i];
        mul = m1 * m2;
        if (mul < -CROSS_LENGTH * CROSS_LENGTH)
        {
            //б��ʮ��
            for (int j = i - 1; j >= start; j--)
            {
                line[j] = 0;
            }
            return i;
        }
    }
    return -1;
}

//������С�������ֱ��
float ls[2];
float *GetLSMatchingLine(int line[], int start, int end)//leftLine, leftYStart, y2
{
    int h1, sum1, sum2, sum3, sum4;//��С���˷�����
    //��С���˷����(��Y��X)XΪ׼ȷֵ��YΪ����ֵ
    sum1 = 0; sum2 = 0; sum3 = 0; sum4 = 0;
    h1 = end - start + 1;
    for (int i = start; i <= end; i++)
    {
        sum1 += line[i];//y���
        sum2 += PROW - 1 - i;//x���
        sum3 += (PROW - 1 - i) * line[i];//x*y���
        sum4 += (PROW - 1 - i) * (PROW - 1 - i);//x^2���
    }
    ls[0] = (float)(h1 * sum3 - sum1 * sum2) / (h1 * sum4 - sum2 * sum2);//a
    ls[1] = (float)(sum1 / h1 - ls[0] * ((float)sum2 / h1));//b
    return ls;
}

//�ж��߶�����
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
    //ֱ�߷���1�����߷���2
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

//����˫��Ѱ��
int ys[2];
int MIN_CON_POINT=5;
int *ReverseSearchLine(uint8 *pSrc, int line[], int start, float a, float b, int direction)
//p2, leftLine, leftYEnd, a, b, 1  
{
    int midTrue, midX, num, lastX = 0, conCnt = 0, lost = 0;
    ys[0]=-1;
    ys[1]=-1;
    //���Ѱ��
    if (direction == 1)
    {
        conCnt = 0;
        lost = 0;
        //ֱ�ӱ�Ե
        for (int i = PROW - 1; i > start + 5; i--)
        {
            midTrue = (int)(a * (PROW - 1 - i) + b);
            midX = midTrue;
            if (midX > rightEdge[PROW - 1 - i]) { midX = rightEdge[PROW - 1 - i]; }
            if (midX < leftEdge[PROW - 1 - i]) { midX = leftEdge[PROW - 1 - i]; }
            for (int j = midX; j > 0; j--)//���м������ɨ��
            {
                num = *(pSrc+(PROW - 1 - i)*PCOLUMN+j);
                if (num == 1)//�������
                {
                    if (midTrue - j > WIDTH / 2 + 10)
                    {
                        break;
                    }
					//J����������
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
        //ֱ�ӱ�Ե
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

//ֱ��ʮ��ȷ����ֱ��
int MAX_HORLINE_LENGTH = 15;
int GetStraightDownRAngle(uint8 *src, int line[], int start, int end, int direction)
{
    int pn=0,st=0;
    //����
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
                //ֱ��ʮ��
                return i - 1;
            }
        }
    }
    //����
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
                //ֱ��ʮ��
                return i - 1;
            }
        }
    }
    return -1;
}

//ֱ��ʮ��ȷ����ֱ��
int GetStraightUpRAngle(uint8 *src, int line[], int start, int end, int direction)
{
    int pn=0,st=0;
    //����
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
                //ֱ��ʮ��
                for (int j = i; j >= start; j--)
                {
                    line[j] = 0;
                }
                return i + 1;
            }
        }
    }
    //����
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
                //ֱ��ʮ��
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

//ʮ�ֽ��油��
void CrossMakeUpLine(int line[], int start, int end)
{
    int i, j, num, y1 = 0, y2 = 0, t1, t2;
    double slope;
    //�Բ���0���������㰴б�ʲ���
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
//��������1
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
//������Ĵ���
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
    //����
    if (direction == 1)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[end]; j < HEART; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//������
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
    //����
    else if (direction == 2)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[end]; j > HEART; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//������
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
    //����
    if (direction == 1)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[end]; j < HEART; j++)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//������
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
    //����
    else if (direction == 2)
    {
        if(end-start<=2){st=start+1;}
        else{st=end-2;}
        for (int i = st; i <= end; i++)
        {
            pn = 0;
            for (int j = line[end]; j > HEART; j--)
            {
                if (*(src+(PROW - 1 - i)*PCOLUMN+j) == 0)//������
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

//*************�ж�ʮ�ֻ���**************/
//    0Ϊֱ���뻷
//    1Ϊֱ����ʮ��
//    2Ϊб�뻷
//    3Ϊб��ʮ��
//    -1Ϊ������
int Crossorring(uint8 *src,uint8 *pSrc)
{
	
	int i,j,k,num,sum=0;
	int midY=-1;
	float a,b,c,d;
	int  ly1 = -1, ry1 = -1;
	int Linemid;
	int  ly2 = -1, ry2 = -1;
	int endmix,endmax,startmix,startmax;
	//ֱ��
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
		
		if ((ly2 != -1 && ry2 != -1)||(ly1 != -1 && ry1 != -1))//������ֱ��
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
					if(num == 0)//������
					{sum++;}
					if(sum>=10)return 0;//ֱ��Բ��
				}
			}
			if((endmax!=PROW - 1)&&(sum<3))
			{
				return 1;//ֱ��ֱ��
			}
		}	
	}
	/*else if(leftYStart != -1&&rightYStart == -1)//ֻ������
	{		
		midY=GetBiasDownRAngleX(leftLine, leftYStart, leftYEnd);
		if(midY!=-1)
		{
			
			float *abf = GetLSMatchingLine(leftLine, leftYStart, midY);
            a=abf[0];
			float *abb = GetLSMatchingLine(leftLine, midY+1, leftYEnd);
            b=abb[0];
			Angle=a*b;
			if(Angle<-3)return 2;//б�뻷
			else if(-3<Angle<0) return 3;//б��ʮ��
			else return -1;
                       
		}
		else return -1;
	}*/
	/*else if(leftYStart == -1&&rightYStart != -1)//ֻ������
	{
		midY=GetBiasDownRAngle(rightLine, rightYStart, rightYEnd);
		if(midY!=-1)
		{
			
			float *abf = GetLSMatchingLine(rightLine, rightYStart, midY);
            a=abf[0];
			float *abb = GetLSMatchingLine(rightLine, midY+1, rightYEnd);
            b=abb[0];
			Angle=a*b;
			if(Angle<-3)return 2;//б�뻷
			else if(-3<Angle<0) return 3;//б��ʮ��
			else return -1;                       
		}
		else return -1;
	}*/
	else return -1;
}




//ͼ�����㷨
/***********************************/
uint8 tr,staticTr;
void imageProcess(uint8 *src)//srcУ�����ͼ��
{
        int8 re;
        uint8 i;
        
        int test[5];       

        //Ѱ˫��
        RecordBWChange (src,p2);//��¼������

        
        re=SearchBaseLines(p2);//Ѱ˫��
        
        //if(re==-1) ArraySetValue(p2,255);
         // re=RemoveNoise(1);//�Ƴ�����
        //if(re==-1) ArraySetValue(p2,255);
        //ʮ�ֽ���
        //������
        //re=RemoveNoise(2);//�Ƴ�����
        //if(re==-1) ArraySetValue(p2,255);
        CrossRecognize(p1,p2);
        
       
        //xyz=Crossorring(p1,p2);
        CalculateLeadLine();//���������� 
        ///////��������
        huan_sign=judgebigring(20);
        if(huan_sign<20&&huan_sign>15)
        {  
            DELAY_MS ( 10 ) ;          
            for(int i=0;i<20;i++)
              leadLine[i]=leftEdge[i]+WIDTH / 3;    
        }
        
       ////////�����ϰ�
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
        midpoint_before=AAGAFilter(leadLine);//��Ȩƽ���˲�
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
///////////�жϴ�Բ������ȫ�ڵ�����
////////
////  black_hang_up  :�������ҵ��ĵ�һ��ȫ�������black_hang���������У�������ȫ�����ж�Ϊ����
int judgebigring(int black_hang_up)
{
	int plot_black_L=0,plot_white_L=0,all_black=0,all_white=0,parts_black=0,wide_hang=0,black_hang=0,i;
        int plot_black_R=0,plot_white_R=0,all_black_hang=0;   
        int plot_sign;
        //���ж���������Լʱ��  ���Ϊȫ�ڣ������Ϊȫ��
        for(uint8 y=leftEdge[1]+1;y<rightEdge[1]-1; y++)
        {
          
        if (processBuf1[1][y]==255)//0�Ǻڣ�255�� 
          {
            plot_sign++;
            if(plot_sign>4)
             return 0;                       
          } 
        }
        for(uint8 y=leftEdge[58]+1;y<rightEdge[58]-1; y++)
        {
          
        if (processBuf1[58][y]==0)//0�Ǻڣ�255�� 
          {
               plot_sign++;
            if(plot_sign>4)
             return 0;                      
          } 
        }
       //////////////////////////////////////////
        
	for (i = 0; i < PROW-10; i++)//ÿһ��
    {
         wide_hang=rightEdge[PROW-1-i]-leftEdge[PROW-1-i]-3;
        for (uint8 l = HEART,r=HEART;l>leftEdge[PROW-1-i]+1,r<rightEdge[PROW-1-i]-1; l--,r++)
        {      
          if (processBuf1[PROW-1-i][HEART]==0)//0�Ǻڣ�255�� 
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
          if(((plot_black_L+plot_black_R)>=wide_hang/4)&&(plot_white_L>=2)&&(plot_white_R>=2))   ////�м���ֺ�ɫ�������ǰ�ɫ
           {
           parts_black++;
            }
           if((plot_black_L+plot_black_R)>=wide_hang)
           {
            all_black++;
           if(parts_black>=3)
           {black_hang=i;    //����ȫ�ڵ�����
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
        ///////////////////////////////���������м�ڣ����߰� ���������������������󣬼������� black_hang���ǲ��Ǵ��ڼ���ȫ��
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
           if(processBuf1[PROW-1-j][r]==0)//0�Ǻڣ�255�� )
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