#include "include.h"
#include "image.h"
#include "MK60_gpio.h"

#include "arm_math.h"

#define ColConnect(a,b) (a|b)!=255				//����ͨ 1��ͨ
#define RowConnect(a,b) (0x01&((a) | ((b) >>7)))==0	//����ͨ 0Ϊ��ͨ


uint8   stopflag=0;
uint8   startgogo=0;
//uint8   zhili=0;
uint8   wending=0;
uint8   biansu=0;
uint8   biancan=0;
uint8  zhuantou=0;

//����ͨͼ
unsigned char ConGraph[Img_H][Img_W] = { 0 };//��ͨͼ
uint8 IsConnect[Img_H]={0};
extern uint8 img[60][20];
//�ɼ��õ�
//uint8 Buffer[120];
//˫����ɼ��õ�
uint8 processReady=0;
uint8 wantProcess=0;
//�ɼ��õ�
uint8 ImageBuffer1[SROW][SCOLUMN];
uint8 ImageBuffer2[SROW][SCOLUMN];

uint8 *pSample=ImageBuffer1[0];
uint8 *pProcess=ImageBuffer2[0];
uint8 *pTemp=ImageBuffer1[0];
//����
int leftYStart, leftYEnd, rightYStart, rightYEnd, leadYStart, leadYEnd;//����,���ߺ������ߵ�Y������ʼ����ֹ��
int LineType;  //��������
int leftLine[PROW];
int rightLine[PROW];
int leadLine[PROW];
int ShowArray[PROW];//��λ����ʾ����
int WIDTH=30;//��¼�������
int leadlength;
int midpoint_before = HEART,midpoint_before_E = HEART;

int offset_1;//�˴���offset_1  ���ɸ��ı�����Ϊoffset��Ұ���Ĺؼ��ְ�����Ȼ��ô���Ķ���֪��
int offset_2;
int dOffset;
int oldOffset;
int newOffset;
float Cuvre[4];

float CircleRate=0;


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
int AAGAFilter(int *src)
{ 
 
   int i=0,Value=0;
   int sum=0,CoeSum=1830;
   int Coefficient[60]; 

   for(i=1;i<61;i++) //  ���
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
/*****************************************Ѱ��˫��***********************************/
//��¼������
void RecordBWChange(uint8 *src,uint8 *pSrc)//P1 P2
{
    int num1, num2;
   ArraySetValue(pSrc, 255); //�������飬P2ȫ��0 
    for (int i = 0; i < PROW; i++)//ÿһ��
    {
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
        }
       
 
    }
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
    int i,j,m,n;
    //���֮ǰ��Ѱ������
    leftYStart = -1; rightYStart = -1; leftYEnd = -1; rightYEnd = -1; leadYStart = -1; leadYEnd = -1;
    zhuantou=0;
    for (i = 0; i < PROW; i++)
    {
        leftLine[i] = 0;
        rightLine[i] = 0;
        leadLine[i] = 0;
    }

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
            for (m = 0; m < leftCnt; m++)
            {
                for (n = 0; n < rightCnt; n++)
                {
                    t1 = rightHalf[n] - leftHalf[m];//���ҵ���
                    //1cm  Ϊ1.3����
                    //�о��˷����ý���У����ʹ��  
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
    int y1, y2, y3, y4, yl, yr;
    float divide;
    //ֻ������(��ȫû����)
    if (leftYStart != -1 && rightYStart == -1)
    {
        leadYStart = leftYStart;
        leadYEnd = leftYEnd;
        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            leadLine[i] = leftLine[i] + WIDTH / 2;
        }
    }
    //ֻ������(��ȫû����)
    else if (leftYStart == -1 && rightYStart != -1)
    {
        leadYStart = rightYStart;
        leadYEnd = rightYEnd;
        for (int i = leadYStart; i <= leadYEnd; i++)
        {
            leadLine[i] = rightLine[i] - WIDTH / 2;
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
                leadLine[i] = leftLine[i] + WIDTH / 2;
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
                leadLine[i] = rightLine[i] - WIDTH / 2;
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
                    leadLine[i] = leftLine[i] + WIDTH / 2;
                }
                else
                {
                    leadLine[i] = rightLine[i] - WIDTH / 2;
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
                    leadLine[(yl + yr) / 2] = (leftLine[yl] + rightLine[yr]) / 2;
                }
            }
            CompleteLine(leadLine);
        }
    }
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




//ͼ�����㷨
/***********************************/
uint8 tr,staticTr;
void imageProcess(uint8 *src)//srcУ�����ͼ��
{
        int8 re;
        uint8 i;
        
        //Ѱ˫��
        RecordBWChange(src,p2);//��¼������

        re=SearchBaseLines(p2);//Ѱ˫��
        CrossRecognize(p1,p2);
        CalculateLeadLine();//���������� 
        leadlength=leadYEnd-leadYStart;
        
        Cuvre[0]=CuvreControl(leadYStart+leadlength*2/3,leadYEnd);
        Cuvre[1]=CuvreControl(leadYStart+leadlength/3,leadYStart+leadlength*2/3-1);
        Cuvre[2]=CuvreControl(leadYStart,leadYStart+leadlength/3-1);
       // Cuvre[3]=(Cuvre[0]*7+Cuvre[1]*2+Cuvre[2])/(3.0*10.0); 
        midpoint_before=AAGAFilter(leadLine);//��Ȩƽ���˲�
        
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