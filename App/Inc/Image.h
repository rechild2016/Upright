#ifndef __IMAGE_H__
#define __IMAGE_H__

#define Img_W  10  //ͼ���(��/8)
#define Img_H 60   //ͼ��ĸ�
typedef struct  {
	int x;
	int y;
}Point;

int Search(unsigned char *MidLine);
void DFS(int x,int y);
int ImageProcess();
#endif