#ifndef __IMAGE_H__
#define __IMAGE_H__

#define Img_W  10  //Í¼ÏñµÄ(¿í/8)
#define Img_H 60   //Í¼ÏñµÄ¸ß
typedef struct  {
	int x;
	int y;
}Point;

int Search(unsigned char *MidLine);
void DFS(int x,int y);
int ImageProcess();
#endif