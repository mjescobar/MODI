#ifndef _VMODI_H_
#define _VMODI_H_

#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <xbee.h>
#include <string.h>
#include "modi.h"
#include "decontroller.h"
#include "drcontroller.h"
#include "drcontroller2.h"

//
// Camera configuration.
//
char *vconf = (char *) "";

int xsize, ysize;
int thresh = 100;
int count = 0;

char *cparam_name    = (char *) "Data/camera_para.dat";
ARParam cparam;

char *patt_name1      = (char *) "Data/patt.cubito";
int patt_id1;


double patt_width     = 80.0;
double patt_center[2] = {0.0, 0.0};


double patt_trans2[3][4];

int image_width = 1920;
int image_height = 1080;
unsigned char *pframe;

// artoolkit

static void   init(char * bmpName, char *outFile);
static void   cleanup(void);
static void   keyEvent( unsigned char key, int x, int y);
static void   mainLoop(void);
static void   draw(  double ** patt_trans, int r,int g, int b );


// xbee communication

struct xbee *xbee;
struct xbee_con *con;
struct xbee_conAddress address;
unsigned char txRet;
xbee_err ret;
int cont;
unsigned char rotation[4] = {0x02,0x02,0x32,0x00};

/// controllers
decontroller deCont;
drcontroller drCont;
drcontroller2 drContV2;
int controller_version2;


// obtains the angle of a certain fiducial
double object_angle(double (*source)[4],double *pos){
	double rot_quat[4],dummy3[3];
	double inv[3][4];
	double sign;
	double angle;
	arUtilMat2QuatPos(source, rot_quat, pos);
	arUtilMatInv(source,inv);
	arUtilMat2QuatPos(inv, rot_quat, dummy3);
	sign = ((rot_quat[0]*rot_quat[1])>0)? 1 : -1;
	angle = sign*rot_quat[1]*rot_quat[1]*180;
	return angle;
}

// set an arbitrary value in a certain position of the image
void setval(ARUint8 *dataPtr,int x, int y, unsigned char *value){
    if(x>0&&y>0&&x<image_width&&y<image_height){
	dataPtr[image_width*3*y+x*3] = value[0];
	dataPtr[image_width*3*y+x*3+1] = value[1];
	dataPtr[image_width*3*y+x*3+2] = value[2];
    }
}
// draws a rectangle over the image
void drect(ARUint8 *dataPtr, int x, int y, int width, int height,int filled, unsigned char *color){
	int qq = 0;
	for (qq = 0; qq < width; qq++){
		setval(dataPtr,x+qq,y,color);
		setval(dataPtr,x+qq,y+height,color);
	}
	for (qq = 0; qq < height; qq++){
		setval(dataPtr,x,y+qq,color);
		setval(dataPtr,x+width,y+qq,color);
	}
}
#endif

