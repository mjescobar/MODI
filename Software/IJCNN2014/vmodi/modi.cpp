#include "modi.h"
#include "math.h"
#include <stdlib.h>


unsigned char	red[] = {255,0,0};
unsigned char	green[] = {0,255,0};
unsigned char	blue[] = {0,0,255};

modi::modi(){
  pos.x = 0;
  pos.y = 0;
  angle = 0;
  ofsx = 0;
  ofsy = 0;
  range = 100;
  // virtual sensors
  scen = (pt*) malloc(sizeof(pt)*range);
  slef = (pt*) malloc(sizeof(pt)*range);
  srig = (pt*) malloc(sizeof(pt)*range);
  sval_c = range;
  sval_l = range;
  sval_r = range;
  speed = 0x32;
  for(int i = 0; i < range; i++){
    scen[i].x = ofsx + 0;
    scen[i].y = ofsy + i;
    srig[i].x = ofsx + -i;
    srig[i].y = ofsy + i;
    slef[i].x = ofsx + i;
    slef[i].y = ofsy + i;
  }
  patt_id = 0;
  k_index = 0;

  // controller specific
  state = -1;
  thr_c = false;
  thr_l = false;
  thr_r = false;
  // -------------------
  pfile = NULL;
}
modi::modi(int inoffx, int inoffy, int inrange){
  pos.x = 0;
  pos.y = 0;
  angle = 0;
  ofsx = inoffx;
  ofsy = inoffy;
  range = inrange;
  // virtual sensors
  scen = (pt*) malloc(sizeof(pt)*range);
  slef = (pt*) malloc(sizeof(pt)*range);
  srig = (pt*) malloc(sizeof(pt)*range);
  sval_c = range;
  sval_l = range;
  sval_r = range;
  speed = 60;
  for(int i = 0; i < range; i++){
    scen[i].x = ofsx + 0;
    scen[i].y = ofsy + i;
    srig[i].x = ofsx + -i;
    srig[i].y = ofsy + i;
    slef[i].x = ofsx + i;
    slef[i].y = ofsy + i;
  }
  patt_id = 0;
  k_index = 0;

 // controller specific
  state = -1;
  thr_c = false;
  thr_l = false;
  thr_r = false;
  // -------------------

  pfile = NULL;
}
//set the current position of the robot
void modi::setpos(int inx, int iny){
	pos.x = inx;
	pos.y = iny;
}
//updates the pixels which are inside the range of the sensors
void modi::updateSenPix(){
  double ang = (2*M_PI/360)*angle;
  double cosang = cos(ang);
  double sinang = sin(ang);
  for(int i = 0; i < range; i++){

    scen[i].x = round(cosang*ofsx - sinang*(i+ofsy)) + pos.x;
    scen[i].y = round(sinang*ofsx + cosang*(i+ofsy)) + pos.y;

    srig[i].x = round(cosang*(-i+ofsx) - sinang*(i+ofsy)) + pos.x;
    srig[i].y = round(sinang*(-i+ofsx) + cosang*(i+ofsy)) + pos.y;

    slef[i].x = round(cosang*(i+ofsx) - sinang*(i+ofsy)) + pos.x;
    slef[i].y = round(sinang*(i+ofsx) + cosang*(i+ofsy)) + pos.y;

  }
}
//old version
void modi::updateSDist(bmap bitmap){
  int lat_range = (int) round(range*0.71);
  for(int i = 0; i < range; i++){
    if(bitmap.get(scen[i].x,scen[i].y,0) ==255){
      sval_c = i;
      break;
    }
    if(i == (range-1)) sval_c = range;
  }
  for(int i = 0; i < lat_range; i++){
    if(bitmap.get(slef[i].x,slef[i].y,0) ==255){
      sval_l = (int) round(i*1.41);
      break;
    }
    if(i == (lat_range-1)) sval_l = range;
  }
  for(int i = 0; i < lat_range; i++){
    if(bitmap.get(srig[i].x,srig[i].y,0) ==255){
      sval_r = (int) round(i*1.41);
      break;
    }
    if(i == (lat_range-1)) sval_r = range;
  }
}
/*
void modi::updateSDist(bmap bitmap){
  int lat_range = (int) round(range*0.71);
  int tc, tl, tr;
  tc = tl = tr = 1;
  for(int i = 0; i < range; i++){
    if(tc) sval_c = i;
    if(tl&&i<lat_range) sval_l = (int) round(i*1.41);
    if(tr&&i<lat_range) sval_r = (int) round(i*1.41);

    if(bitmap.get(scen[i].x,scen[i].y,0) ==255) tc = 0;
    if(bitmap.get(slef[i].x,slef[i].y,0) ==255) tl = 0;
    if(bitmap.get(srig[i].x,srig[i].y,0) ==255) tr = 0;

    if(!tc&&!tl&&!tr) break;
  }
    if(sval_l>lat_range) sval_l = range;
    if(sval_r>lat_range) sval_r = range;
}
*/
void modi::initLogger(char *filename){
  pfile = fopen(filename,"w");
}

void modi::report(bool copy2file){
  printf("modi: x:%d, y:%d, a:%f, sl:%d, sc: %d, sr: %d, speed:%d\n",pos.x, pos.y, angle, px2cm(sval_l), px2cm(sval_c), px2cm(sval_r),speed);
  if(copy2file)
    if(pfile!=NULL) fprintf(pfile, "%d,%d,%f,%d,%d,%d,%d\n",pos.x, pos.y, angle, sval_l, sval_c,sval_r,speed);

}
void modi::setval(unsigned char *dataPtr,int image_width, int image_height, int x, int y, unsigned char *value){
    if(x>0&&y>0&&x<image_width&&y<image_height){
	dataPtr[image_width*3*y+x*3] = value[0];
	dataPtr[image_width*3*y+x*3+1] = value[1];
	dataPtr[image_width*3*y+x*3+2] = value[2];
    }
}

void modi::drawSensorsInto(unsigned char *dataPtr,int width, int height){
  for(int i = 0; i < range; i++){
    if(i< sval_c) setval(dataPtr, width, height, scen[i].x, scen[i].y,green);
    if(i< round(sval_l*0.71)) setval(dataPtr, width, height, slef[i].x, slef[i].y,red);
    if(i< round(sval_r*0.71)) setval(dataPtr, width, height, srig[i].x, srig[i].y,blue);
  }

}

bool modi::isVisible(){
  return (k_index>-1);
}

void modi::setState(int instate){
  state = instate;
}

