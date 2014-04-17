#ifndef _MODI_H_
#define _MODI_H_
#include "bmap.h"
#include <math.h>
#include <stdio.h>

typedef struct pt{
  int x;
  int y;
}pt;

class modi{
public:
  modi();
  modi(int offsx, int offsy, int range);
  void setpos(int x, int y);
  void updateSenPix();
  void updateSDist(bmap bitmap);
  void drawSensorsInto(unsigned char *dataPtr,int width, int height);
  bool isVisible();
  void report(bool copy2file);
  void setState(int instate);
  void initLogger(char *filename);


  // navigation
  pt pos;
  double angle;
  int ofsx;
  int ofsy;
  unsigned char speed;
  // virtual sensors
  pt *scen;
  pt *slef;
  pt *srig;
  int sval_c;
  int sval_l;
  int sval_r;
  int range;
  // fiducial detection
  int patt_id;
  int k_index;
  // controller information
  int state;
  bool thr_l;
  bool thr_c;
  bool thr_r;
  // logger
  FILE *pfile;

private:
  void setval(unsigned char *dataPtr,int image_width, int image_height, int x, int y, unsigned char *value);
};

static int cm2px(int value){
  return (int) value*7.4;
}
static int px2cm(int value){
  return (int) ((float)value/7.4);
}
#endif
