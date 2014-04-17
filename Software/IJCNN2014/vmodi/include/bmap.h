#ifndef _BMAP_H_
#define _BMAP_H_
#include <string>

class bmap{
public:
  bmap();
  bmap(const char* filename,int inoffx, int inoffy, int framewidth, int frameheight);
  void load(const char* filename);
  void set(int x, int y, unsigned char * value);
  unsigned char get(int x, int y,int ch);

  void overlayInto( unsigned char * dataPtr, int image_width);

  int width;
  int height;
  int size;
  int framesize;

  unsigned char * image;
  int channels;
  int offx;
  int offy;
};
#endif
