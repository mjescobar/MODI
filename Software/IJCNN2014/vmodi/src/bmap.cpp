#include "bmap.h"
#include <stdlib.h>
#include <stdio.h>

bmap::bmap(){
  width = 0;
  height = 0;
  size = 0;
  framesize = 0;
  image = NULL;
  channels = 3;
  offx = 0;
  offy = 0;
}
bmap::bmap(const char* filename,int inoffx, int inoffy, int framewidth, int frameheight){
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header
    // extract image height and width from header
    width = *(int*)&info[18];
    height = *(int*)&info[22];

	//printf("width: %d, height: %d\n",width,height);
    size = 3 * width * height;
    framesize = 3 * framewidth * frameheight;
	// allocate 3 bytes per pixel
    image = (unsigned char*) malloc(sizeof(unsigned char)*size);
    fread(image, sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);
    offx = inoffx;
    offy = inoffy;
}

void bmap::load(const char* filename){
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header
    // extract image height and width from header
    width = *(int*)&info[18];
    height = *(int*)&info[22];

	//printf("width: %d, height: %d\n",width,height);
    size = 3 * width * height;

	// allocate 3 bytes per pixel
    image = (unsigned char*) malloc(sizeof(unsigned char)*size);
    fread(image, sizeof(unsigned char), size, f); // read the rest of the data at once
    fclose(f);
}

void bmap::set(int x, int y, unsigned char *value){
    if(x>0&&y>0&&x<width&&y<height){
	image[width*channels*y+x*channels] = value[0];
	image[width*channels*y+x*channels+1] = value[1];
	image[width*channels*y+x*channels+2] = value[2];
    }
}

unsigned char bmap::get(int x, int y, int ch){
  return image[width*3*(y-offy) + 3*(x-offx)+ch];
}

void bmap::overlayInto( unsigned char * dataPtr, int image_width){
   for(int i = 0; i < height; i++){
      for(int j = 0; j < width; j++){
	if(image[width*3*i+3*j+2]) dataPtr[image_width*3*(offy+i)+(offx+j)*3] =  image[width*3*i+3*j+2];
	if(image[width*3*i+3*j+1]) dataPtr[image_width*3*(offy+i)+(offx+j)*3+1] = image[width*3*i+3*j+1];
	if(image[width*3*i+3*j]) dataPtr[image_width*3*(offy+i)+(offx+j)*3+2] = image[width*3*i+3*j];
      }
  }
}

