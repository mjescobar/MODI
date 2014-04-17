INC_DIR= ../include
LIB_DIR= ../lib
BIN_DIR= ../bin

LDFLAG=-pthread -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lxml2 -lglib-2.0 -L/usr/X11R6/lib -L/usr/local/lib -L$(LIB_DIR)
LIBS= -lxbee -lARgsub -lARvideo -lAR -lpthread -lglut -lGLU -lGL -lXi -lX11 -lm -pthread -lgstreamer-0.10 -lgobject-2.0 -lgmodule-2.0 -lgthread-2.0 -lrt -lxml2 -lglib-2.0
CFLAG= -O -pthread -I/usr/include/gstreamer-0.10 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -I/usr/include/libxml2 -I/usr/X11R6/include -g -I$(INC_DIR)

OBJS = modi.o bmap.o
HEADDERS = vmodi.h modi.h bmap.h

all: $(BIN_DIR)/vmodi

$(BIN_DIR)/vmodi: vmodi.o $(OBJS)
	cc -o $(BIN_DIR)/vmodi vmodi.o $(OBJS) $(LDFLAG) $(LIBS)

vmodi.o: vmodi.cpp $(HEADDERS)
	cc -c $(CFLAG) vmodi.cpp
	
bmap.o: bmap.cpp $(HEADERS)
	cc -c $(CFLAG) bmap.cpp
	
modi.o: modi.cpp $(HEADDERS)
	cc -c $(CFLAG) modi.cpp

clean:
	rm -f *.o
	rm -f $(BIN_DIR)/vmodi
