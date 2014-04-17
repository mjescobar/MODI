#include "vmodi.h"

modi m1;
bmap b1;
int fcounter = 0;

// show the received data in the xbee module

void myCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data)
{
    if ((*pkt)->dataLen == 0)
    {
        printf("too short...\n");
        return;
    }
    printf("rx: [%s]\n", ((*pkt)->data));
}


int main(int argc, char **argv)
{
    printf("starting!\n");
    int dummy = 1;
    glutInit(&dummy,NULL);
    if(argc>2) init(argv[3], argv[2]);
    else init((char *) "scenarios/env_1.bmp",NULL);
    if(argc>1)  fcounter = atoi(argv[1]);
    else fcounter = -1;
    arVideoCapStart();
    argMainLoop( NULL, keyEvent, mainLoop );
    if ((ret = xbee_conEnd(con)) != XBEE_ENONE) xbee_errorToStr(ret);
    xbee_shutdown(xbee);
    return (0);
}

static void   keyEvent( unsigned char key, int x, int y)
{
    /* quit if the ESC key is pressed */
    if( key == 'w')
    {
        m1.speed = (m1.speed+10) < 0xff ? m1.speed + 10 : 0xff;
    }
    if( key == 's')
    {
        m1.speed = (m1.speed-10) > 0x00 ? m1.speed - 10 : 0x00;
    }
    if( key == 'z')
    {
        m1.state = 6;
    }
    if( key == 0x1b )
    {
        printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
        cleanup();
        exit(0);
    }
}
/* main loop */
static void mainLoop(void)
{
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j;//, k1, k2;
    double 	    patt_trans1[3][4];


    /* grab a video frame */
    if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL )
    {
        arUtilSleep(2);
        return;
    }
    if( count == 0 ) arUtilTimerReset();
    count++;

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 )
    {
        cleanup();
        exit(0);
    }
    // virtual sensors section: updating and drawing
    m1.updateSenPix();
    m1.updateSDist(b1);
    // a copy of the frame is used to avoid flickering of the overlayed image
    memcpy(pframe, dataPtr,b1.framesize);
    m1.drawSensorsInto(pframe,image_width,image_height);
    b1.overlayInto(pframe,image_width);

    argDrawMode2D();
    argDispImage( pframe, 0,0 );
    arVideoCapNext();

    /* check for object visibility */
    m1.k_index = -1;
    for( j = 0; j < marker_num; j++ )
    {
        if( m1.patt_id == marker_info[j].id )	if( m1.k_index == -1 ) m1.k_index = j;
    }
    if( m1.k_index == -1 /*agregar los demás con AND*/)
    {
        argSwapBuffers();
        return;
    }
    /* get the transformation between the marker and the real camera */
    if(m1.isVisible()) arGetTransMat(&marker_info[m1.k_index], patt_center, patt_width, patt_trans1);
    /* updating position and angle */
    int xpos = (int) round(marker_info[m1.k_index].pos[0]);
    int ypos = (int) round(marker_info[m1.k_index].pos[1]);
    m1.setpos(xpos,ypos);
    ///-----------------------
    float fading = 0.10;
    ///-----------------------
    m1.angle = fading*m1.angle + (1-fading)*object_angle(patt_trans1,marker_info[m1.k_index].pos);
    cont++;
    /* every 4 frames, a control signal is sent to the robot. or instead, the value of the virtual sensors */
    if(cont==3)
    {

        m1.report(true);

        deCont.update();

        if(controller_version2)
            drContV2.update();
        else
            drCont.update();

        unsigned char * actuation = deCont.getActuation();
        xbee_connTx(con, NULL, actuation ,4);

        cont = 0;
    }
    argSwapBuffers();
    printf(" fc%4d ",fcounter);
    if(fcounter>0) fcounter--;
    if(fcounter==0)
    {
        unsigned char stop[] = {0x01,0x05,0x00,0x00};
        xbee_connTx(con, NULL, stop ,4);
        cleanup();
        exit(0);
    }
}

static void init( char * bitmapname, char *outfilename )
{

    // setup of ARToolkit
    ARParam  wparam;
    if( arVideoOpen( vconf ) < 0 ) exit(0);
    if( arVideoInqSize(&xsize, &ysize) < 0 ) exit(0);
    printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);
    if( arParamLoad(cparam_name, 1, &wparam) < 0 )
    {
        printf("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &wparam, xsize, ysize, &cparam );
    arInitCparam( &cparam );
    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );
    // setup of Modi and virtual representation
    m1 = modi(0,30,370); // 50 cm sensor range
    m1.patt_id = arLoadPatt(patt_name1);
    // change here if you want to change the position of the scenario in the screen
    b1 = bmap(bitmapname,500,80,image_width,image_height);
    pframe = (unsigned char *) malloc(sizeof(unsigned char)*b1.framesize);


    if(m1.patt_id < 0/* || patt_id2 < 0*/ )
    {
        printf("pattern load error !!\n");
        exit(0);
    }
    if(outfilename!=NULL)   m1.initLogger(outfilename);
    else m1.initLogger((char *) "movements.txt");
    /* open the graphics window */

    argInit( &cparam, 1.0, 0, 0, 0, 0 );

    cont = 0;

    // generation xbee object
    if ((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB0", 57600)) != XBEE_ENONE) xbee_errorToStr(ret);
    memset(&address, 0, sizeof(address));
    address.addr64_enabled = 1;
    address.addr64[0] = 0x00;
    address.addr64[1] = 0x13;
    address.addr64[2] = 0xA2;
    address.addr64[3] = 0x00;
    address.addr64[4] = 0x40;
    address.addr64[5] = 0x91;
    address.addr64[6] = 0xA1;
    address.addr64[7] = 0x87;
    if ((ret = xbee_conNew(xbee, &con, "Data",&address)) != XBEE_ENONE) xbee_errorToStr(ret);
    if ((ret = xbee_conCallbackSet(con, myCB, NULL)) != XBEE_ENONE) xbee_errorToStr(ret);

    unsigned char command1[4] = {0x02,0x03,0x64,0x64};
    //unsigned char command2[4] = {0x02,0x01,0x00,0x32};
    //unsigned char command3[4] = {0x02,0x02,0x32,0x00};
    unsigned char command4[4] = {0x02,0x01,0x00,0x00};
    ret = xbee_connTx(con, NULL, command1,4);
    usleep(100000);
    ret = xbee_connTx(con, NULL, command4,4);


    /// pretty important!!!
    controller_version2 = 1;
    /// -------------------

    deCont = decontroller(cm2px(14) /*minNoObj*/,
                          cm2px(17) /*minNoObjLat*/,
                          cm2px(10) /*critNoObj*/,
                          0.8 /*alpha*/);
    drCont = drcontroller(cm2px(18)/*distance to the wall*/,
                          m1.speed /*robot speed*/,
                          &(m1.pos)/*robot position*/);

    drContV2 = drcontroller2(cm2px(18) /*distance to the wall*/,
                             m1.speed /*robot speed*/,
                             &(m1.pos)/*robot position*/);


    deCont.attachTo(&m1);

    if(controller_version2)
    {
        deCont.attachBeta(drContV2.beta);
        drContV2.attachX(deCont.xvec);
    }
    else
    {
        deCont.attachBeta(drCont.beta);
        drCont.attachX(deCont.xvec);
    }
    //deCont.blockBeta(true);
}
/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}

static void draw( double (*patt_trans)[4], int r,int g, int b )
{
    float rc = r/255;
    float gc = g/255;
    float bc = b/255;
    double    gl_para[16];
    GLfloat   mat_ambient[]     = {rc, gc, bc, 1.0};
    GLfloat   mat_flash[]       = {rc, gc, bc, 1.0};

    GLfloat   mat_flash_shiny[] = {50.0};
    GLfloat   light_position[]  = {100.0,-200.0,200.0,0.0};
    GLfloat   ambi[]            = {0.1, 0.1, 0.1, 0.1};
    GLfloat   lightZeroColor[]  = {0.9, 0.9, 0.9, 0.1};

    argDrawMode3D();
    argDraw3dCamera( 0, 0 );
    glClearDepth( 1.0 );
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* load the camera transformation matrix */
    argConvGlpara(patt_trans, gl_para);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixd( gl_para );

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightZeroColor);
    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_flash);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_flash_shiny);
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMatrixMode(GL_MODELVIEW);
    glTranslatef( 0.0, 0.0, 25.0 );
    glutSolidCube(50.0);
    glDisable( GL_LIGHTING );

    glDisable( GL_DEPTH_TEST );
}

