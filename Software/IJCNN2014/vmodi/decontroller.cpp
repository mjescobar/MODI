#include "decontroller.h"


decontroller::decontroller()
{
    cs = ls =  rs = 0;
    control =  (unsigned char *) malloc(sizeof(unsigned char)*4);
    prevcontrol[0] = prevcontrol[1] = 0;
    control[0] = 0x02;
    minNoObj = cm2px(14); // cm -> min. distance to avoid collision on the front
    minObjLat = cm2px(14); //  min. distance to avoid collision on the side
    critObjLat = cm2px(7); //  critical minimum distance to an object
    isBblocked = false;
    xvec = new std::vector<int>();
    xvec->push_back(1);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    cState = 1;
    alpha = 0.6;
}

decontroller::decontroller(int IminNoObj, int IminObjLat, int IcritObjLat, float ialpha)
{
    cs = ls = rs = 0;
    control =  (unsigned char *) malloc(sizeof(unsigned char)*4);
    prevcontrol[0] = prevcontrol[1] = 0;
    control[0] = 0x02;
    minNoObj = IminNoObj; // cm -> min. distance to avoid collision on the front
    minObjLat = IminObjLat; //  min. distance to avoid collision on the side
    critObjLat = IcritObjLat; //  critical minimum distance to an object
    isBblocked = false;
    xvec = new std::vector<int>();
    xvec->push_back(1);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    xvec->push_back(0);
    cState = 1;
    alpha = ialpha;
}

void decontroller::attachTo(modi *irob)
{
    robot = irob;
}

void decontroller::blockBeta(bool state)
{
    isBblocked = state;
}

void decontroller::attachBeta(std::vector<int> *vbeta)
{
    beta = vbeta;
}
unsigned char* decontroller::getActuation()
{
    return control;
}

void decontroller::update()
{
    // retrieve current values of the robot's sensors
    cs = robot->sval_c;
    ls = robot->sval_l;
    rs = robot->sval_r;

    // read the pre-set speed of the motors
    unsigned char speed = robot->speed;

    // temporary variables
    int factor;
    // threat detection (at the end of the updating process, these values
    // are saved into X vector).
    bool thr_c = (cs < critObjLat/2)? true:false;	//center
    bool thr_l = (ls < critObjLat)? true:false; 		//left
    bool thr_r = (rs < critObjLat)? true:false;		//right

    // only if there's no threats detected, the state bias can be added.
    if((!thr_l)&&(!thr_c)&&(!thr_r)&&(!isBblocked)&&(cState<4)) cState = cState + (*beta)[0];

    // some prints about the current state of the controller
    if(!isBblocked) printf(" state: %d, speed: %d ,(%d%d%d), modifiers [%d,%d,%d] ", cState, speed, thr_l, thr_c, thr_r,(*beta)[0],(*beta)[1],(*beta)[2]);
    else printf(" state: %d, speed: %d ,(%d%d%d)", cState, speed, thr_l, thr_c, thr_r);

    // main finite-state-machine
    switch(cState)
    {
    case 1: //forward
        factor = (unsigned char)round((cs-minNoObj)/5);
        control[1] = 0x03;
        control[2] = speed+factor;
        control[3] = speed+factor;
        if(((cs<minNoObj)&&(rs<ls)) || rs<(minObjLat)) cState = 2; //go left
        if(((cs<minNoObj)&&(ls<rs)) || ls<(minObjLat)) cState = 3; //go right
        break;
    case 2: // left
        control[1] = 0x03;
        control[2] = speed;
        control[3] = (!isBblocked)? (unsigned char) (*beta)[1] : 0x00;
        if(rs < critObjLat)  cState = 4; // critical turn left
        if(cs > minNoObj+cm2px(1)) cState = 1; // back to forward
        break;
    case 3: // right
        control[1] = 0x03;
        control[2] = (!isBblocked)? (unsigned char) (*beta)[2] : 0x00;
        control[3] = speed;
        if(ls < critObjLat)  cState = 5; // critical turn right
        if(cs > minNoObj+cm2px(1)) cState = 1; // back to forward
        break;
    case 4: // critical left
        control[1] = 0x01;
        control[2] = speed;
        control[3] = speed;
        if(rs > critObjLat+cm2px(3))  cState = 2; // normal turn left
        if(cs > minNoObj+cm2px(3)) cState = 1; // back to forward
        break;
    case 5: // critical right
        control[1] = 0x02;
        control[2] = speed;
        control[3] = speed;
        if(ls > critObjLat+cm2px(3))  cState = 3; // normal turn right
        if(cs > minNoObj+cm2px(3)) cState = 1; // back to forward
        break;
    case 6: // stop
        control[1] = 0x02;
        control[2] = 0x00;
        control[3] = 0x00;
    }

    // X vector updating
    (*xvec)[0] = cState;
    (*xvec)[1] = thr_c;
    (*xvec)[2] = thr_l;
    (*xvec)[3] = thr_r;
    (*xvec)[4] = px2cm(cs);
    (*xvec)[5] = px2cm(ls);
    (*xvec)[6] = px2cm(rs);

    prevcontrol[0] = control[2] = (unsigned char) prevcontrol[0]*(1-alpha)+control[2]*alpha;
    prevcontrol[1] = control[3] = (unsigned char) prevcontrol[1]*(1-alpha)+control[3]*alpha;

}
