#include "drcontroller2.h"

drcontroller2::drcontroller2()
{
    // beta initialization
    beta = new std::vector<int>();
    beta->push_back(0);
    beta->push_back(0);
    beta->push_back(0);


    // threshold, config. parameter
    threshold = cm2px(13);

    // creation of the memory module
    mem = new drMemory();
    // some values are saved in memory
    mem->setValue("fcd",0);
    mem->setValue("hold",0);
    mem->setValue("speed",50);

    // creation of the control module
    control = new drControl2(threshold);
    // some conections are created.
    //This is the work of the interoceptive system.
    control->attachBeta(beta);
    control->attachX(xvec);
    control->attachMemory(mem);
    control->linkPos(current_position);
}

drcontroller2::drcontroller2(int ithreshold,int speed, pt *posi)
{
    // beta initialization
    beta = new std::vector<int>();
    beta->push_back(0);
    beta->push_back(0);
    beta->push_back(0);

    // threshold, config. parameter
    threshold = ithreshold;
    // current position linker
    current_position = posi;

    // creation of the memory module
    mem = new drMemory();
    // some values are saved in memory
    mem->setValue("fcd",0);
    mem->setValue("hold",0);
    mem->setValue("speed",speed);
    mem->setValue("detcorners",1);

    // creation of the control module
    control = new drControl2(threshold);
    // some conections are created.
    //This is the work of the interoceptive system.
    control->attachBeta(beta);
    control->attachX(xvec);
    control->attachMemory(mem);
    control->linkPos(posi);
}

void drcontroller2::attachX(std::vector<int> *ixvec)
{
    xvec = ixvec;
    control->attachX(ixvec);
}

void drcontroller2::update()
{
    int cdetected = (int) mem->getValue("detcorners");
    // the updating of the controller only consists on the updating of the controller.
    // All the signals are already moved to the other modules thanks to the connections
    // created in the constructor of the controller

    // DETECTION OF ALL THE CORNERS
    if(cdetected) control->doControl();
    else control->doControl2();



}
void drcontroller2::linkPos(pt *posi)
{
    current_position = posi;
}
