#include "drcontroller.h"

drcontroller::drcontroller()
{
    // beta initialization
    beta = new std::vector<int>();
    beta->push_back(0);
    beta->push_back(0);
    beta->push_back(0);


    // threshold, config. parameter
    threshold = cm2px(17);

    // creation of the memory module
    mem = new drMemory();
    // some values are saved in memory
    mem->setValue("fcd",0);
    mem->setValue("hold",0);
    mem->setValue("speed",50);

    // creation of the control module
    control = new drControl(threshold);
    // some conections are created.
    //This is the work of the interoceptive system.
    control->attachBeta(beta);
    control->attachX(xvec);
    control->attachMemory(mem);
    control->linkPos(current_position);
}

drcontroller::drcontroller(int ithreshold,int speed, pt *posi)
{
    // beta initialization
    beta = new std::vector<int>();
    beta->push_back(0);
    beta->push_back(0);
    beta->push_back(0);

    // threshold, config. parameter
    threshold = ithreshold;
    current_position = posi;

    // creation of the memory module
    mem = new drMemory();
    // some values are saved in memory
    mem->setValue("fcd",0);
    mem->setValue("hold",0);
    mem->setValue("speed",speed);

    // creation of the control module
    control = new drControl(threshold);
    // some conections are created.
    //This is the work of the interoceptive system.
    control->attachBeta(beta);
    control->attachX(xvec);
    control->attachMemory(mem);
    control->linkPos(posi);
}

void drcontroller::attachX(std::vector<int> *ixvec)
{
    xvec = ixvec;
    control->attachX(ixvec);
}

void drcontroller::update()
{
    // the updating of the controller only consists on the updating of the controller.
    // All the signals are already moved to the other modules thanks to the connections
    // created in the constructor of the controller
    control->doControl();
}
void drcontroller::linkPos(pt *posi)
{
    current_position = posi;
}
