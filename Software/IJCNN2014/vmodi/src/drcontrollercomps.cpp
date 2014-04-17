#include "drcontrollercomps.h"

drMemory::drMemory()
{
    memory = std::map<std::string,float>();
    corners = std::vector<pt>();
    for(int i = 0; i < 20; i++) window.push_back(0);
}
void drMemory::setValue(std::string key, float value)
{
    memory[key] = value;
}
float drMemory::getValue(std::string key)
{
    return memory[key];
}



drControl::drControl()
{
}
drControl::drControl(int ithreshold)
{
    radius = cm2px(16);
    threshold = px2cm(ithreshold);
    end_cond = prev = curr = 0;
}
void drControl::attachX(std::vector<int> *ixvec)
{
    xvec = ixvec;
}
void drControl::attachBeta(std::vector<int> *ibeta)
{
    beta = ibeta;
}
void drControl::attachMemory(drMemory *imem)
{
    mem = imem;
}
void drControl::linkPos(pt *posi)
{
    current_position = posi;
}
int drControl::checkCorner(pt position)
{
    if(mem->corners.size()<2) return -1;
    for(int i = 0; i < mem->corners.size()-1; i++)
    {
        int dx = mem->corners[i].x-position.x;
        dx = (dx>0)? dx: -dx;
        int dy = mem->corners[i].y-position.y;
        dy = (dy>0)? dy: -dy;
        if((dx<radius)&&(dy<radius)) return i;
    }
    return -1;
}
void drControl::doControl()
{
    int cs = (*xvec)[4];
    int ls = (*xvec)[5];
    int rs = (*xvec)[6];
    int fcd = (int) mem->getValue("fcd");
    int hold_state = (int) mem->getValue("hold");
    int rob_speed = (int) mem->getValue("speed");
    int state = (*xvec)[0];
    int statebias, mlbias, mrbias;
    statebias = mlbias = mrbias = 0;

    if(state==6)
    {
        printf("check_corner= %d ",checkCorner(mem->corners.back()));
        for(int i = 0; i < mem->corners.size(); i++)
            printf("{%d,%d} ", mem->corners[i].x,mem->corners[i].y);
        printf("\n");
        return;
    }

    mem->window.pop_front();
    if(cs<12&&rs<12) mem->window.push_back((50-cs)+(50-rs));
    else mem->window.push_back(0);
    int counter = 0;
    for(std::list<int>::iterator it = mem->window.begin(); it != mem->window.end(); ++it) counter += (*it);
    printf("cont %3d, ",counter);

    curr = (counter>=500)? 1:0;

    if(!prev&&curr)
    {
        pt point;
        point.x = (*current_position).x;
        point.y = (*current_position).y;
        mem->corners.push_back(point);
        mem->setValue("cnt",1);
    }
    if(prev&&curr)
    {
        mem->corners.back().x += (*current_position).x;
        mem->corners.back().y += (*current_position).y;
        mem->setValue("cnt",mem->getValue("cnt")+1);
    }
    if(prev&&!curr)
    {
        int count = mem->getValue("cnt");
        mem->corners.back().x = mem->corners.back().x/count;
        mem->corners.back().y = mem->corners.back().y/count;
        mem->setValue("cnt",0);
        int chAlr = checkCorner(mem->corners.back());
        if(chAlr>=0)
        {
            if(chAlr<2&&mem->corners.size()>3) end_cond = 1;
            if(chAlr<2&&mem->corners.size()<3)
            {
                fcd = 0;
                statebias = 1 - state;
                return;
            }
            if(chAlr>1) mem->corners.pop_back();
            printf("chAlr= %d ",chAlr);
        }
        else printf(" {%d,%d} ", mem->corners.back().x,mem->corners.back().y);

    }
    //printf("corner 100\% ");

    prev = curr;
    //printf("next:%d, ",state);
    // if is the first collision but with the left sensor,
    // change to hold state and turn to the right until the
    // right sensor make contact with the wall
    if(((state > 1) && (!fcd))&&(ls < rs)) hold_state = 1;
    if(hold_state) printf("hold state activated\n");
    // FIRST COLLISION DETECTOR
    // if stated changed (due a wall detection) and
    // fcd was previously zero, is the first collision!
    fcd = (state > 1) + fcd;
    if(!fcd) printf("not collisioned yet ");
    // if is not collided, the state remains unchanged.
    if(!fcd)
    {
        statebias = 0;
    }
    else
    {
        float hat = 0.95;
        int errorpos = ((rs-threshold)>0)? rs-threshold : threshold - rs;
        float max_error = 1.25*((float)threshold);
        float k = ((float)rob_speed)/max_error;
        int pterm = (int) (k*errorpos);
        pterm = (pterm>(int)(hat*rob_speed))? hat*rob_speed:pterm;

        if(rs<threshold)//||cs>threshold)
        {
            statebias = 2 - state;
            /// proportional controller
            mlbias = rob_speed - pterm;
        }
        else
        {
            statebias = 3 - state;
            /// proportional controller
            mrbias = rob_speed - pterm;
        }
        printf("c2err:%2d, rob_speed= %2d, pterm=%2d, bias: [%2d,%2d] ",errorpos,rob_speed,pterm,mlbias,mrbias);
    }
    if(hold_state)
    {
        // if right sensor finally is closer to a wall than left
        // sensor, hold state is deactivated. Otherwise, the robot
        // starts hard right turn. (state 5)
        if(rs<ls)
        {
            hold_state = 0;
            statebias = 3-state;
            mrbias = rob_speed;
        }
        else statebias = 5-state;
    }

    statebias = end_cond? 6-state : statebias;
    // beta vector update
    (*beta)[0] = statebias;
    (*beta)[1] = mlbias;
    (*beta)[2] = mrbias;
    // memory update
    mem->setValue("fcd",fcd);
    mem->setValue("hold",hold_state);

}


drControl2::drControl2()
{
}
drControl2::drControl2(int ithreshold)
{
    radius = cm2px(15);
    threshold = px2cm(ithreshold);
    end_cond = prev = curr = 0;
}
void drControl2::attachX(std::vector<int> *ixvec)
{
    xvec = ixvec;
}
void drControl2::attachBeta(std::vector<int> *ibeta)
{
    beta = ibeta;
}
void drControl2::attachMemory(drMemory *imem)
{
    mem = imem;
}
void drControl2::linkPos(pt *posi)
{
    current_position = posi;
}
int drControl2::checkCorner(pt position)
{
    if(mem->corners.size()<2) return -1;
    for(int i = 0; i < mem->corners.size()-1; i++)
    {
        int dx = mem->corners[i].x-position.x;
        dx = (dx>0)? dx: -dx;
        int dy = mem->corners[i].y-position.y;
        dy = (dy>0)? dy: -dy;
        if((dx<radius)&&(dy<radius)) return i;
    }
    return -1;
}
int drControl2::checkCorner2(pt position, int skipped)
{
    for(int i = 0; i < mem->corners.size(); i++)
    {
        if(i==skipped) continue;
        int dx = mem->corners[i].x-position.x;
        dx = (dx>0)? dx: -dx;
        int dy = mem->corners[i].y-position.y;
        dy = (dy>0)? dy: -dy;
        if((dx<radius)&&(dy<radius)) return i;
    }
    return -1;
}
void drControl2::doControl()
{
    int cs = (*xvec)[4];
    int ls = (*xvec)[5];
    int rs = (*xvec)[6];
    int fcd = (int) mem->getValue("fcd");
    int hold_state = (int) mem->getValue("hold");
    int rob_speed = (int) mem->getValue("speed");
    int state = (*xvec)[0];
    int statebias, mlbias, mrbias;
    statebias = mlbias = mrbias = 0;

    if(state==6)
    {
        printf("check_corner= %d ",checkCorner(mem->corners.back()));
        for(int i = 0; i < mem->corners.size(); i++)
            printf("{%d,%d} ", mem->corners[i].x,mem->corners[i].y);
        printf("\n");
        return;
    }

    mem->window.pop_front();
    if(cs<12&&rs<12) mem->window.push_back((50-cs)+(50-rs));
    else mem->window.push_back(0);
    int counter = 0;
    for(std::list<int>::iterator it = mem->window.begin(); it != mem->window.end(); ++it) counter += (*it);
    printf("cont %3d, ",counter);

    curr = (counter>=500)? 1:0;

    if(!prev&&curr)
    {
        pt point;
        point.x = (*current_position).x;
        point.y = (*current_position).y;
        mem->corners.push_back(point);
        mem->setValue("cnt",1);
    }
    if(prev&&curr)
    {
        mem->corners.back().x += (*current_position).x;
        mem->corners.back().y += (*current_position).y;
        mem->setValue("cnt",mem->getValue("cnt")+1);
    }
    if(prev&&!curr)
    {
        int count = mem->getValue("cnt");
        mem->corners.back().x = mem->corners.back().x/count;
        mem->corners.back().y = mem->corners.back().y/count;
        mem->setValue("cnt",0);
        int chAlr = checkCorner(mem->corners.back());
        if(chAlr>=0)
        {
            if(chAlr<2&&mem->corners.size()>3) end_cond = 1;
            if(chAlr<2&&mem->corners.size()<3)
            {
                fcd = 0;
                statebias = 1 - state;
                return;
            }
            if(chAlr>1) mem->corners.pop_back();
            printf("chAlr= %d ",chAlr);
        }
        else printf(" {%d,%d} ", mem->corners.back().x,mem->corners.back().y);

    }
    //printf("corner 100\% ");

    prev = curr;
    //printf("next:%d, ",state);
    // if is the first collision but with the left sensor,
    // change to hold state and turn to the right until the
    // right sensor make contact with the wall
    if(((state > 1) && (!fcd))&&(ls < rs)) hold_state = 1;
    if(hold_state) printf("hold state activated\n");
    // FIRST COLLISION DETECTOR
    // if stated changed (due a wall detection) and
    // fcd was previously zero, is the first collision!
    fcd = (state > 1) + fcd;
    if(!fcd) printf("not collisioned yet ");
    // if is not collided, the state remains unchanged.
    if(!fcd)
    {
        statebias = 0;
    }
    else
    {
        float hat = 0.95;
        int errorpos = ((rs-threshold)>0)? rs-threshold : threshold - rs;
        float max_error = 1.25*((float)threshold);
        float k = ((float)rob_speed)/max_error;
        int pterm = (int) (k*errorpos);
        pterm = (pterm>(int)(hat*rob_speed))? hat*rob_speed:pterm;

        if(rs<threshold)//||cs>threshold)
        {
            statebias = 2 - state;
            /// proportional controller
            mlbias = rob_speed - pterm;
        }
        else
        {
            statebias = 3 - state;
            /// proportional controller
            mrbias = rob_speed - pterm;
        }
        printf("c2err:%2d, rob_speed= %2d, pterm=%2d, bias: [%2d,%2d] ",errorpos,rob_speed,pterm,mlbias,mrbias);
    }
    if(hold_state)
    {
        // if right sensor finally is closer to a wall than left
        // sensor, hold state is deactivated. Otherwise, the robot
        // starts hard right turn. (state 5)
        if(rs<ls)
        {
            hold_state = 0;
            statebias = 3-state;
            mrbias = rob_speed;
        }
        else statebias = 5-state;
    }

    //statebias = end_cond? 6-state : statebias;
    if(end_cond)
    {
        prev = curr = -1;
        mem->setValue("detcorners",0);
        // beta vector nullified: acts like DE alone
        (*beta)[0] = 0;
        (*beta)[1] = 0;
        (*beta)[2] = 0;
    }
    else
    {
        // beta vector update
        (*beta)[0] = statebias;
        (*beta)[1] = mlbias;
        (*beta)[2] = mrbias;
    }
    // memory update
    mem->setValue("fcd",fcd);
    mem->setValue("hold",hold_state);

}
void drControl2::doControl2()
{
    int cs = (*xvec)[4];
    int ls = (*xvec)[5];
    int rs = (*xvec)[6];
    int fcd = (int) mem->getValue("fcd");
    int hold_state = (int) mem->getValue("hold");
    int rob_speed = (int) mem->getValue("speed");
    int state = (*xvec)[0];
    int statebias, mlbias, mrbias;
    end_cond = statebias = mlbias = mrbias = 0;

    pt point = (*current_position); //point.x=(*current_position).x;point.y=(*current_position).y;
    int curr = checkCorner2(point,-1);
    if(curr>=0)
    {
        if(curr<=1&&!hold_state){
            hold_state = 1;
            threshold = threshold + 5;
            radius = radius + cm2px(5);
        }
        if(curr>=2) hold_state = 0;

    }
    prev = curr;
    printf("current=%d, prev=%d, radius:%d, threshold:%d",curr, prev, radius, threshold);
    float hat = 0.95;
    int errorpos = ((rs-threshold)>0)? rs-threshold : threshold - rs;
    float max_error = 1.25*((float)threshold);
    float k = ((float)rob_speed)/max_error;
    int pterm = (int) (k*errorpos);
    pterm = (pterm>(int)(hat*rob_speed))? hat*rob_speed:pterm;

    int motorfact = 20;

    if(rs<threshold)
    {
        statebias = 2 - state;
        /// proportional controller
        mlbias = rob_speed - pterm;
    }
    else if(curr>0&&cs<threshold)
    {
        statebias = 2 - state;
        /// proportional controller
        mlbias = motorfact;
    }
    else
    {
        statebias = 3 - state;
        /// proportional controller
        mrbias = rob_speed - pterm;
    }

    mem->setValue("hold",hold_state);
    // beta vector update
    statebias = (threshold>50)? 6-state : statebias;
    (*beta)[0] = statebias;
    (*beta)[1] = mlbias;
    (*beta)[2] = mrbias;

}

