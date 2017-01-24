#include "segment.h"
#include "vehicle.h"

Segment::Segment():next_(NULL),corentry(NULL) {
    empty.setv(0);
    empty.setid(-1);
    car_num = 0;
}
Segment::~Segment() {}

void Segment::printInfo()const{
}

void Segment::setInit(double _seg_start, double _seg_end, int _lane_num)
{
    seg_start = _seg_start;
    seg_end = _seg_end;
    seg_dist = seg_end - seg_start;
    empty.setpos(seg_start);
    lane_num = _lane_num;
    upbound = -1;
    lowbound = 0xffffff;
}

Vehicle* Segment::search_forsd(Vehicle *cur,double pos,int lane){
    int i;

    VERBOSE(2,("[MT][SD]search_forsd(pos=%.2lf,lane=%d,ss=%.2f)\n",pos,lane,seg_start));

    if(lane >= lane_num) return NULL;
    example.pos = pos+1;
    set<IVehic>::iterator it = carseq[lane].lower_bound(example);
    if(it == carseq[lane].end())
        return NULL;
    return it->addr;
}

Vehicle* Segment::search(Vehicle *cur,double pos,int lane,bool flag){
    
    VERBOSE(2,("[MT]search(pos=%.2lf,lane=%d,ss=%.2f)\n",pos,lane,seg_start));
    if(flag){ // search car with ahead : larger than
        if(lane >= lane_num) return &empty;
        example.pos = pos+1;
        set<IVehic>::iterator it = carseq[lane].lower_bound(example);
        if(it != carseq[lane].end())return it->addr;
        if(next_ != NULL){
            Vehicle *res = next_->search(cur,pos,lane,flag);
            return res;
        }
        return NULL;
    }else{ // search car behind
        example.pos = pos-1;
        set<IVehic>::iterator it = carseq[lane].lower_bound(example);

        if(it==carseq[lane].begin() || --it == carseq[lane].begin() ){
            return NULL;
        }else return it->addr;
    }
}

bool Segment::remove(int id,double lastpos,int orilane){
    example.pos = lastpos;
    carseq[orilane].erase(example);
    return true;
}

////// @param pos : current position
////// @param lastpos : this data needs to be removed 
int Segment::modify(Vehicle *cur,int id,double pos,double lastpos,int curlane,int orilane)
{
    bool res = remove(id,lastpos,orilane);
    insert(cur,id,pos,curlane);
    return res;
}

////// Return the bin index
int Segment::insert(Vehicle* cur,int id,double pos,int curlane)
{
    IVehic tmp;
    tmp.id = id;
    tmp.pos = pos;
    tmp.v = cur->getv();
    tmp.addr = cur;
    tmp.isd = cur->ISD();
    carseq[curlane].insert(tmp);
    return 0;
}
