#include "vehicle.h"
#include "segment.h"

Vehicle FAREST,NEAREST;
static int convoy_count=0;

Vehicle& Vehicle::operator = (const Vehicle& fr)
{
    FAREST.setInit(&FAREST,-100,false,120,0xfffff,0,0,0,0,0,0,0);
    NEAREST.setInit(&NEAREST,-200,false,0,0,0,0,0,0,0,0,0);
    headaddr = fr.headaddr;
    self = fr.self;
    flw_list.clear();
    flw_list = fr.flw_list;
    ifhead = fr.ifhead;
    ifconvoy = fr.ifconvoy;
    ifupdacc = fr.ifupdacc;
    convoy_id = fr.convoy_id;
    id_ = fr.id_;
    isd = fr.isd;
    lastpos_ = fr.lastpos_;
    curseg_ = fr.curseg_;
    exit_ = fr.exit_;
    v_ = fr.v_;
    a_ = fr.a_;
    pos_ = fr.pos_;
    lane_ = fr.lane_;
    orilane_ = fr.orilane_;
    entertime_ = fr.entertime_;
    len_ = fr.len_;
    vm_ = fr.vm_;
    am_ = fr.am_;
    kc_ = fr.kc_;
    kf_ = fr.kf_;

    return *this;
}

Vehicle::Vehicle(const Vehicle& fr)
{
    FAREST.setInit(&FAREST,-100,false,120,0xffffff,0,0,0,0,0,0,0);
    NEAREST.setInit(&NEAREST,-200,false,0,0,0,0,0,0,0,0,0);
    id_ = fr.id_;
    self = fr.self;
    convoy_id = fr.convoy_id;
    headaddr = fr.headaddr;
    flw_list.clear();
    flw_list = fr.flw_list;
    ifhead = fr.ifhead;
    ifconvoy = fr.ifconvoy;
    ifupdacc = fr.ifupdacc;
    isd = fr.isd;
    lastpos_ = fr.lastpos_;
    curseg_ = fr.curseg_;
    exit_ = fr.exit_;
    v_ = fr.v_;
    orilane_ = fr.orilane_;
    a_ = fr.a_;
    pos_ = fr.pos_;
    lane_ = fr.lane_;
    entertime_ = fr.entertime_;
    len_ = fr.len_;
    vm_ = fr.vm_;
    am_ = fr.am_;
    kc_ = fr.kc_;
    kf_ = fr.kf_;
}

void Vehicle::resetSelf(Vehicle *s)
{
    VERBOSE(2,("[VH]Reset-self (%d,head=%d,convoy=%d)\n",id_,ifhead,ifconvoy))
    Vehicle *old=self;
    self=s;
    if(ifhead) upd_flw_list();
    else if(ifconvoy)headaddr->upd_headarr(old,s);

    if( curseg_->modify(self,id_,pos_,lastpos_,lane_,orilane_) != 1){
        //printf("Reset failed!\n");
        //cin.get();
    }
}

bool Vehicle::clearit(){
    //curseg_->//printInfo();
    VERBOSE(2,("[VH]clear(%d)\n",id_));
    if(curseg_ == NULL) return false;
    //curseg_->modify(self,id_,pos_,pos_,lane_,lane_);
    ifhead=ifconvoy=ifupdacc=false;
    headaddr = NULL;
    isd=false;
    convoy_id=-1;
    id_=-1;
    flw_list.clear();
    entertime_ = 0;
    lastpos_ = 0;
    lane_ = orilane_ = 0;
    self=NULL;
    curseg_=NULL;
    return true;
}

Vehicle::Vehicle():self(NULL),curseg_(NULL),exit_(false),pos_(0),v_(0),a_(0),id_(0),len_(0),
vm_(0),am_(0),kc_(0),kf_(0) {
    ifhead=ifconvoy=ifupdacc=false;
    headaddr = NULL;
    isd=false;
    convoy_id=-1;
    entertime_ = 0;
    lastpos_ = 0;
    lane_ = orilane_ = 0;
    FAREST.setInit(&FAREST,-100,false,120,0xfffff,0,0,0,0,0,0,0);
    NEAREST.setInit(&NEAREST,-200,false,0,0,0,0,0,0,0,0,0);
}

Vehicle::~Vehicle()
{
    
}
void Vehicle::setSeg(Segment *seg){curseg_ = seg;}

void Vehicle::printInfo(){
    //printf("Vehicle No.%d:\n",id_);
    //printf("Pos:%.2f\tVec:%.2f\tAcc:%.2f\n",pos_,v_,a_);
}

void Vehicle::setInit(Vehicle *s,int id,bool iselfdrive,double v,double pos,int lane,
		int entertime,double len,double vm,double am,double kc,double kf){
    self = s;
    id_ = id;
    isd = iselfdrive;
    v_ = v;
    pos_ = pos;
    lane_ = orilane_ = lane;
    entertime_ = entertime;
    vm_ = vm;
    am_ = am;
    rundist=acumtime=0;
    kc_ = kc;
    kf_ = kf;
    len_ = len;
}

////// Update States
void Vehicle::upd_pos()
{
    if((ifhead!=ifconvoy)&&ifhead){
        //printf("Conflict! %d %d\n",ifhead,ifconvoy);
        ////cin.get();
    }
    if(ifconvoy && headaddr == NULL){
        //printf("Conflict HEAD NULL!(%d)\n",id_);
        //cin.get();
        if(flw_list.size()>0){
            for(int i=0;i<flw_list.size();i++)
                flw_list[i]->reset_convoy();
        }
        reset_convoy();
    }

    ifupdacc = false;

    lastpos_ = pos_;
    if(a_>-59){
        pos_ += v_ + a_ / 2;
        rundist += v_ + a_/2;
        acumtime ++;
    }
}
void Vehicle::upd_vec()
{
    if(a_ < -59)v_=0;
    else v_ += a_;
    if (v_ > vm_)
        v_ = vm_;
    if (v_ < 0)
        v_ = 0;
}

double Vehicle::upd_acc(){
    if(id_<0)return -1;
    if(isd){
        return upd_acc_sd();
    }else
        return upd_acc_hm();
}

double Vehicle::upd_acc_sd()
{
    VERBOSE(2,("[VH][SD]upd_acc_sd(%d)\n",id_));
    restore();

    if(!ifhead && headaddr==self){
        if(flw_list.size()>0)ifhead=true;
        else { 
            ifhead=false;
            reset_convoy();
        }
    }

    if(ifconvoy){
        if(!ifhead){
            if(!headaddr->hasupdacc())headaddr->upd_acc();
            a_ = headaddr->geta();
            ifupdacc = true;
            return -9; /// following mode
        }
    }

    Vehicle *front_car = curseg_->search(self,pos_,lane_,true);
    if(front_car == NULL) 
        front_car  = &FAREST;
    
    double dist = front_car->pos_ - pos_;
    if(dist < -1){
        //printf("DIST underflow!\n");
        front_car = &FAREST;
        dist = 1000;
    }
    //if (dist < len_)
    //    a_ = -60.0;
    if (dist < d_safe_sd())
        a_ = -am_;
    else if (dist > d_safe_sd() + d_dec_sd(front_car->v_))
        a_ = am_;
    else
    {
        if (v_ > front_car->v_)
            a_ = -1;
        else
            a_ = 0;
    }

    if(a_>0 && (v_ >= vm_ || v_ >= SPEED_LIM )) a_ = -2;
    if(a_<0 && v_ < 0.01 ) a_ = 0;
    ifupdacc = true;
    return dist;
}

double Vehicle::upd_acc_hm()
{
    VERBOSE(2,("[VH][HM]upd_acc_hm(%d)\n",id_));
    Vehicle *front_car = curseg_->search(self,pos_,lane_,true);
    if(front_car == NULL) 
        front_car = &FAREST;
    
    double dist = front_car->pos_ - pos_;
    if(dist < -1){
        //printf("DIST underflow!\n");
        front_car = &FAREST;
        ////cin.get();
    }
    if (dist < 2*len_)
        a_ = -60.0;
    else if (dist < d_safe_hm())
        a_ = -am_;
    else if (dist > d_safe_hm() + d_dec_hm(front_car->v_))
        a_ = am_ * kf_;
    else
    {
        if (v_ > front_car->v_)
            a_ = -am_;
        else
            a_ = 0;
    }

    if(a_>0 && (v_ >= vm_ || v_ >= SPEED_LIM )) a_ = -am_*kf_;
    if(a_<0 && v_ < 0.01 ) a_ = 0;
    return dist;
}

bool Vehicle::upd_bin(){
    if(id_<0)return -1;
    VERBOSE(2,("[VH]Upd_bin(%d).\n",id_));
    if(curseg_ == NULL || self == NULL){
        //printf("Unintialized!\n");
        return false;
    }
    curseg_->modify(self,id_,pos_,lastpos_,lane_,orilane_);
    lastpos_ = pos_;
    orilane_ = lane_;
    VERBOSE(2,("[VH]Upd_bin() done.\n"));
    return true;
}

bool Vehicle::alter(){
    if(id_<0)return -1;
    if(isd){
        return alter_sd();
    }else{
        return alter_hm();
    }
}

bool Vehicle::alter_sd(){
    VERBOSE(2,("[VH]Check altering(%d).\n",id_));
    double hc = 1;
    if(ifconvoy){
        if(!ifhead) return false;
        hc += KS_HSD * (1+flw_list.size());
    }

    /// now is the head or is oridinary situation

    Vehicle *front_car = curseg_->search(self,pos_, lane_, true);
    if(front_car == NULL) 
        front_car = &FAREST;
    
    if(front_car->v_ > 0.001){
        if (v_ <= front_car->v_)
            return false;
        double dist = front_car->pos_ - pos_;
        if (dist < d_safe_sd())
            return false;
        else if (dist > d_safe_sd() + d_dec_sd(front_car->v_))
            return false;

        int new_lane;
        if (RAND01 < 0.5)
            new_lane = lane_ - 1;
        else
            new_lane = lane_ + 1;

        if (alter_to_sd(new_lane, front_car->v_,hc))
            return true;
        else
        {
            new_lane = 2 * lane_ - new_lane;
            return alter_to_sd(new_lane, front_car->v_,hc);
        }
    }else{
        //// Are force changing conflict with convoy?
        VERBOSE(2,("[VH]Force change.\n"));
        orilane_ = lane_;
        lane_ = rand()%curseg_->lane_num;
        lastpos_ = pos_;
        upd_bin();
        orilane_ = lane_;
        return true;
    }

}

bool Vehicle::alter_hm() {
    VERBOSE(2,("[VH]Check altering(%d).\n",id_));
    Vehicle *front_car = curseg_->search(self,pos_, lane_, true);
    if(front_car == NULL) 
        front_car = &FAREST;
    
    if(front_car->v_ > 0.001){
        if (v_ <= front_car->v_)
            return false;
        double dist = front_car->pos_ - pos_;
        if (dist < d_safe_hm())
            return false;
        else if (dist > d_safe_hm() + d_dec_hm(front_car->v_))
            return false;

        int new_lane;
        if (RAND01 < 0.5)
            new_lane = lane_ - 1;
        else
            new_lane = lane_ + 1;

        if (alter_to_hm(new_lane, front_car->v_))
            return true;
        else
        {
            new_lane = 2 * lane_ - new_lane;
            return alter_to_hm(new_lane, front_car->v_);
        }
    }else{
        VERBOSE(2,("[VH]Force change.\n"));
        orilane_ = lane_;
        lane_ = rand()%curseg_->lane_num;
        lastpos_ = pos_;
        upd_bin();
        orilane_ = lane_;
        return true;
    }
}

/// After this function , orilane_ == lane_
bool Vehicle::alter_to_hm(int new_lane, double v_front)
{
    if (new_lane < 0 || new_lane >= curseg_->lane_num)
        return false;

    Vehicle *front_car = curseg_->search(self,pos_, new_lane, true);
    if(front_car == NULL) 
        front_car = &FAREST;
    
    if (front_car->v_ <= v_front)
        return false;
    if (front_car->pos_ - pos_ <= d_dec_hm(front_car->v_) + d_safe_hm())
        return false;

    Vehicle *back_car = curseg_->search(self,pos_, new_lane, false);
    if(back_car == NULL)
    	back_car = &NEAREST;
    
    if (pos_ - back_car->pos_ <= back_car->d_dec_hm(v_)  + back_car->d_safe_hm())
        return false;

    double p = 1.0;
    if (v_ - v_front < kc_)
        p = (v_ - v_front) / kc_;

    if (RAND01 < p)
    {
        VERBOSE(2,("[VH]Altered from %d to %d\n",lane_,new_lane));
        orilane_ = lane_;
        lane_ = new_lane;
        lastpos_ = pos_;
        upd_bin();
        orilane_ = lane_;
        return true;
    }
    return false;
}

/// After this function , orilane_ == lane_
bool Vehicle::alter_to_sd(int new_lane, double v_front,double hc)
{
    if (new_lane < 0 || new_lane >= curseg_->lane_num)
        return false;

    Vehicle *front_car = curseg_->search(self,pos_, new_lane, true);
    if(front_car == NULL) 
        front_car = &FAREST;
    
    if (front_car->v_ <= v_front)
        return false;
    if (front_car->pos_ - pos_ <= d_dec_sd(front_car->v_) + d_safe_sd()*hc)
        return false;

    Vehicle *back_car = curseg_->search(self,pos_, new_lane, false);
    if(back_car == NULL)
    	back_car = &NEAREST;
    
    if (pos_ - back_car->pos_ <= back_car->d_dec_sd(v_)  + back_car->d_safe_sd()*hc)
        return false;

    double p = 1.0;
    if (v_ - v_front < kc_)
        p = (v_ - v_front) / kc_;

    bool viable=true;
    for(int i=0;i<flw_list.size();i++){
        if(flw_list[i]->curseg_->getlane() <= new_lane){
            viable=false;
            break;
        }
    }
    if(!viable)return false;

    VERBOSE(2,("[VH]Altered from %d to %d\n",lane_,new_lane));
    orilane_ = lane_;
    lane_ = new_lane;
    lastpos_ = pos_;
    upd_bin();
    if(ifhead){
        VERBOSE(2,("[VH]Lead Change All.\n"));
        Vehicle *cur;
        for(int i=0;i<flw_list.size();i++){
            cur = flw_list[i];
            cur->lane_ = new_lane;
            cur->upd_bin();
            //modify(cur,cur->id_,cur->pos_,cur->pos_,new_lane,cur->orilane_);
        }
    }
    orilane_ = lane_;
    return true;
}

Result Vehicle::detect_cross(int T) {
    Result res;
    res.t=-2;
    if(id_<0)return res;
    res.t=-1;
    if(curseg_ == NULL){
        res.exit = true;
        return res;
    }
    if(curseg_->seg_end > pos_) return res;

    VERBOSE(2,("[VH]Crossing Detected.(%d)\n",id_));
    res.t = acumtime;
    res.v = rundist/acumtime;

    if(exit_ || curseg_->getNext() == NULL){
        res.exit = true;
        if( curseg_->remove(id_,pos_,lane_) != 1){
            //printf("RM Failed!\n");
            //cin.get();
        }
        //// MARKED
        if((ifhead || ifconvoy) && isd) exit_in_convoy();

        VERBOSE(2,("[VH]To exit.\n"));
        return res;
    }
    VERBOSE(2,("[VH]To cross.\n"));
    ASSERT_EQ(orilane_,lane_,"DCLANE")
    ASSERT_EQ(lastpos_,pos_,"DCPOS")
    if( curseg_->remove(id_,pos_,lane_) != 1){}
    curseg_ = curseg_->getNext();
    if(curseg_->lane_num <= lane_){
        //printf("Not having changed lane!\n");
        lane_ = rand()%(curseg_->lane_num);
    }
    entertime_ = T;
    curseg_->insert(self,id_,pos_,lane_);
    orilane_ = lane_;
    return res;
}

void Vehicle::randLeave(){
    if(id_<0)return;
    if(curseg_ == NULL)return;

    if(curseg_->getNext() == NULL) return;
    exit_ = curseg_->getNext()->corentry->randLeave();
}

////// Self-driving cars

bool Vehicle::upd_convoy(){
    if(id_<0)return -1;
    VERBOSE(2,("[VH][SD]upd_convoy(id=%d,ifhead=%d,ifconvoy=%d)\n",id_,ifhead,ifconvoy));
    if(!isd)return false;
    if(ifconvoy && !ifhead) return false;

    Vehicle *front_car = NULL,*anhead = NULL;
    int i,new_lane;
    for(i=0;i<3;i++){
        new_lane = lane_ + DT_ARR[i];
        if(new_lane < 0 || new_lane >= curseg_->getlane())continue;

        front_car = curseg_->search_forsd(self,pos_,new_lane);
        if(front_car == NULL || !front_car->ISD())continue;
        if(front_car == self){
            //printf("Impossible self equal!\n");
            //cin.get();
        }

        if(front_car->getpos() - pos_ > FAR_SD || front_car->getpos() < pos_ - 1 )return false;

        if(!ifconvoy){
            if(!front_car->ifconvoy && !front_car->ifhead)front_car->create_convoy();

            /// headaddr of head car must point to itself
            anhead = front_car->headaddr;
            if(anhead==NULL)continue;
            if(anhead->lane_ >= curseg_->getlane()) {return false;}
            if(anhead->add_enabled(1)){
                VERBOSE(2,("[VH]Added to convoy.Apply(%d)\n",id_));
                headaddr = anhead;
                anhead->add_this(self);
                ifconvoy=true;
                ifhead = false;
                return true;
            }
        }

        if(ifhead){
            if(front_car->ifconvoy){
                anhead = front_car->headaddr;
                if(anhead == self)continue;
                if(!anhead->add_enabled(flw_list.size()+1))continue;
                ifhead = false;
                ifconvoy = true;
                convoy_id = -1;
                anhead->merge_to(self,&flw_list);
                flw_list.clear();
            }
        }
    }
    return false;
}

void Vehicle::add_this(Vehicle *t){
    if(id_<0)return ;
    VERBOSE(2,("[VH][SD]Application accepted.(%d)\n",id_));
    if(!ifhead){
        //printf("Cannot decide!\n");
        //cin.get();
    }
    flw_list.push_back(t);
    if(VERBOSITY_LEVEL>=2){
        //for(int i=0;i<flw_list.size();i++)
            //printf("%d ",flw_list[i]->getID());
        //printf("\n");
    }
    int ind = flw_list.size();
    /// The last one is original size times the rest
    t->setpos(pos_ - ind*d_safe_sd() * KS_BWT);
    t->setlane(lane_);
    t->upd_bin();
}

void Vehicle::create_convoy(){
    convoy_id = ++convoy_count;
    VERBOSE(2,("[VH]create_convoy(%d)-%d\n",id_,convoy_id));
    
    ifhead = true;
    ifconvoy = true;
    headaddr = self;
}

void Vehicle::merge_to(Vehicle *newhead,vector<Vehicle*> *mlist){
    VERBOSE(2,("[VH][SD]Merge from (%d) to (%d) , sized (%d) to (%d)\n",newhead->id_,
        id_,mlist->size()+1,flw_list.size()+1));
    
    if(!ifhead){
        //printf("Non-head calling merge_to!%d %d\n",ifhead,ifconvoy);
        if(flw_list.size()>0){
            ifhead=true;
            ifconvoy=true;
        }
        restore();
        //cin.get();
        
    }
    flw_list.push_back(newhead);
    for(int i=0;i<mlist->size();i++)
        flw_list.push_back((*mlist)[i]);
    upd_flw_list();
    if(VERBOSITY_LEVEL>=2){
        //for(int i=0;i<flw_list.size();i++)
            //printf("%d ",flw_list[i]->getID());
        //printf("\n");
    }
    force_flwpos();
}

void Vehicle::upd_flw_list(){
    int i;
    if(!ifhead){
        //printf("Non-head using upd_flw!\n");
        //cin.get();
    }
    for(i=0;i<flw_list.size();i++)
        flw_list[i]->set_head(self);
}

void Vehicle::set_head(Vehicle *new_head){
    if(!ifconvoy){
        //printf("Non-convoy updating head!\n");
        ////cin.get();
        ifconvoy = true;
    }
    headaddr = new_head;
}

void Vehicle::upd_headarr(Vehicle *old_self,Vehicle *new_self){
    if(!ifhead){
        //printf("Non-Head using upd_headarr!(%d)\n",id_);
        //cin.get();
    }
    bool deled=false;
    for(int i=0;i<flw_list.size();i++)
        if(flw_list[i] == old_self){
            flw_list[i] = new_self;
            deled=true;
            break;
        }
    if(!deled){
        //printf("Updating failed! Not searched!(%d) (%d) - (%d)\n",id_,old_self,new_self);
        //for(int i=0;i<flw_list.size();i++)//printf("(%d,%d)",flw_list[i],flw_list[i]->getID());
        //printf("\n");
        //cin.get();
    }
}

void Vehicle::exit_in_convoy(){
    VERBOSE(2,("[VH]exit_in_convoy(%d)%d %d\n",id_,ifhead,ifconvoy));
    restore();
    if(ifhead){
        if(flw_list.size() < 1){
            //printf("Wrong sized!\n");
            //cin.get();
            return;
        }
        /// Degrade : only 2 cars
        if(flw_list.size() == 1){
            VERBOSE(2,("[VH]clearing the convoy.(%d) (%d)\n",id_,flw_list[0]->getID()));
            flw_list[0]->reset_convoy();
            return;
        }
        VERBOSE(2,("[VH]%d set %d as new head.\n",id_,flw_list[0]->getID()));
        Vehicle *bhdaddr = flw_list[0];
        flw_list.erase(flw_list.begin());
        bhdaddr->set_as_head(&flw_list);
        flw_list.clear();
        ifhead = ifconvoy = false;
    }
    else if(ifconvoy){
        VERBOSE(2,("[VH]Leave in convoy.(%d)\n"));
        headaddr->leave_convoy(self);
    }else{
        //printf("Neither head nor convoy, wrong!.\n");
        //cin.get();
    }
}

void Vehicle::set_as_head(vector<Vehicle*> *farr){
    flw_list.clear();
    flw_list = *farr;
    if(ifhead){
        //printf("Already head calling set_as_head!\n");
        //cin.get();
    }
    ifhead = true;
    ifconvoy = true;
    headaddr = self;
    upd_flw_list();
}

void Vehicle::leave_convoy(Vehicle *tar){
    VERBOSE(2,("[VH]leave_convoy(%d).\n",id_));

    bool flag=false;
    if(!ifhead){
        //printf("Non-head calling force_flwpos()!\n");
        //cin.get();
    }
    vector<Vehicle*>::iterator it=flw_list.begin();
    for(;it<flw_list.end();it++)
        if(*it == tar){
            flag = true;
            (*it)->reset_convoy();
            flw_list.erase(it);
            break;
        }
    force_flwpos();
    if(!flag){
        //printf("Not leaved!\n");
        //cin.get();
    }
    if(flw_list.size()<1){
        reset_convoy();
    }
}

void Vehicle::force_flwpos(){
    int i;
    for(i=0;i<flw_list.size();i++){
        flw_list[i]->setpos(pos_ - len_);
        flw_list[i]->upd_bin();
        /// we don't care wether the car falls outside the segment
        /// if it is smaller than 0 in the segment, then take 0
    }
}
