#ifndef CARGEN_H_
#define CARGEN_H_

#include "common.h"
#include "vehicle.h"
#include "randgen.h"
#include <stdlib.h>
#include <time.h>

class VehicleGenerator{
public:
    VehicleGenerator(){
        isdp = 0.9; /// hm for default
        lane_num = 5;
        v_ = 70/3.6;
        pos_ = 10;
        lane_ = 0; 
        entertime_ = 0;
        len_ = 5.0;
        //vm_ = 96.6/3.6;
        vm_ = 60 / 3.6;
        am_ = 4.0;
        kc_ = 0.7;
        kf_ = 0.7;
        rng.set_gaussian(v_,5.5);
    }
    VehicleGenerator(Pool<Vehicle>* pnt):table_(pnt){}
    void printInfo()const{
        printf("Entry Genpos=%.2lf\tFreq=%.2lf\tlane_num=%d\n",pos_,f_,lane_num);
    }
    double getpos()const{return pos_;}
    void setPSD(double psd){isdp=psd;}
    void setTable(Pool<Vehicle>* pnt){
        table_ = pnt;
    }
    void setfrequency(double k){
        f_ = k;
    }
    void setisdp(double f){isdp = f;}
    bool isNegative()const{return f_<0.01;}

    bool randLeave()const{
        if(f_>-0.001) return false;
        return (RAND01+f_)<0.001;
    }

    int genVehi(int id,int T_){
        if(f_<0.0001)return id;
        VERBOSE(2,("[VG]genVehi(%d).\n",id));
        double res = RAND01;
        if(f_ < res){
            VERBOSE(2,("[VG]Not generating %.2lf.\n",f_));
            return id;
        }
        int index = table_->setNew();
        if(index == -1){
            printf("Failed to allocate new car!\n");
            cin.get();
            return id;
        }

        Vehicle *s=&((*table_)[index]); /// The pointer of the new vehicle
        
        bool iselfdrive = (RAND01<isdp);
        if(iselfdrive) rng.set_gaussian(v_,1);
        double v = rng.gaussian();
        double pos = (pos_);
        int lane = rand()%lane_num; 
        int entertime = T_;
        double len = len_ + DELTA(1);
        double vm = vm_ + DELTA(5);
        if(iselfdrive) vm = vm_ + DELTA(1);
        double am = am_ + DELTA(2);
        double kc = kc_ + double(DELTA(1))/10.;
        double kf = kf_ + double(DELTA(1))/10.;   
        /// The order of parameters is the same in function
        (*table_)[index].setInit(s,id,iselfdrive,v,pos,lane,entertime,len,vm,am,kc,kf);
        VERBOSE(2,("[VG]New vehicle generated. (pos=%.2lf)\n",pos));
        return id+1;
    }
    void setlanenum(int num){lane_num = num;}
    void setbgpos(double pos){pos_ = pos;}
    void setf(int flow){f_ = double(flow) /24. / 3600.;}
private:
    RandNumGenerator rng;

    double f_;
    /// The table generators going to modify
    Pool<Vehicle>* table_;
    //vector<Pool<IVehic> >** segbin;

    double isdp;

    int lane_num;
    double v_;
    double pos_;
    int lane_; 
    int entertime_;
    double len_;
    double vm_;
    double am_;
    double kc_;
    double kf_;
};

#endif
