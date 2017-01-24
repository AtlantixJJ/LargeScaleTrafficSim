#ifndef COMMON_h_
#define COMMON_h_

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
using namespace std;

const int VERBOSITY_LEVEL = 0;
const int MAXN = 5000000,MAX_SEGS = 150;
const int MAX_LANE = 10;
const int MAX_CONVOY_LENGTH = 20;
const int MAX_CONF = 10;
const double SPEED_LIM = 96.6/3.6;
const double MPB = 10;
const int MRAND = 23456789;
const double FARTHRE = 800.;
const double KS_BWT = 0.3;
const double KS_HSD = 0.2;
const double FAR_SD = 50.;

const int DT_ARR[3] = {-1,0,1};

#define DELTA(x) (rand()%(2*x+1) - x)
#define RAND01 ((float)(rand()%MRAND) / (float)(MRAND))

#define ASSERT_EQ(A,B,str) if( (A) != (B) ) {printf("ASSERT_EQ Failed!");printf(str);cin.get();}
#define SAFENULL(expr) sp=expr; if(sp == NULL) printf("failed!\n")
#define SAFECALL(expr) sflag=expr; if(sflag == false) printf("failed!\n") 
#define INSTANTIATE(P) template Pool<P>
#define VERBOSE(l,x) if(l <= VERBOSITY_LEVEL ){printf x;}

#define ABS(x) (((x)>0?(x):-(x)))

class VehicleGenerator;
class GlobalSimulator;
class Segment;
class Vehicle;

struct IVehic{
    bool isd;
    int id;
    double pos,v;
    Vehicle *addr;
    IVehic():id(-1),pos(-1),v(-1),addr(NULL){}
    void clearit(){
        id = pos = v = -1;
    }
    bool operator < (const IVehic &a) const{
        return pos < a.pos;
    }
    bool operator < (double pos_) const{
        return pos < pos_;
    }
    bool operator == (double pos_)const{
        return ABS(pos - pos_)<0.001;
    }
};

struct Result{
    int t;
    bool exit;
    double v;
    Result():t(-1),exit(false){}
};


///// Note : when adding a NEW element , you must call setNew first
/////        when modifying a exsiting element, it is not needed
template<class DType>
class Pool{
public:
    /// top begin from 1 ; 0 is null
    Pool():top(1),M(0){
        data_ = NULL;
    }
    const DType& operator [](int x)const{
        if(x<1||x>=top) return data_[0];
        return data_[x];
    }
    DType& operator [](int x){
        /// allow for modification at top
        if(x<1||x>top) return data_[0];
        return data_[x];
    }
    int setNew(){
        if(data_!=NULL && top<M) return top++;
        else {
            printf("Space overflow!Re allocating space...\n%d %d\n",top,M);
            delete [] data_;
            DType *tmp = new DType[M<<1];
            for(int i=1;i<top;i++) tmp[i] = data_[i];
            data_ = tmp;
            M = (M<<1)-1;
            return top++;
        }
    }
    bool release(){
        if(data_)delete [] data_;
        else return false;
        data_ = NULL;
        M = 0;
        top = 1;
        return true;
    }
    ~Pool(){release();}
    bool init(int M_,const char *na="POOL\0"){
        M = M_;
        //name = (char *)na;
        if(data_ != NULL) return false;
        if(M <= 0)return false;
        data_ = new DType[M+1];
        top=1;
        if(data_ != NULL) return true;
        return false;
    }
    bool remove(int x){
        if(x<1 || x>=top)return false;      
        if(x<top-1){
            data_[x].clearit();
            data_[x] = data_[top-1];
            data_[top-1].clearit();
        }
        top--;
        //data_[top].clearit();
        return true;
    }
    int top;
    char *name;
    DType *data_;
    int M;
};

//INSTANTIATE(Vehicle);INSTANTIATE(Segment);

#endif
