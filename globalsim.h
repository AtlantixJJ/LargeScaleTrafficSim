#ifndef GLOBALSIM_H_
#define GLOBALSIM_H_

#include "common.h"
#include "vehicle.h"
#include "segment.h"
#include "cargen.h"

class GlobalSimulator{
public:
    GlobalSimulator(const char *path="./data/5.txt")
    {
        VERBOSE(2,("[GS]GlobalSimulator():: \n"));
        bool sflag;
        
        initSegEntFile(path);
        
        SAFECALL(_vhi.init(MAXN));

        VERBOSE(2,("[GS]GlobalSimulator():: \n"));
    }

    void closeall(){
        fclose(logging);
        fclose(res_file);
    }

    ~GlobalSimulator(){

    }

    /// @param flag true for forward direction only
    void initSegEntFile(const char *fpath="./data/5.txt",bool flag=true){
        int i,j,flow,flane,blane,index,lastflow;
        double start,end,zerof;
        bool sflag;

        FILE *segfile = fopen(fpath,"r");
        fscanf(segfile,"%d",&seg_num);

        SAFECALL(_segs.init(seg_num+1));
        SAFECALL(_entry.init(seg_num+2));

        /// Set the first line
        fscanf(segfile,"%lf %lf %d %d %d",&zerof,&end,&lastflow,&flane,&blane);
        index = _segs.setNew();
        if(!flag)flane=blane;
        _segs[index].setno(1);
        _segs[index].setInit(0,(end-zerof)*1000,flane);

        index = _entry.setNew();
        _entry[index].setbgpos(0);
        _entry[index].setlanenum(flane);
        _entry[index].setTable(&_vhi);
        _entry[index].setf(lastflow/2);

        _segs[index].setEntry(&_entry[index]);

        for(i=2;i<seg_num;i++){
            fscanf(segfile,"%lf %lf %d %d %d",&start,&end,&flow,&flane,&blane);
            index = _segs.setNew();
            if(!flag)flane=blane;
            _segs[index].setInit((start-zerof)*1000,(end-zerof)*1000,flane);
            _segs[index].setno(i);
            _segs[index-1].setNext(&_segs[index]);

            index = _entry.setNew();
            _entry[index].setbgpos((start-zerof)*1000);
            _entry[index].setlanenum(flane);
            _entry[index].setTable(&_vhi);
            _entry[index].setf((flow-lastflow)/2);
            lastflow = flow;

            _segs[index].setEntry(&_entry[index]);
            //_segs[index].printInfo();
            //_entry[index].printInfo();
        }
        fclose(segfile);
    }

    void setTime(int t){T=t;}
    
    void initStim(int t,int percent,int entry){
        int ind;
        int i;
        int num_entry = 1;
        char func[256];

        count = 1;
        T = t;
	    this->vb = 1;
        this->psd = double(percent)/100.;

        sprintf(func,"res%d-%d.txt",percent,entry);
        logging = fopen("log.txt","w");
        res_file = fopen(func,"w");

        for(i=1;i<_entry.top;i++) _entry[i].setPSD(psd);

        fprintf(logging,"%d %d\n",t,5);
        for(i=1;i<_entry.top;i++)
            fprintf(logging,"%.2f ",_entry[i].getpos());
        fprintf(logging,"\n");
    }

    void Simulate(){
        int i,j;
        bool sflag;
        now = 0;
        Result res;
        
        VERBOSE(0,("[GS]Simulation Start. ( T=%d , interv=%d , thre=%d )\n",
            T,interval,thre));
        while(now < T){
            if(2<=vb)printf("[S](Time:%d)(Car:%d)\n",now,_vhi.top);
            else if(1<=vb && now % 100 == 0)printf("[GS](Time:%d)(Car:%d)\n",now,_vhi.top);
            /// Update vehicle status
            for(i=1;i<_vhi.top;i++){
                if(!_vhi[i].isInitialized()){
                    SAFECALL(_vhi.remove(i));
                    continue;
                }
                _vhi[i].upd_pos();
                _vhi[i].upd_vec();
                _vhi[i].upd_bin();
                if(_vhi[i].ISD())_vhi[i].upd_convoy();
            }

            /// detect crossing and leaving
            /// update acceleration            
            for(i=1;i<_vhi.top;i++){
                res = _vhi[i].detect_cross(now);
                if(res.exit){
                    VERBOSE(2,("Leaved %d\n",_vhi[i].getID()));
                    fprintf(res_file,"%d %d %d %.2lf %d\n",_vhi[i].ISD(),_vhi[i].isconvoy(),_vhi[i].getseg()->no,res.v,res.t);
                    SAFECALL(_vhi.remove(i)); /// now original i is moved to top 
                    if(i != _vhi.top) _vhi[i].resetSelf(&_vhi[i]);
                    
                    continue;
                }
                if(res.t != -1 && res.t != -2){
                    //double v = _vhi[i].getseg()->seg_dist / res.t;
                    //if(res.t<10)continue;
                    fprintf(res_file,"%d %d %d %.2lf %d\n",_vhi[i].ISD(),_vhi[i].isconvoy(),_vhi[i].getseg()->no,res.v,res.t);
                    VERBOSE(2,("Crossed %d\n",_vhi[i].getID()));
                    _vhi[i].resetTime(now);
                    _vhi[i].randLeave();
                }
                _vhi[i].alter();
                //acc_val[i] = 
                _vhi[i].upd_acc();
            }
        
            /// logging car states
            if(1<=vb){
                fprintf(logging,"R %d\n",now);
                for(i=1;i<_vhi.top;i++){
                    //fprintf(logging,"%d %d %.2f %.2f %.2f %d\n",_vhi[i].getID(),_vhi[i].get_convoy_id(),_vhi[i].getlane(),
                    //    _vhi[i].getpos(),_vhi[i].convoy_len());
                    fprintf(logging,"%d %d %d %.2f\n",_vhi[i].getID(),_vhi[i].getlane(),_vhi[i].isconvoy(),_vhi[i].getpos());
                }
            }
            
            /// all the entry with f>0 should produce cars
            for(i=1;i<_entry.top;i++){
                if(_entry[i].isNegative()) continue;
                if(_segs[i].is_stuck()){
                    VERBOSE(2,("."));
                    _segs[i].reset_stuck();
                    continue;
                }
                if(count == _entry[i].genVehi(count,now))continue;
                /// i-th entry must produce cars in i-th segment
                _vhi[_vhi.top-1].setSeg(&_segs[i]);
                _vhi[_vhi.top-1].initSelf(&_vhi[_vhi.top-1]);
                _vhi[_vhi.top-1].randLeave();
                _vhi[_vhi.top-1].upd_bin();
                //printf("[NEWCAR]%.2lf %.2lf\n",_vhi[_vhi.top-1].getpos(),_segs[i].getst());
                count++;
            }
            now++;
            //cin.get();
        }
        closeall();
        VERBOSE(0,("[GE]Simulation Done."));
    }

/*
    void test(){
        /// Set one initial segment
        int ind = _segs.setNew();
        _segs[ind].setInit(0,10000,5);

        /// Test generator
        int cur,i,N=10;
        for(i=0;i<N;i++){
            _entry[1].genTest(i+3);
        }
        for(i=1;i<_vhi.top;i++){
            if(!_vhi[i].ifSelf())continue;
            _vhi[i].setSeg(&_segs[1]);
            _vhi[i].upd_bin();
        }
        _segs[1].sort_all(0);
        for(i=1;i<_vhi.top;i+=3){
            printf("%d\n",i);
            _vhi[i].printInfo();
            _vhi[i].clearit();
            _vhi.remove(i);
        }
        _segs[1].sort_all(0);
    }
*/
private:
    FILE *logging;
    FILE *res_file;

    Pool<Segment> _segs;
    int seg_num;
    
    Pool<Vehicle> _vhi;
    Pool<VehicleGenerator> _entry;

    /////// Simulation Variables
    int T;
    int vb; /// control the debug output
    int now;
    int count; /// control the generated car ID
    int interval;
    int thre;   /// simplified generating option

    ////// Logging value
    double psd;

};

#endif
