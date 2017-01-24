// Vehicle.h

#ifndef Vehicle_h_
#define Vehicle_h_

#include "common.h"
using namespace std;

class Vehicle
{
	friend class VehicleGenerator;
public:
	Vehicle& operator = (const Vehicle& fr);
	Vehicle(const Vehicle& fr);
	Vehicle();
	~Vehicle();

	/// Clear the content but spare for new cars
	bool clearit();
	void printInfo();
	void setSeg(Segment *seg);
	void initSelf(Vehicle *s){self=s;}
	void resetSelf(Vehicle *s);
	void setInit(Vehicle *s,int id,bool iselfdrive,double v,double pos,int lane, 
		int entertime,double len,double vm,double am,double kc,double kf);
	void setv(double v){v_=v;}
	void setpos(double pos){pos_ = pos;}
	void setlane(int newlane){lane_ = newlane;}
	void setid(int id){this->id_ = id;}
	void resetTime(int time){entertime_=time;}

	////// Update States
	void randLeave();
	void upd_pos(); //void upd_pos_hm();
	void upd_vec(); //void upd_vec_hm();
	double upd_acc(); double upd_acc_hm(); double upd_acc_sd();
	bool upd_bin();
	bool upd_convoy();
	bool alter(); bool alter_hm(); bool alter_sd();
	//bool alter_to(int new_lane, double v_front,double hs=1);
	bool alter_to_hm(int new_lane,double v_front);
	bool alter_to_sd(int new_lane,double v_front,double hc=1);

	Result detect_cross(int T);

	////// Self-driving car operations
	/// Change following car's father pointer in case father pointer changes
	void upd_flw_list();
	void set_head(Vehicle *new_head);
	void upd_headarr(Vehicle *old_self,Vehicle *new_self);
	void exit_in_convoy();
	/// A car in the convoy leaves
	void leave_convoy(Vehicle *tar);
	
	void restore(){
		if(ifconvoy && headaddr == NULL){
        //cin.get();
			if(flw_list.size()>0){
				for(int i=0;i<flw_list.size();i++)
					flw_list[i]->reset_convoy();
			}
			reset_convoy();
		}
	}

	void reset_convoy() { 
		ifhead = ifconvoy = ifupdacc = false;
		convoy_id = -1;
		headaddr = NULL;	
		flw_list.clear();
	}
	void force_flwpos();
	void set_as_head(vector<Vehicle*> *farr);
	void add_this(Vehicle *t);
	void create_convoy();
	void merge_to(Vehicle *newhead,vector<Vehicle*> *mlist);

	////// Information Interface
	bool add_enabled(int x)const{ return flw_list.size()+x<MAX_CONVOY_LENGTH;}
	bool ISD()const{return isd;}
	bool hasupdacc()const {return ifupdacc;}
	int convoy_len()const{
		if(!ifhead)return -1;
		return flw_list.size()+1;
	}
	bool ishead()const{return ifhead;}
	bool isconvoy()const{return ifconvoy;}
	int get_convoy_id()const{return convoy_id;}
	inline bool ifSelf()const{return self!=NULL;}
	inline double getpos()const{return pos_;}
	inline double getv()const {return v_;}
	inline double geta()const {return a_;}
	Segment* getseg(){return curseg_;}
	inline int getlane()const {return lane_;}
	inline int getID()const {return id_;}
	inline int isInitialized()const {return curseg_ != NULL && self != NULL;}

private:
	//static int convoy_id;

	/////// Stimulation Variables
	vector<Vehicle*> flw_list;
	Vehicle* self;
	Vehicle* headaddr;
	Segment* curseg_;
	bool exit_;	/// If the vehicle exit at the end of the current segment

	bool isd;
	bool ifhead,ifconvoy;
	bool ifupdacc;

	double pos_,lastpos_,v_,a_,rundist;
	int acumtime;
	int lane_,orilane_;
	int entertime_; /// the time to enter the segment
	int id_;
	int convoy_id;

	/////// Property Variables
	double len_; /// The length of the car
	double vm_; /// Maximum Speed
	double am_; /// Maximum Acceleration

	double kc_; // coefficient for change lane
	double kf_; // coefficient for free mode (Acceleration)


	double d_safe_hm()const {return 0.6 * (v_ + v_ * v_ / (2 * am_) + len_ ); }
	double d_safe_sd()const {return 2*d_safe_hm(); return v_ * v_ / (2 * am_) + len_; }
	double d_dec_sd(double v_front)const {
		if(v_ < v_front) return 0;
		return (v_ - v_front) * (v_ - v_front) / (2 * am_); 
	}
	double d_dec_hm(double v_front)const {
		if(v_ < v_front) return 0;
		return (v_ - v_front) * (v_ - v_front) / am_; 
	}
	double rand_01()const {return ( rand() * 1.0 / RAND_MAX );}
};

//Vehicle FAREST,NEAREST;

#endif // Vehicle_h_
