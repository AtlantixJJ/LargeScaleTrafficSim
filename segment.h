 // Segment.h

#ifndef Segment_h_
#define Segment_h_

#include <set>
#include "common.h"
#include "vehicle.h"
#include "cargen.h"
using namespace std;

class Segment
{
public:
	Segment();
	~Segment();
	/// linear sequence
	set<IVehic > carseq[MAX_LANE];
	double seg_start;
	double seg_end;
	double seg_dist;
	int lane_num;
	int car_num;
	int no;
	int upbound,lowbound,maxbound;
	Segment *next_;
	VehicleGenerator *corentry;
	Vehicle empty;
	IVehic example;
	bool stuck;
	
	void printInfo()const;
	//vector<Pool<IVehic> >** getBinPnt(){return &bin_;}
	void setInit(double _seg_start, double _seg_end, int _lane_num);
	void sort_all(int l);
	void setno(int no){this->no = no;}
	void setNext(Segment *next){next_ = next;}
	void setEntry(VehicleGenerator *en){corentry = en;}
	bool hasNext()const {return next_ != NULL;}
	Segment* getNext()const{return next_;}
	double getst()const{return seg_start;}
	double geted()const {return seg_end;}
	bool is_stuck()const{return stuck;}
	void reset_stuck() {stuck=false;}
	int getlane()const{return lane_num;}
	void recheck();

	Vehicle* search(Vehicle *cur,double pos,int lane,bool flag);
	Vehicle* search_forsd(Vehicle *cur,double pos,int lane);

	int search_in_bin(int id,int lane,int oribin);

	bool remove(int id,double lastpos,int orilane);

	int modify(Vehicle *cur,int id,double pos,double lastpos,int curlane,int orilane);

	////// Return the bin index
	int insert(Vehicle *cur,int id,double pos,int curlane);
};

#endif //Segment_h_
