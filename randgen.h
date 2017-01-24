#ifndef RANDGEN_H_
#define RANDGEN_H_

#include "common.h"

class RandNumGenerator{
public:
    RandNumGenerator(){
        mean = 0;
        sigma = 0;
        V1 = V2 = phase = 0;
    }

    ~RandNumGenerator(){}

    void set_gaussian(float mean,float sigma){
        this->mean = mean;
        this->sigma = sigma;
    }

    double gaussian(){
        double X;
        if ( phase == 0 ) {
            do {
                double U1 = (double)rand() / RAND_MAX;
                double U2 = (double)rand() / RAND_MAX;
                V1 = 2 * U1 - 1;
                V2 = 2 * U2 - 1;
                S = V1 * V1 + V2 * V2;
            } while(S >= 1 || S == 0);
            X = V1 * sqrt(-2 * log(S) / S);
        } else
            X = V2 * sqrt(-2 * log(S) / S);
        phase = 1 - phase;
        return X*sigma+mean;
    }
private:
    float mean,sigma;

    ///// Gaussian Variables
    double V1,V2,S,phase;
};



#endif