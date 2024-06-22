//
// Created by ilia on 11.6.24..
//

#ifndef FROG_RMSMETER_H
#define FROG_RMSMETER_H

#define RMS_BUF_SIZE (4096)
class RMSMeter {
public:
    RMSMeter();
    ~RMSMeter() = default;
    float process(float in);
    float get();
private:
    float buf[RMS_BUF_SIZE] = {};
    float currRMS = 0;
};


#endif //FROG_RMSMETER_H
