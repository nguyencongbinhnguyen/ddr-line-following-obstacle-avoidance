#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

struct PwmCommand {
    float left;
    float right;
    float cmdL;
    float cmdR;
    float alpha;
    bool feasible;
};

struct Output {
    float uv;
    float uw;
    float tauL;
    float tauR;
    float cmdL;
    float cmdR;
    float alpha;
    float pwmL;
    float pwmR;
    float z2;
    float wd;
    float wdd;
    int fault;
    float thetaEq;
    float e2;
    float bb;
    float targetRaw;
    float target;
};

#endif