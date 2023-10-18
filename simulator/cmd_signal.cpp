#include "cmd_signal.h"


CmdSignalOneWheel::CmdSignalOneWheel() {
    cmds.push_back((CmdSignalBase*) new ConstantCmdSignal(0));
    timeVec.push_back(0);
}

CmdSignalOneWheel::~CmdSignalOneWheel() {
    CmdSignalBase* cmd;
    while(!cmds.empty()) {
        cmd = cmds.back();
        if(cmd != nullptr)
            delete cmd;
        cmds.pop_back();        
    }
}

int CmdSignalOneWheel::addCmd(const CmdSignalBase *cmd, double startTime, double endTime) {
    if(endTime < 0)
        return 1;
    if(startTime >= endTime)
        return 1;

    int posBeforeStart = -1;
    int posAfterEnd = timeVec.size();
    if(timeVec.size() > 0) {
        double t;
        for(int i = 0; i < timeVec.size(); i++) {
            t = timeVec.at(i);
            if(t < startTime)
                posBeforeStart = i;
            else if(posAfterEnd < 0 && t > endTime) {
                posAfterEnd = i;
                break;
            }
        }
    }

    // first, let's delete the cmds that has been replaced by the new cmd
    while(posAfterEnd - posBeforeStart > 2) {
        // the position after 'posBeforeStart' represents a cmd
        // that has been replaced
        timeVec.erase(timeVec.begin() + posBeforeStart + 1);
        CmdSignalBase* cmdReplaced = cmds.at(posBeforeStart + 1);
        cmds.erase(cmds.begin() + posBeforeStart + 1);
        delete cmdReplaced;
        posAfterEnd--;
    }

    // now we have 2 cases: or we are inserting inside one cmd, like:
    //      A______A -> A___AB_BA_A
    // or our new cmd isn't inside only one cmd:
    //      A__AB___B -> A_AC__CB__B
    // in the first case, we have (posAfterEnd - posBeforeStart == 1)
    if(posAfterEnd - posBeforeStart == 1) {
        // in this case, we must insert also a copy after our new cmd
        // ends
        cmds.insert(cmds.begin() + posBeforeStart + 1, cmd->copy());
        cmds.insert(cmds.begin() + posBeforeStart + 2, cmds.at(posBeforeStart)->copy());
        timeVec.insert(timeVec.begin() + posBeforeStart + 1, startTime);
        timeVec.insert(timeVec.begin() + posBeforeStart + 2, endTime);
    }
    // in the second case
    // (posAfterEnd == posBeforeStart) shouldn't happen, but by precaution, let's 
    // make another if
    else if(posAfterEnd - posBeforeStart == 2) {
        // in this case, we just insert between and modifie the time of the start
        // of the last cmd of the 3
        cmds.insert(cmds.begin() + posBeforeStart + 1, cmd->copy());
        timeVec.insert(timeVec.begin() + posBeforeStart + 1, startTime);
        timeVec[posBeforeStart + 2] = endTime;
    }
    else return 1; // just precaution
    
    return 0;
}

double CmdSignalOneWheel::getCmdNow(double timeNow) const {
    if(timeNow < 0)
        return 0;

    for(int i = 0; i < timeVec.size(); i++)
        if(timeVec.at(i) > timeNow)
            return cmds.at(i-1)->getCmdNow(timeNow);
    // if didn't find, its because the last one is where we are
    return cmds.at(timeVec.size() - 1)->getCmdNow(timeNow);
}

void CmdSignalOneWheel::reset() {
    CmdSignalBase* cmd;
    while(!cmds.empty()) {
        cmd = cmds.back();
        if(cmd != nullptr)
            delete cmd;
        cmds.pop_back();        
    }
    cmds.push_back((CmdSignalBase*) new ConstantCmdSignal(0));
    timeVec.push_back(0);
}




CmdSignal::CmdSignal() : desiredRobotStates(3) {}

CmdSignal::~CmdSignal() {}

int CmdSignal::addCmd_robotStateV(const CmdSignalBase *cmd, double startTime, double endTime) {
    return addCmd(cmd, startTime, endTime, V_INDEX);
}

int CmdSignal::addCmd_robotStateVn(const CmdSignalBase *cmd, double startTime, double endTime) {
    return addCmd(cmd, startTime, endTime, Vn_INDEX);
}

int CmdSignal::addCmd_robotStateOmega(const CmdSignalBase *cmd, double startTime, double endTime) {
    return addCmd(cmd, startTime, endTime, OMEGA_INDEX);
}

int CmdSignal::addCmd(const CmdSignalBase *cmd, double startTime, double endTime, const int stateIndex) {
    return desiredRobotStates.at(stateIndex).addCmd(cmd, startTime, endTime);
}

Eigen::Vector3d CmdSignal::getCmdNow(double timeNow) const {
    Eigen::Vector3d cmdValues;
    for(int i = 0; i < 3; i++)
        cmdValues[i] = desiredRobotStates.at(i).getCmdNow(timeNow);
    return cmdValues;
}

void CmdSignal::reset() {
    for(int i = 0; i < 4; i++)
        desiredRobotStates.at(i).reset();
}