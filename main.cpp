#include "main.h"

int main() {
    std::cout << "starting program" << std::endl << std::endl;

    #ifdef DBG_MODE
    startDebugLog();
    #endif


    Simulator sim;
    CmdSignal cmds;
    CmdSignalBase *cmdPtr, *cmdPtrMinus;

    // choose my cmd
    ConstantCmdSignal cmd(1), cmdMinus(-1);
    cmdPtr = (CmdSignalBase*) &cmd;
    cmdPtrMinus = (CmdSignalBase*) &cmdMinus;
    cmds.addCmd_robotStateV(cmdPtr, 0, 1);
    cmds.addCmd_robotStateVn(cmdPtr, 1, 2);
    cmds.addCmd_robotStateOmega(cmdPtr, 2, 3);
    cmds.addCmd_robotStateV(cmdPtrMinus, 3, 4);
    cmds.addCmd_robotStateVn(cmdPtrMinus, 4, 5);
    cmds.addCmd_robotStateOmega(cmdPtrMinus, 5, 6);
    cmds.addCmd_robotStateV(cmdPtr, 6, 10);
    cmds.addCmd_robotStateVn(cmdPtr, 7, 10);
    cmds.addCmd_robotStateOmega(cmdPtr, 8, 9);


    // simulate
    std::cout << "starting simulation" << std::endl << std::endl;
    sim.simulateFor(10, cmds);
    std::cout << "finished simulation! Finishing program" << std::endl;

    return 0;
}