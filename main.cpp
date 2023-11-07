#include "main.h"

int main() {
    std::cout << "starting program" << std::endl << std::endl;

    #ifdef DBG_MODE
    startDebugLog();
    #endif


    Simulator sim;
    CmdSignal cmds;
    CmdSignalBase *cmdPtr;

    // choose my cmd
    ConstantCmdSignal cmd(1);
    cmdPtr = (CmdSignalBase*) &cmd;    
    cmds.addCmd_robotStateV(cmdPtr, 0, 10);

    // simulate
    std::cout << "starting simulation" << std::endl << std::endl;
    sim.simulateFor(1, cmds);
    std::cout << "finished simulation! Finishing program" << std::endl;

    return 0;
}