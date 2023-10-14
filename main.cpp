#include "main.h"

int main() {
    Simulator sim;
    CmdSignal cmds;
    CmdSignalBase *cmdPtr;

    // choose my cmd
    ConstantCmdSignal cmd(1);
    cmdPtr = (CmdSignalBase*) &cmd;    
    cmds.addCmd_robotStateV(cmdPtr, 1, 3);
    cmds.addCmd_robotStateVn(cmdPtr, 2, 4);

    // simulate
    sim.simulateFor(5, cmds);
    
    return 0;
}