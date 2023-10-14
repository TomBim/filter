#ifndef CMD_SIGNAL_H
#define CMD_SIGNAL_H

#include <vector>
#include <eigen3/Eigen/Core>


enum class SignalType{ Constant, Line, /*Triangular, Pulse*/ };

// @todo make this happen

class CmdSignalBase {
    private:
        const SignalType signalType;
    
    public:
        CmdSignalBase(SignalType signalType) : signalType(signalType) {}

        virtual ~CmdSignalBase() {}

        /// @brief gg
        /// @param timeNow in seconds.
        /// @return commmand at the given time.
        virtual double getCmdNow(double timeNow) const = 0;

        virtual CmdSignalBase* copy() const = 0;
};

class ConstantCmdSignal : CmdSignalBase {
    private:
        const double constantValue;

    public:
        ConstantCmdSignal(const double constantValue) : CmdSignalBase(SignalType::Constant), 
            constantValue(constantValue) {}
        
        double getCmdNow(double timeNow) const { return constantValue; }

        CmdSignalBase* copy() const { return (CmdSignalBase*) new ConstantCmdSignal(constantValue); }
};

class LineCmdSignal : CmdSignalBase {
    private:
        const double a, b; // f(t) = a.t + b

    public:
        LineCmdSignal(const double a, const double b) : CmdSignalBase(SignalType::Line),
            a(a), b(b) {}

        LineCmdSignal(const double value0, const double t0, const double value1, const double t1) :
            CmdSignalBase(SignalType::Line),
            a((value1 - value0) / (t1 - t0)),
            b(value0 - a*t0) {}
        
        double getCmdNow(double timeNow) const { return a * timeNow + b; }

        CmdSignalBase* copy() const { return (CmdSignalBase*) new LineCmdSignal(a,b); }
};

class CmdSignalOneWheel {
    private:
        std::vector<CmdSignalBase*> cmds;
        std::vector<double> timeVec;    // in seconds

    public:
        CmdSignalOneWheel();

        ~CmdSignalOneWheel();

        int addCmd(const CmdSignalBase *cmd, double startTime, double endTime);

        double getCmdNow(double timeNow) const;

        void reset();
};

class CmdSignal {
    private:
        static const int V_INDEX = 0;
        static const int Vn_INDEX = 1;
        static const int OMEGA_INDEX = 2;
        std::vector<CmdSignalOneWheel> desiredRobotStates;

        /// @param startTime time to start the cmd (in seconds)
        /// @param endTime to start the cmd (in seconds)
        int addCmd(const CmdSignalBase *cmd, double startTime, double endTime, const int stateIndex);    
    public:
        CmdSignal();
        
        ~CmdSignal();

        /// @param startTime time to start the cmd (in seconds)
        /// @param endTime to start the cmd (in seconds)
        int addCmd_robotStateV(const CmdSignalBase *cmd, double startTime, double endTime);

        /// @param startTime time to start the cmd (in seconds)
        /// @param endTime to start the cmd (in seconds)
        int addCmd_robotStateVn(const CmdSignalBase *cmd, double startTime, double endTime);

        /// @param startTime time to start the cmd (in seconds)
        /// @param endTime to start the cmd (in seconds)
        int addCmd_robotStateOmega(const CmdSignalBase *cmd, double startTime, double endTime);

        Eigen::Vector3d getCmdNow(double timeNow) const;

        void reset();
};

#endif