#include "simulator.h"

std::string Simulator::createName4Log() {
    time_t currentTime;
    struct tm *localTime;

    time(&currentTime);
    localTime = localtime(&currentTime);

    std::ostringstream date, hour;
    date << std::setw(2) << std::setfill('0') << localTime->tm_year + 1900 \
         << std::setw(2) << std::setfill('0') << localTime->tm_mon + 1 \
         << std::setw(2) << std::setfill('0') << localTime->tm_mday;


    hour << std::setw(2) << std::setfill('0') << localTime->tm_hour \
         << std::setw(2) << std::setfill('0') << localTime->tm_min \
         << std::setw(2) << std::setfill('0') << localTime->tm_sec;

    return "results/" + date.str() + "-" + hour.str();
}

void Simulator::bufferize(int bufferPos, double timeNow) {
    timeBuffer[bufferPos] = timeNow;
    robotReadSpd_Buffer[bufferPos] = robot->estimateRobotsSpd();
    robotTrueSpd_Buffer[bufferPos] = robot->getRobotsTrueSpd();
    wheelsReadSpd_Buffer[bufferPos] = robot->getLastEncodersRead();
    wheelsTrueSpd_Buffer[bufferPos] = robot->getWheelsTrueSpd();
}

void Simulator::flushAllBuffers() {
    flushAllBuffers(bufferSize - 1);
}

void Simulator::flushAllBuffers(int printUntil) {
    // verify if printUntil is inside the allowed range
    if(printUntil < 0)
        return;
    if(printUntil >= bufferSize)
        printUntil = bufferSize - 1;

    // flush on fyles
    printBufferOnFile(fileNames.at(SpdType::RobotTrue), robotTrueSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::RobotEstimation), robotReadSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::WheelsTrue), wheelsTrueSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::WheelsRead), wheelsReadSpd_Buffer, printUntil);
}

void Simulator::printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector4d> &buffer, int printUntil) {
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()) {
        for(int i = 0; i <= printUntil; i++)
            file << timeBuffer.at(i) << "\t" << buffer.at(i).format(tableFormatTAB) << std::endl;
        file.close();
    }
}

void Simulator::printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector3d> &buffer, int printUntil) {
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()) {
        for(int i = 0; i <= printUntil; i++)
            file << timeBuffer.at(i) * seconds2timeUnity << "\t" <<  buffer.at(i).format(tableFormatTAB) << std::endl;
        file.close();
    }
}

void Simulator::startLogFiles() {
    std::ofstream file;
    std::string header;
    for(int i = 0; i < static_cast<int>(SpdType::NTypes); i++) {
        // put names into class' variable
        fileNames.putAt(i, logFileName + suffixes4logFile.at(i));

        // create the file and put a header
        file.open(fileNames.at(i), std::ios_base::out);
        if(file.is_open()) {
            std::string str2beReplaced = "{timeUnity}";
            int timeUnityPos = logFileHeaders.at(i).find(str2beReplaced);
            if(timeUnityPos >= 0)
                file << logFileHeaders.at(i).replace(timeUnityPos, str2beReplaced.size(), timeUnity) << std::endl;
            else
                file << logFileHeaders.at(i) << std::endl;
            file.close();
        }
    }
}

Simulator::Simulator() : robotTrueSpd_Buffer(bufferSize), robotReadSpd_Buffer(bufferSize),
                         wheelsTrueSpd_Buffer(bufferSize), wheelsReadSpd_Buffer(bufferSize),
                         timeBuffer(bufferSize), tableFormatTAB(Eigen::StreamPrecision, 0, " ", "\t") {
    robot = new Robot(STD_DEV_NOISE_MOV);
}

Simulator::~Simulator() {
    delete robot;
}

void Simulator::simulateFor(double duration, const std::string logFileName, const CmdSignal &cmdSignal) {
    // check if duration is valid
    if(duration <= 0)
        return;

    // start logs
    this->logFileName = logFileName;
    startLogFiles();

    // simulate
    const int nSteps = ceil(duration * Fs);    
    int bufferPos = 0;
    for(int step = 0; step < nSteps; step++) {
        #ifdef DBG_MODE
        std::cout << std::endl << "# STEP = " << step << std::endl;
        #endif
        robot->updateRobotStatus(cmdSignal.getCmdNow(Ts * step));
        bufferPos = step % bufferSize;


        if(bufferPos == 0 && step > 0)
            flushAllBuffers();

        bufferize(bufferPos, Ts * step);
    }

    // flush the rest of the buffer
    flushAllBuffers(bufferPos);

    // finish simulation
    // nothing here, for now xD
}

void Simulator::simulateFor(double duration, const CmdSignal &cmdSignal) {
    this->simulateFor(duration, createName4Log(), cmdSignal);
}

void Simulator::setTimeUnity(std::string timeUnity) {
    if(timeUnity == "s") {
        this->timeUnity = "s";
        this->seconds2timeUnity = 1;
    }
    else if(timeUnity == "ms") {
        this->timeUnity = "ms";
        this->seconds2timeUnity = 1e3;
    }
    else if(timeUnity == "us") {
        this->timeUnity = "us";
        this->seconds2timeUnity = 1e6;
    }
    else if(timeUnity == "ns") {
        this->timeUnity = "ns";
        this->seconds2timeUnity = 1e9;
    }
    else if(timeUnity == "ps") {
        this->timeUnity = "ps";
        this->seconds2timeUnity = 1e12;
    }
}

const vecEachSpdType<std::string> Simulator::suffixes4logFile(
    std::vector<SpdType>({SpdType::RobotEstimation, SpdType::RobotTrue, SpdType::WheelsRead, SpdType::WheelsTrue}),
    std::vector<std::string>({"rEstim.txt",         "rTrue.txt",        "wRead.txt",         "wTrue.txt"})
);

const vecEachSpdType<std::string> Simulator::logFileHeaders(
    std::vector<SpdType>({SpdType::RobotEstimation, SpdType::RobotTrue, SpdType::WheelsRead, SpdType::WheelsTrue}),
    std::vector<std::string>({("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)")})
);