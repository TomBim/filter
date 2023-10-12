#include "simulator.h"

std::string Simulator::createName4Log() {
    time_t currentTime;
    struct tm *localTime;

    time(&currentTime);
    localTime = localtime(&currentTime);

    std::string date, hour;
    date = std::to_string(localTime->tm_year) + std::to_string(localTime->tm_mon) + std::to_string(localTime->tm_mday);
    hour = std::to_string(localTime->tm_hour) + std::to_string(localTime->tm_min) + std::to_string(localTime->tm_sec);

    return "results/" + date + "-" + hour;
}

void Simulator::bufferize(int bufferPos) {
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
    printBufferOnFile(fileNames.at(spdType::RobotTrue), robotTrueSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(spdType::RobotEstimation), robotReadSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(spdType::WheelsTrue), wheelsTrueSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(spdType::WheelsRead), wheelsReadSpd_Buffer, printUntil);
}

void Simulator::printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector4d> &buffer, int printUntil) {
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()) {
        for(int i = 0; i <= printUntil; i++)
            file << timeBuffer.at(i) << "\t" << buffer.at(i).format(tableFormatTAB);
        file.close();
    }
}

void Simulator::printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector3d> &buffer, int printUntil) {
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()) {
        for(int i = 0; i <= printUntil; i++)
            file << timeBuffer.at(i) << "\t" <<  buffer.at(i).format(tableFormatTAB);
        file.close();
    }
}

void Simulator::startLogFiles() {
    std::ofstream file;
    std::string header;
    for(int i = 0; i < static_cast<int>(spdType::NTypes); i++) {
        // put names into class' variable
        fileNames.putAt(i, logFileName + suffixes4logFile.at(i));

        // create the file and put a header
        file.open(fileNames.at(i), std::ios_base::out);
        if(file.is_open()) {
            std::string str2beReplaced = "{timeUnity}";
            int timeUnityPos = logFileHeaders.at(i).find(str2beReplaced);
            if(timeUnityPos > 0)
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
    robot = new Robot(STD_DEV_NOISE);
}

Simulator::~Simulator() {
    delete robot;
}

void Simulator::simulateFor(double duration, const std::string logFileName, const CmdSignal &cmdSignal) {
    // check if duration is valid
    if(duration <= 0)
        return;

    // start logs
    startLogFiles();

    // simulate
    this->logFileName = logFileName;
    const int nSteps = ceil(duration * Fs);    
    int bufferPos = 0;
    for(int step = 0; step < nSteps; step++) {
        robot->updateRobotStatus(cmdSignal.getCmdNow(Fs * step));
        bufferPos = step % bufferSize;

        if(bufferPos == 0 && step > 0)
            flushAllBuffers();

        bufferize(bufferPos);
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
        this->seconds2timeUnity = 1e-3;
    }
    else if(timeUnity == "us") {
        this->timeUnity = "us";
        this->seconds2timeUnity = 1e-6;
    }
    else if(timeUnity == "ns") {
        this->timeUnity = "ns";
        this->seconds2timeUnity = 1e-9;
    }
    else if(timeUnity == "ps") {
        this->timeUnity = "ps";
        this->seconds2timeUnity = 1e-12;
    }
}

const vecEachSpdType<std::string> Simulator::suffixes4logFile(
    std::vector<spdType>({spdType::RobotEstimation, spdType::RobotTrue, spdType::WheelsRead, spdType::WheelsTrue}),
    std::vector<std::string>({"rEstim.txt",         "rTrue.txt",        "wRead.txt",         "wTrue.txt"})
);

const vecEachSpdType<std::string> Simulator::logFileHeaders(
    std::vector<spdType>({spdType::RobotEstimation, spdType::RobotTrue, spdType::WheelsRead, spdType::WheelsTrue}),
    std::vector<std::string>({("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)")})
);