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

    std::ostringstream folder;
    folder << "results/";
    createFolder(folder.str());
    if(filterType == "info") 
        #ifdef FILTER_V_2_0
        folder << "IFv2/";
        #else
        folder << "IFv1/";
        #endif
    else if(filterType == "kalman")
        #ifdef FILTER_V_2_0
        folder << "KFv2/";
        #else
        folder << "KFv1/";
        #endif
    else
        folder << "noFilter/";
    createFolder(folder.str());

    return folder.str() + date.str() + "-" + hour.str();
}

void Simulator::createFolder(const std::string &folderName) {
    const fs::path folderPath{folderName};
    if (!fs::exists(folderPath))
        fs::create_directory(folderPath);
}

void Simulator::bufferize(int bufferPos, double timeNow) {
    timeBuffer[bufferPos] = timeNow;
    robotEstimSpd_Buffer[bufferPos] = robot->estimateRobotsSpd();
    robotTrueSpd_Buffer[bufferPos] = robot->getRobotsTrueSpd();
    wheelsEstimSpd_Buffer[bufferPos] = robot->getEstimatedWheelsSpd();
    wheelsTrueSpd_Buffer[bufferPos] = robot->getWheelsTrueSpd();
    wheelsReadSpd_Buffer[bufferPos] = robot->getEncodersRead() / N;
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
    printBufferOnFile(fileNames.at(SpdType::RobotEstimation), robotEstimSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::WheelsTrue), wheelsTrueSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::WheelsEstimation), wheelsEstimSpd_Buffer, printUntil);
    printBufferOnFile(fileNames.at(SpdType::WheelsReadByEncoders), wheelsReadSpd_Buffer, printUntil);
}

void Simulator::printBufferOnFile(const std::string &fileName, const std::vector<Eigen::Vector4d> &buffer, int printUntil) {
    std::ofstream file(fileName, std::ios_base::app);
    if(file.is_open()) {
        for(int i = 0; i <= printUntil; i++)
            file << timeBuffer.at(i) * seconds2timeUnity << "\t" << buffer.at(i).format(tableFormatTAB) << std::endl;
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
        fileNames.putAt(i, preffix4logFile + suffixes4logFile.at(i));

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

void Simulator::updatePerfMeasures() {
    perfMeasurers->setTotalSpentTime(robot->getTotalSpentTimeFiltering());
    perfMeasurers->putRobotEstim(robot->estimateRobotsSpd(), robot->getRobotsTrueSpd());
    perfMeasurers->putWheelsEstim(robot->getEstimatedWheelsSpd(), robot->getWheelsTrueSpd());
}

void Simulator::printPerfMeasuresOnFile() const {
    const std::string fileName = preffix4logFile + "perfMeas.txt";
    std::ofstream file(fileName, std::ios_base::out);
    if(file.is_open()) {
        file << "Time spent on filter (s) = " << perfMeasurers->getTotalSpentTime() << std::endl
            << "Robot's MSE = " << perfMeasurers->getMSE_robot().transpose() << std::endl
            << "Wheels' MSE = " << perfMeasurers->getMSE_wheels().transpose() << std::endl;
        file.close();
    }
}

Simulator::Simulator() : robotTrueSpd_Buffer(bufferSize), robotEstimSpd_Buffer(bufferSize),
                         wheelsTrueSpd_Buffer(bufferSize), wheelsEstimSpd_Buffer(bufferSize),
                         wheelsReadSpd_Buffer(bufferSize), timeBuffer(bufferSize),
                         tableFormatTAB(Eigen::StreamPrecision, 0, " ", "\t") {
    robot = new Robot(filterType, STD_DEV_NOISE_MOV);
    perfMeasurers = new PerformanceMeasurers();
}

Simulator::~Simulator() {
    delete robot;
}

void Simulator::simulateFor(double duration, const std::string preffix4logFile, const CmdSignal &cmdSignal) {
    // check if duration is valid
    if(duration <= 0)
        return;

    // start logs
    this->preffix4logFile = preffix4logFile;
    startLogFiles();

    // start performance measures
    perfMeasurers->reset();

    #ifdef DBG_MODE
        #ifdef FILTER_V_2_0
        debugBuffer << "## FILTER V2";
        #else
        debugBuffer << "## FILTER V1";
        #endif
    #endif

    // simulate
    const int nSteps = ceil(duration * Fs);
    const int updateBarPeriod_steps = ((nSteps > 100) ? (nSteps / 100) : 1);
    float progress = 0;
    int lastBarUpdate_step = 0;
    int bufferPos = 0;
    for(int step = 0; step < nSteps; step++) {
        #ifdef DBG_MODE
        updateDebugLog();
        debugBuffer << std::endl << "# STEP = " << step << std::endl;
        #endif
        if(step == 0)
            progressBar(0);
        else if(step - lastBarUpdate_step >= updateBarPeriod_steps) {
            progress = step * 1.0 / nSteps;
            progressBar(progress);
            lastBarUpdate_step = step;
        }
        
        robot->updateRobotStatus(cmdSignal.getCmdNow(Ts * step));
        bufferPos = step % bufferSize;
        if(bufferPos == 0 && step > 0)
            flushAllBuffers();
        bufferize(bufferPos, Ts * step);

        updatePerfMeasures();
    }
    progressBar(1);
    std::cout << std::endl;

    // flush the rest of the buffer
    flushAllBuffers(bufferPos);

    // print performance measurers
    printPerfMeasuresOnFile();

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
    std::vector<SpdType>({SpdType::RobotEstimation, SpdType::RobotTrue, SpdType::WheelsEstimation, 
                          SpdType::WheelsTrue, SpdType::WheelsReadByEncoders}),
    std::vector<std::string>({"rEstim.txt", "rTrue.txt", "wEstim.txt", "wTrue.txt", "wRead.txt"})
);

const vecEachSpdType<std::string> Simulator::logFileHeaders(
    std::vector<SpdType>({SpdType::RobotEstimation, SpdType::RobotTrue, SpdType::WheelsEstimation, 
                          SpdType::WheelsTrue, SpdType::WheelsReadByEncoders}),
    std::vector<std::string>({("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tvx (m/s)\tvy (m/s)\tomega (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)"),
                              ("t ({timeUnity})\tomega1 (rad/s)\tomega2 (rads)\tomega3 (rad/s)\tomega4 (rad/s)")})
);