#include "some_functions.h"

void progressBar(float progress) {
    const int barWidth = 40;

    if(progress < 0)
        progress = 0;
    
    if(progress < 1) {
        int pos = progress * barWidth;
        std::cout << "[";
        for(int i = 0; i < pos; i++)
            std::cout << "=";
        for(int i = pos; i < barWidth; i++)
            std::cout << " ";
        std::cout << "] " << int(progress * 100) << " %\r";
        std::cout.flush();
    } else {
        std::cout << "Done!";
        int nChars2Delete = barWidth + 2 + 3 + 3 - 5; // bar + '[',']' + 3 numbers + ' ',' ','%' = 2+3+3
        for(int i = 0; i < nChars2Delete; i++)
            std::cout << " ";
        std::cout << std::endl;
    }
}

void logger(const std::string fileName, Eigen::VectorXd* data) {
/*    std::ofstream logFile(fileName);

    if(!logFile.is_open())
        return;
  */
 std::cout << data << std::endl;  
    
}

void logEigenVector(const auto vec, const std::ofstream logFile, const std::string style) {
}
    



