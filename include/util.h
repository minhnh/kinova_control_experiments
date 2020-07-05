/**
 * @copyright 2020 Minh Nguyen
 */
#ifndef KINOVA_CTRL_UTIL_H
#define KINOVA_CTRL_UTIL_H

#include <fstream>
#include <map>
#include <vector>
#include <chrono>
#include <libconfig.h++>
#include "abag.h"

namespace sc = std::chrono;

namespace kinova_ctrl
{

bool waitMicroSeconds(const sc::time_point<sc::steady_clock> &pStartTime, const sc::microseconds &pDuration);

void writeDataRow(
    std::ofstream &pFileStream, const abagState_t &pState,
    long pTime, double &pError, double &pCommand, double &pMeasured
);

libconfig::Setting & loadConfigFile(libconfig::Config &cfg, const char* pFileName);

void loadAbagConfig(
    const libconfig::Setting &pConfigRoot, const char* pCtrlApproach, std::map<std::string, std::vector<double>> &
);

void loadKinovaConfig(
    const libconfig::Setting &pConfigRoot, std::vector<double> &pCartForceLimits, std::string &pHostName,
    std::string &pUsername, std::string &pPassword, unsigned int &pPort, unsigned int &pPortRealTime
);

void printConfigurations(const char* pCtrlApproach, const std::string &pHostname, const std::string &pUser,
    const std::string &pPasswd, const unsigned int &pPort, const unsigned int &pPortRT,
    const std::vector<double> &pCartForceLimits, const std::map<std::string, std::vector<double>> &pAbagConfigs);

}  // namespace kinova_ctrl

#endif  // KINOVA_CTRL_UTIL_H
