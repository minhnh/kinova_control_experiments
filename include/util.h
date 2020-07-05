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

}  // namespace kinova_ctrl

#endif  // KINOVA_CTRL_UTIL_H
