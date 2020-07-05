/**
 * @copyright 2020 Minh Nguyen
 */
#include <iostream>
#include <sstream>
#include <string.h>
#include "util.h"
#include "constants.hpp"

namespace kc_const = kinova_ctrl::constants;

namespace kinova_ctrl
{

bool waitMicroSeconds(const sc::time_point<sc::steady_clock> &pStartTime, const sc::microseconds &pDuration)
{
    auto now = sc::steady_clock::now();
    if (now - pStartTime > pDuration) return false;

    do {
        now = sc::steady_clock::now();
    } while (now - pStartTime < pDuration);
    return true;
}

void writeDataRow(
    std::ofstream &pFileStream, const abagState_t &pState,
    long pTime, double &pError, double &pCommand, double &pMeasured
) {
    pFileStream << pTime << ","
                << pError << ","
                << pState.signedErr_access << ","
                << pState.bias_access << ","
                << pState.gain_access << ","
                << pState.eBar_access << ","
                << pCommand << ","
                << pMeasured << std::endl;
}

libconfig::Setting & loadConfigFile(libconfig::Config &pCfg, const char* pFileName)
{
    try
    {
        pCfg.readFile(pFileName);
    }
    catch (const libconfig::FileIOException &fioex)
    {
        std::cerr << "FileIOException reading configuration file: " << pFileName << std::endl;
        throw;
    }
    catch(const libconfig::ParseException &pex)
    {
        std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
                  << " - " << pex.getError() << std::endl;
        throw;
    }

    return pCfg.getRoot();
}

void loadAbagConfig(const libconfig::Setting &pConfigRoot, const char* pCtrlApproach,
    std::map<std::string, std::vector<double>> &pAbagConfigs
) {
    std::stringstream errMsgStream;
    unsigned int expectedParamNum = 0;
    if      (strcmp(pCtrlApproach, "position_velocity") == 0)   expectedParamNum = 6;
    else if (strcmp(pCtrlApproach, "position") == 0)            expectedParamNum = 3;
    else if (strcmp(pCtrlApproach, "velocity") == 0)            expectedParamNum = 3;
    else {
        errMsgStream << "loadAbagConfig: unexpected control approach: " << pCtrlApproach;
        throw std::runtime_error(errMsgStream.str());
    }

    try
    {
        const libconfig::Setting &controlParams = pConfigRoot["controllers"][pCtrlApproach];

        for (auto configName : kc_const::config::ABAG_CONFIG_NAMES) {
            std::vector<double> configValues;
            const libconfig::Setting &settings = controlParams.lookup(configName);
            for (int i = 0; i < settings.getLength(); i++) {
                configValues.push_back(settings[i]);
            }
            if (configValues.size() != expectedParamNum) {
                errMsgStream << "loadAbagConfig: expected " << expectedParamNum
                            << " params for alpha, found " << configValues.size();
                throw std::runtime_error(errMsgStream.str());
            }
            pAbagConfigs[configName] = configValues;
        }
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "setting not found: " << nfex.getPath() << std::endl;
        throw;
    }
    catch(libconfig::SettingTypeException &typeEx)
    {
        std::cerr << "type exception: " << typeEx.what() << std::endl;
        throw;
    }
}

void loadKinovaConfig(const libconfig::Setting &pConfigRoot,
    std::vector<double> &pCartForceLimits, std::string &pHostName, std::string &pUsername, std::string &pPassword,
    unsigned int &pPort, unsigned int &pPortRealTime
) {
    try
    {
        std::stringstream errMsgStream;
        const libconfig::Setting &robotSettings = pConfigRoot["robot"];

        const libconfig::Setting &cartForceSettings = robotSettings[kc_const::config::CART_FORCE_LIMIT.c_str()];
        for (int i = 0; i < cartForceSettings.getLength(); i++) {
            pCartForceLimits.push_back(cartForceSettings[i]);
        }
        if (pCartForceLimits.size() != 6) {
            errMsgStream << "loadKinovaConfig: expected 6 params for cartesian force limite, found "
                         << pCartForceLimits.size();
            throw std::runtime_error(errMsgStream.str());
        }

        if (!robotSettings.lookupValue(kc_const::config::HOSTNAME, pHostName)) {
            errMsgStream << "loadKinovaConfig: failed to load hostname config";
            throw std::runtime_error(errMsgStream.str());
        }

        if (!robotSettings.lookupValue(kc_const::config::USER, pUsername)) {
            errMsgStream << "loadKinovaConfig: failed to load username config";
            throw std::runtime_error(errMsgStream.str());
        }

        if (!robotSettings.lookupValue(kc_const::config::PASSWD, pPassword)) {
            errMsgStream << "loadKinovaConfig: failed to load password config";
            throw std::runtime_error(errMsgStream.str());
        }

        if (!robotSettings.lookupValue(kc_const::config::PORT, pPort)) {
            errMsgStream << "loadKinovaConfig: failed to load port config";
            throw std::runtime_error(errMsgStream.str());
        }

        if (!robotSettings.lookupValue(kc_const::config::PORT_RT, pPortRealTime)) {
            errMsgStream << "loadKinovaConfig: failed to load real-time port config";
            throw std::runtime_error(errMsgStream.str());
        }
    }
    catch(const libconfig::SettingNotFoundException &nfex)
    {
        std::cerr << "setting not found: " << nfex.getPath() << std::endl;
        throw;
    }
    catch(libconfig::SettingTypeException &typeEx)
    {
        std::cerr << "type exception: " << typeEx.what() << std::endl;
        throw;
    }
}

}  // namespace kinova_ctrl
