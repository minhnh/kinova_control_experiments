/**
 * @copyright 2020 Minh Nguyen
 */
#include <iostream>
#include "util.h"

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

void loadAbagConfig(const libconfig::Setting &pConfigRoot, const char* pCtrlApproach, std::vector<double> &pAlphas)
{
    try
    {
        std::cout << pCtrlApproach << std::endl;
        const libconfig::Setting &controllers = pConfigRoot["controllers"];
        std::cout << "got controllers" << std::endl;
        const libconfig::Setting &controlParams = controllers[pCtrlApproach];
        std::cout << "got control params" << std::endl;
        const libconfig::Setting &alphaSettings = controlParams.lookup("alpha");

        for (int i = 0; i < alphaSettings.getLength(); i++) {
            pAlphas.push_back(alphaSettings[i]);
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
