/*
* Copyright (c) 2020 Minh Nguyen inc. All rights reserved.
*/
#ifndef _KINOVA_UTIL_H_
#define _KINOVA_UTIL_H_

#include <string>
#include <memory>

#include <kdl_parser/kdl_parser.hpp>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>

#define PI 3.14159265358979323846
#define DEG_TO_RAD(x) (x) * PI / 180.0
#define RAD_TO_DEG(x) (x) * 180.0 / PI
#define ACTUATOR_COUNT 7

namespace k_api = Kinova::Api;

namespace kinova_ctrl
{

class KinovaBaseConnection
{
private:
    std::shared_ptr<k_api::TransportClientTcp> mTransportClientTcp;
    std::shared_ptr<k_api::RouterClient> mRouterTcp;
    std::shared_ptr<k_api::TransportClientUdp> mTransportClientUdp;
    std::shared_ptr<k_api::RouterClient> mRouterUdp;
    std::shared_ptr<k_api::SessionManager> mSessionManagerTcp;
    std::shared_ptr<k_api::SessionManager> mSessionManagerUdp;

public:
    std::shared_ptr<k_api::Base::BaseClient> mBaseClient;
    std::shared_ptr<k_api::BaseCyclic::BaseCyclicClient> mBaseCyclicClient;
    std::shared_ptr<k_api::ActuatorConfig::ActuatorConfigClient> mActuatorConfigClient;

    KinovaBaseConnection(std::string, uint32_t, uint32_t, std::string, std::string);
    ~KinovaBaseConnection();
};

void loadUrdfModel(const std::string &pUrdfPath, KDL::Tree &pKinovaTree, KDL::Chain &pKinovaChain);

void move_to_home_position(k_api::Base::BaseClient* pBase, uint32_t pTimeoutSec = 20);

void handleKinovaException(k_api::KDetailedException& ex);

void stopRobot(k_api::ActuatorConfig::ActuatorConfigClient* actuator_config);

}  // namespace kinova_ctrl

#endif  // _KINOVA_UTIL_H
