#include "communication.h"
#include "dds/dds_communicator.h"

#include <stdexcept>
#include <iostream>

namespace leju {

ICommunication& Communication::getInstance() {
    // Leaky singleton: intentionally never deleted to avoid static destruction
    // order issues when CycloneDDS is statically linked. The OS will reclaim
    // all memory when the process exits.
    static DdsCommunicator* instance = new DdsCommunicator();
    return *instance;
}

} // namespace leju