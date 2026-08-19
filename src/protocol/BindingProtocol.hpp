// src/protocol/BindingProtocol.hpp
#pragma once
#include "protocol/Packet.hpp"
#include "storage/ConfigStore.hpp"

namespace RC::Protocol {

class BindingProtocol {
public:
    /// Check if a packet is a valid bind packet
    static bool isBindPacket(const Packet& pkt);

    /// Extract bind data and update the config struct.
    /// In a full implementation, channel hopping sequences and addresses 
    /// would be passed in the packet payload.
    static void extractToConfig(const Packet& pkt, RC::Storage::ReceiverConfig& config);
};

} // namespace RC::Protocol
