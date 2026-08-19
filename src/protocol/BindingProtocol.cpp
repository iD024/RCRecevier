// src/protocol/BindingProtocol.cpp
#include "protocol/BindingProtocol.hpp"

namespace RC::Protocol {

bool BindingProtocol::isBindPacket(const Packet& pkt) {
    return pkt.type == static_cast<uint8_t>(PacketType::Bind);
}

void BindingProtocol::extractToConfig(const Packet& pkt, RC::Storage::ReceiverConfig& config) {
    config.boundTransmitterId = pkt.transmitterId;
    
    // In a full frequency-hopping system, the transmitter would send its hop sequence
    // and base channel in the packet payload. For this simplified implementation, we 
    // derive a pseudo-random channel and address from the transmitter ID.
    config.radioChannel = static_cast<uint8_t>((pkt.transmitterId % 80U) + 10U); 
    
    config.radioAddr[0] = static_cast<uint8_t>(pkt.transmitterId & 0xFFU);
    config.radioAddr[1] = static_cast<uint8_t>((pkt.transmitterId >> 8U) & 0xFFU);
    config.radioAddr[2] = static_cast<uint8_t>((pkt.transmitterId >> 16U) & 0xFFU);
    config.radioAddr[3] = static_cast<uint8_t>((pkt.transmitterId >> 24U) & 0xFFU);
    config.radioAddr[4] = 0xE7U; // Fixed 5th byte
}

} // namespace RC::Protocol
