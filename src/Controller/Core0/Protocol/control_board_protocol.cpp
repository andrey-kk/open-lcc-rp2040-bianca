//
// Created by Magnus Nordlander on 2021-06-27.
// Modified to force V2 Protocol Handshake
//

#include <cstdio>
#include <cmath>
#include "control_board_protocol.h"
#include "utils/polymath.h"
#include "utils/checksum.h"

float high_gain_adc_to_float(uint16_t adcValue) {
    double a = 2.80075E-07;
    double b = -0.000371374;
    double c = 0.272450858;
    double d = -4.737333399;

    return (float)polynomial4(a, b, c, d, adcValue);
}

float low_gain_adc_to_float(uint16_t adcValue) {
    double a = -1.99514E-07;
    double b = 7.66659E-05;
    double c = 0.546325171;
    double d = -17.22637553;

    return (float)polynomial4(a, b, c, d, adcValue);
}

uint16_t float_to_high_gain_adc(float floatValue) {
    double a = -0.000468472;
    double b = 0.097074921;
    double c = 1.6935213;
    double d = 27.8765092;

    return (uint16_t)round(polynomial4(a, b, c, d, floatValue));
}

uint16_t float_to_low_gain_adc(float floatValue) {
    double a = 1.94759E-06;
    double b = -0.000294428;
    double c = 1.812604664;
    double d = 31.49048711;

    return (uint16_t)round(polynomial4(a, b, c, d, floatValue));
}

float ohm_to_high_gain_adc(uint32_t ohm) {
    return 1.567889 + (1018.146 - 1.567889)/(1.f + pow((double)ohm/7181.235, 1.005375));
}

uint32_t high_gain_adc_to_ohm(float floatValue) {
    return 7181.23*pow(-(floatValue-1018.15)/(floatValue-1.56789),(8000.f/8043.f));
}

float ntc_ohm_to_celsius(uint32_t ohm, uint32_t r25, uint32_t b) {
    return (1.f/(log(((float)ohm)/((float)r25))/((float)b)+(1.f/298.15)))-273.15;
}

uint32_t celsius_to_ntc_ohm(float celsius, uint32_t r25, uint32_t b) {
    float kelvin = celsius + 273.15;
    return r25 * exp(b/kelvin - (20.f*(float)b)/5963.f);
}

uint16_t validate_raw_packet(ControlBoardRawPacket packet) {
    uint16_t error = CONTROL_BOARD_VALIDATION_ERROR_NONE;

    if (packet.header != 0x81) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_INVALID_HEADER;
    }

    // Force V2 Validation: This expects a 19-byte V2 packet once the handshake is accepted
    uint8_t calculated_checksum = calculate_checksum(((uint8_t *) &packet + 1), sizeof(packet) - 2, 0x01);
    if (calculated_checksum != packet.checksum) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_INVALID_CHECKSUM;
    }

    auto bbInt = triplet_to_int(packet.brew_boiler_temperature_high_gain);
    auto brew_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bbInt), 50000, 4018);

    auto sbInt = triplet_to_int(packet.service_boiler_temperature_high_gain);
    auto service_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sbInt), 50000, 4018);

    if (brew_boiler_temp > 140) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_BREW_BOILER_TEMP_DANGEROUSLY_HIGH;
    }

    if (service_boiler_temp > 150) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_SERVICE_BOILER_TEMP_DANGEROUSLY_HIGH;
    }

    return error;
}

ControlBoardParsedPacket convert_raw_control_board_packet(ControlBoardRawPacket raw_packet) {
    ControlBoardParsedPacket packet = ControlBoardParsedPacket();
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&raw_packet);

    // Verified: Flags are at Index 10
    uint8_t flags = raw[10];

    // Verified for 9600125: Bit 1 is the Lever. 
    // In your dump, it went high when you moved it.
    packet.brew_switch = (flags & 0x02); 
    
    // Tank logic: Bit 6. 0 is Empty.
    packet.water_tank_empty = ((flags & 0x40) == 0);

    // Verified: Temperatures are shifted. Skip Byte 1 (55) and start at Byte 2.
    uint32_t bb_raw = (raw[2] << 16) | (raw[3] << 8) | raw[4];
    uint32_t sb_raw = (raw[5] << 16) | (raw[6] << 8) | raw[7];

    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sb_raw), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    rawPacket.header = 0x81;

    // V3 units need Bit 0 for Power/Ready
    rawPacket.flags = 0x01; 

    if (parsed_packet.brew_switch) {
        rawPacket.flags |= 0x20; // Engage Pump
        rawPacket.flags |= 0x04; // Open E61 Solenoid Valve
    }

    // Set boiler targets (Triplets)
    rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(parsed_packet.brew_boiler_temperature));
    rawPacket.service_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(parsed_packet.service_boiler_temperature));

    // Force Seed 0x01 for the 9600125 watchdog
    uint8_t* data = reinterpret_cast<uint8_t*>(&rawPacket) + 1;
    rawPacket.checksum = calculate_checksum(data, 16, 0x01); 

    return rawPacket;
}
