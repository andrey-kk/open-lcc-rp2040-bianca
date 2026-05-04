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

    // Because the unit is speaking V1 Dialect at Index 10:
    uint8_t flags = raw[10];

    // V3 Gicar in V1 mode usually needs these bits flipped:
    // Brew Lever (Bit 1): If it's 0, it's ACTIVE (Lever up)
    packet.brew_switch = ((flags & 0x02) == 0);
    
    // Tank (Bit 6): If it's 0, it's EMPTY
    packet.water_tank_empty = ((flags & 0x40) == 0);

    // Re-align Temperatures (V1 shift)
    uint32_t bb_raw = (raw[3] << 16) | (raw[4] << 8) | raw[5];
    uint32_t sb_raw = (raw[6] << 16) | (raw[7] << 8) | raw[8];

    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sb_raw), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    rawPacket.header = 0x81;

    // V3 units often won't pump unless Bit 0 (Power LED) is ON
    rawPacket.flags = 0x01; 

    if (parsed_packet.brew_switch) {
        rawPacket.flags |= 0x20; // Pump Bit
    }
    
    // ... (Keep the rest of your temperature triplet logic here) ...

    // CRITICAL: Calculate checksum with 0x01 seed to keep the heaters alive
    uint8_t* data = reinterpret_cast<uint8_t*>(&rawPacket) + 1;
    rawPacket.checksum = calculate_checksum(data, sizeof(rawPacket) - 2, 0x01);

    return rawPacket;
}
