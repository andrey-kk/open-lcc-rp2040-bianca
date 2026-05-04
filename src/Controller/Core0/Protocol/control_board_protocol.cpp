//
// Created by Magnus Nordlander on 2021-06-27.
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

    // DIRECT MEMORY ACCESS: Bypass the struct names
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&packet);
    uint8_t version_byte = raw[1];
    uint8_t flag_byte = raw[10];

    // Detection: V1 always has version 1 and flag 127
    bool is_v1 = (version_byte == 0x01 || flag_byte == 127);

    if (!is_v1) {
        // Only run V2 checksum for V2 machines
        uint8_t calculated_checksum = calculate_checksum(((uint8_t *) &packet + 1), sizeof(packet) - 2, 0x01);
        if (calculated_checksum != packet.checksum) {
            error |= CONTROL_BOARD_VALIDATION_ERROR_INVALID_CHECKSUM;
        }
    }

    // Safety: Use the raw buffer to calculate safety temps to avoid struct padding errors
    // Brew Temp Triplet starts at raw[3], Service at raw[6]
    uint32_t bb_raw = (raw[3] << 16) | (raw[4] << 8) | raw[5];
    auto brew_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4000);

    if (brew_temp > 140) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_BREW_BOILER_TEMP_DANGEROUSLY_HIGH;
    }

    return error;
}

ControlBoardParsedPacket convert_raw_control_board_packet(ControlBoardRawPacket raw_packet) {
    ControlBoardParsedPacket packet = ControlBoardParsedPacket();
    
    // We know flag 127 is IDLE. Let's look at the bits: 0111 1111
    bool is_v1 = (raw_packet.flags == 127 || raw_packet.flags == 125 || raw_packet.flags == 63);

    if (is_v1) {
        // V1 REVISED MAPPING
        // Brew Switch: Bit 1 (value 2). 1=Idle, 0=Brew.
        packet.brew_switch = ((raw_packet.flags & 0x02) == 0);
        
        // Water Tank: In V1, this is often Bit 6 (0x40) OR Bit 0 (0x01).
        // Let's check both. If either bit is 0, we treat it as empty for safety.
        if (((raw_packet.flags & 0x40) == 0) || ((raw_packet.flags & 0x01) == 0)) {
            packet.water_tank_empty = true;
        } else {
            packet.water_tank_empty = false;
        }

        // Keep this false until we find the real refill bit to prevent startup pumping.
        packet.service_boiler_low = false; 
    } else {
        // Standard V2 Logic
        packet.brew_switch = raw_packet.flags & 0x02;
        packet.water_tank_empty = raw_packet.flags & 0x40;
        packet.service_boiler_low = triplet_to_int(raw_packet.service_boiler_level) > 256;
    }

    // Temperature Math (Verified working in your logs!)
    auto bbInt = triplet_to_int(raw_packet.brew_boiler_temperature_high_gain);
    auto sbInt = triplet_to_int(raw_packet.service_boiler_temperature_high_gain);
    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bbInt), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sbInt), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    rawPacket.header = 0x81;

    rawPacket.flags = 0x0;
    if (parsed_packet.water_tank_empty) {
        rawPacket.flags |= 0x40;
    }
    if (parsed_packet.brew_switch) {
        rawPacket.flags |= 0x02;
    }

    /* @fixme This needs to use the new NTC calculation. We just need the numbers for high-to-low gain */

    uint16_t smallCoffee = float_to_low_gain_adc(parsed_packet.brew_boiler_temperature);
    uint16_t smallService = float_to_low_gain_adc(parsed_packet.service_boiler_temperature);
    uint16_t largeCoffee = float_to_high_gain_adc(parsed_packet.brew_boiler_temperature);
    uint16_t largeService = float_to_high_gain_adc(parsed_packet.service_boiler_temperature);

    rawPacket.brew_boiler_temperature_low_gain = int_to_triplet(smallCoffee);
    rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(largeCoffee);
    rawPacket.service_boiler_temperature_low_gain = int_to_triplet(smallService);
    rawPacket.service_boiler_temperature_high_gain = int_to_triplet(largeService);

    rawPacket.service_boiler_level = int_to_triplet(parsed_packet.service_boiler_low ? 650 : 90);
    rawPacket.checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&rawPacket + 1), sizeof(rawPacket) - 2, 0x01);

    return rawPacket;
}
