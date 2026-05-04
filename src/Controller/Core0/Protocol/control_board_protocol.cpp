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

    static_assert(sizeof(packet) == 18, "Packet size weird");
    
    // Checksum verified against 0x01 seed
    uint8_t calculated_checksum = calculate_checksum(((uint8_t *) &packet + 1), sizeof(packet) - 2, 0x01);
    if (calculated_checksum != packet.checksum) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_INVALID_CHECKSUM;
    }

    // REMOVED the 0xBD flag check here, as V2/V3 flags exceed this mask

    auto brew_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(triplet_to_int(packet.brew_boiler_temperature_high_gain)), 50000, 4000);
    auto service_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(triplet_to_int(packet.service_boiler_temperature_high_gain)), 50000, 4000);

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

    // Standard V2 Protocol Flags are at Index 15
    uint8_t flags = raw[15];

    // Standard V2 Logic (0 = Active/Empty)
    packet.brew_switch = ((flags & 0x02) == 0);
    packet.water_tank_empty = ((flags & 0x40) == 0);

    // Standard V2 Temperature Locations
    uint32_t bb_raw = (raw[2] << 16) | (raw[3] << 8) | raw[4];
    uint32_t sb_raw = (raw[8] << 16) | (raw[9] << 8) | raw[10];

    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sb_raw), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    rawPacket.header = 0x81;

    // V3 units still need Bit 0 for Power/Ready
    rawPacket.flags = 0x01; 

    if (parsed_packet.brew_switch) {
        rawPacket.flags |= 0x20; // Pump ON
        rawPacket.flags |= 0x04; // Solenoid valve OPEN
    }

    uint16_t largeCoffee = float_to_high_gain_adc(parsed_packet.brew_boiler_temperature);
    uint16_t largeService = float_to_high_gain_adc(parsed_packet.service_boiler_temperature);
    rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(largeCoffee);
    rawPacket.service_boiler_temperature_high_gain = int_to_triplet(largeService);

    // V2 Checksum seed (0x01)
    uint8_t* data = reinterpret_cast<uint8_t*>(&rawPacket) + 1;
    rawPacket.checksum = calculate_checksum(data, sizeof(rawPacket) - 2, 0x01); 

    return rawPacket;
}
