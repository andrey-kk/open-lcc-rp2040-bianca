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

    uint8_t calculated_checksum = calculate_checksum(((uint8_t *) &packet + 1), sizeof(packet) - 2, 0x01);
    if (calculated_checksum != packet.checksum) {
        error |= CONTROL_BOARD_VALIDATION_ERROR_INVALID_CHECKSUM;
    }

    auto brew_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(triplet_to_int(packet.brew_boiler_temperature_high_gain)), 50000, 4000);
    auto service_boiler_temp = ntc_ohm_to_celsius(high_gain_adc_to_ohm(triplet_to_int(packet.service_boiler_temperature_high_gain)), 50000, 4000);

    // Keep the safety checks active
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

    // Keep the lever working so the machine stays happy
    packet.brew_switch = ((raw[1] & 0x02) != 0);
    
    // Force tank full so the RP2040 doesn't panic and block the pump
    packet.water_tank_empty = false; 

    // ULTIMATE DIAGNOSTIC: Watch the raw data in real-time
    // Coffee Temp will display the exact decimal value of Byte 1
    packet.brew_boiler_temperature = (float)raw[1];
    
    // Service Temp will display the exact decimal value of Byte 15
    packet.service_boiler_temperature = (float)raw[15];

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    uint8_t* raw = reinterpret_cast<uint8_t*>(&rawPacket);

    // Clear memory
    for(int i=0; i<18; i++) raw[i] = 0;

    rawPacket.header = 0x81;
    raw[1] = 0x01; // Required V3 Keep-Alive

    // 3. OUTGOING COMMANDS (Pump works perfectly on Byte 15)
    if (parsed_packet.brew_switch) {
        raw[15] |= 0x20; // Pump ON
        raw[15] |= 0x04; // Solenoid OPEN
    }

    // 4. REAL TARGETS (Allow Home Assistant Eco Mode to work)
    float t_brew = parsed_packet.brew_boiler_temperature;
    float t_steam = parsed_packet.service_boiler_temperature;

    // Send both High and Low gain ADCs to satisfy the Gicar's hardware safety checks
    rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(t_brew));
    rawPacket.brew_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(t_brew));

    rawPacket.service_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(t_steam));
    rawPacket.service_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(t_steam));

    rawPacket.checksum = calculate_checksum(raw + 1, 16, 0x01);

    return rawPacket;
}
