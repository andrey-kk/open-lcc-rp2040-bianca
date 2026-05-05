//
// Created by Magnus Nordlander on 2021-06-27.
// Revised for Bianca V3 Power Modes
//Edited by Andrey K. on 2026-05-05
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

    // 1. INCOMING SWITCHES (Perfectly verified)
    uint8_t flags = raw[1];
    packet.brew_switch = ((flags & 0x02) != 0);
    
    // Tank is empty when 0x40 is HIGH (Proven by Timer Hijack)
    packet.water_tank_empty = ((flags & 0x40) != 0); 

    // 2. REAL TEMPERATURES (Perfectly verified)
    uint32_t bb_raw = triplet_to_int(raw_packet.brew_boiler_temperature_high_gain);
    uint32_t sb_raw = triplet_to_int(raw_packet.service_boiler_temperature_high_gain);

    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sb_raw), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    uint8_t* raw = reinterpret_cast<uint8_t*>(&rawPacket);

    // Initialize the entire packet memory to 0
    for(int i=0; i<18; i++) raw[i] = 0;

    // Set Header
    rawPacket.header = 0x81;

    // 1. FLAGS (The Pump/Lever Logic)
    // We must ensure the brew_switch flag is set so the Gicar knows the lever is up
    if (parsed_packet.brew_switch) {
        rawPacket.flags |= 0x02;
    }
    // Mirror the tank state back to the Gicar
    if (parsed_packet.water_tank_empty) {
        rawPacket.flags |= 0x40;
    }

    // 2. HEATER CONTROL (Zero Bypass)
    // If Standby is ON or Tank is Empty, we send hard zeros to kill the heaters.
    if (parsed_packet.standby_mode || parsed_packet.water_tank_empty) {
        rawPacket.brew_boiler_temperature_low_gain = int_to_triplet(0);
        rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(0);
        rawPacket.service_boiler_temperature_low_gain = int_to_triplet(0);
        rawPacket.service_boiler_temperature_high_gain = int_to_triplet(0);
    } else {
        // Normal Brew Boiler Heating
        rawPacket.brew_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(parsed_packet.brew_boiler_temperature));
        rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(parsed_packet.brew_boiler_temperature));

        // Service Boiler Eco/Sleep Logic
        if (parsed_packet.eco_mode || parsed_packet.sleep_mode) {
            rawPacket.service_boiler_temperature_low_gain = int_to_triplet(0);
            rawPacket.service_boiler_temperature_high_gain = int_to_triplet(0);
        } else {
            rawPacket.service_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(parsed_packet.service_boiler_temperature));
            rawPacket.service_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(parsed_packet.service_boiler_temperature));
        }
    }

    // 3. SERVICE BOILER LEVEL
    rawPacket.service_boiler_level = int_to_triplet(parsed_packet.service_boiler_low ? 650 : 90);

    // 4. FIXED CHECKSUM
    // The checksum starts at byte 1 (flags) and ends at byte 16.
    // rawPacket.checksum is byte 17.
    rawPacket.checksum = calculate_checksum(raw + 1, 16, 0x01);

    return rawPacket;
}
