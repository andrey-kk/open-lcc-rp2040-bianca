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

    uint8_t flags = raw[1];

    // INCOMING: Verify Lever (0x02) and Tank (0x40)
    packet.brew_switch = ((flags & 0x02) != 0);
    
    // Magnus: Bit 0x40 is 'ON if 0'. If raw is 64 (0x40), the bit is 1, meaning NOT empty.
    // If you lift the tank and wait 5 seconds, this should drop to 0.
    packet.water_tank_empty = ((flags & 0x40) == 0); 

    // TEMPERATURE READS: Use proper triplet conversion to stop -121.4 C error
    uint32_t bb_raw = triplet_to_int(raw_packet.brew_boiler_temperature_high_gain);
    uint32_t sb_raw = triplet_to_int(raw_packet.service_boiler_temperature_high_gain);

    packet.brew_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(bb_raw), 50000, 4018);
    packet.service_boiler_temperature = ntc_ohm_to_celsius(high_gain_adc_to_ohm(sb_raw), 50000, 4018);

    return packet;
}

ControlBoardRawPacket convert_parsed_control_board_packet(ControlBoardParsedPacket parsed_packet) {
    ControlBoardRawPacket rawPacket = ControlBoardRawPacket();
    uint8_t* raw = reinterpret_cast<uint8_t*>(&rawPacket);
    for(int i=0; i<18; i++) raw[i] = 0;

    rawPacket.header = 0x81;
    raw[1] = 0x01; // V3 Handshake

    if (parsed_packet.brew_switch) {
        raw[1] |= 0x20; // Pump command
        raw[1] |= 0x04; // Solenoid command
    }

    // Set targets (fallback to safe defaults if undefined)
    float t_brew = (parsed_packet.brew_boiler_temperature > 20) ? parsed_packet.brew_boiler_temperature : 95.0f;
    float t_steam = (parsed_packet.service_boiler_temperature > 20) ? parsed_packet.service_boiler_temperature : 120.0f;

    // OUTGOING: Satisfy the Gicar by sending BOTH high and low gain ADCs
    rawPacket.brew_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(t_brew));
    rawPacket.brew_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(t_brew));

    rawPacket.service_boiler_temperature_high_gain = int_to_triplet(float_to_high_gain_adc(t_steam));
    rawPacket.service_boiler_temperature_low_gain = int_to_triplet(float_to_low_gain_adc(t_steam));

    rawPacket.checksum = calculate_checksum(raw + 1, 16, 0x01);
    return rawPacket;
}
