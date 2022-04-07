/**
 * @file neo7gps.h
 * 
 * @author Joseph Telaak
 * 
 * @brief Interface to the NEO 7 GPS
 * 
 * @version 0.1
 * 
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// --------- Lib
#include <Arduino.h>
#include <SoftwareSerial.h>

#define DEF_RX_PIN 2
#define DEF_TX_PIN 3

class NEO7GPS {

    /**
     * @brief Navigation Modes
     * 
     */

    enum NAV_MODE : byte {
        NAV_PEDESTRIAN = 0x03,
        NAV_AUTOMOTIVE = 0x04,
        NAV_SEA = 0x05,
        NAV_AIRBORNE = 0x06

    };

    /**
     * @brief GPS Polling Rates
     * 
     */

    enum DATA_RATE : byte[2] {
        GPS_1Hz = {0xE8, 0x03},
        GPS_2Hz = {0xF4, 0x01},
        GPS_3Hz = {0x2C, 0x01},
        GPS_4Hz = {0xFA, 0x00}

    };

    /**
     * @brief Baud rates (UNOR3: 19200, MEGA2560: 38400)
     * 
     */

    enum BAUD_RATE : byte[3] {
        GPS_BAUD_4800 = {0xC0, 0x12, 0x00},
        GPS_BAUD_9600 = {0x80, 0x25, 0x00},
        GPS_BAUD_19200 = {0x00, 0x4B, 0x00},  /*SOFTWARESERIAL LIMIT FOR ARDUINO UNO R3!*/
        GPS_BAUD_38400 = {0x00, 0x96, 0x00},  /*SOFTWARESERIAL LIMIT FOR ARDUINO MEGA 2560!*/
        GPS_BAUD_57600 = {0x00, 0xE1, 0x00},
        GPS_BAUD_115200 = {0x00, 0xC2, 0x01},
        GPS_BAUD_230400 = {0x00, 0x84, 0x03},

    };

    /**
     * @brief Message status
     * 
     */

    enum NMEA_MESSAGE : uint8_t {
        NMEA_OFF = 0x00,
        NMEA_ON = 0x01

    };

    public:
        NEO7GPS(const int RX = DEF_RX_PIN, const int TX = DEF_TX_PIN, 
                const BAUD_RATE BAUD = GPS_BAUD_9600, const NAV_MODE Mode = NAV_AUTOMOTIVE, 
                const DATA_RATE Data_Rate= GPS_4Hz,
                const NMEA_MESSAGE GLL_Sentence = NMEA_OFF, const NMEA_MESSAGE GSA_Sentence = NMEA_OFF, 
                const NMEA_MESSAGE GSV_Sentence = NMEA_OFF, const NMEA_MESSAGE RMC_Sentence = NMEA_OFF,
                const NMEA_MESSAGE VTG_Sentence = NMEA_OFF);

        void setup();
        SoftwareSerial* getSerial();

    private:
        SoftwareSerial* gps_serial;

        void _configure(byte *settings);
        void _beginSerial(byte setting, const int RX, const int TX);
        void _checksum(byte *checksum_payload, byte payload_size);
        void _sendUBX(byte *UBX_msg, byte msg_length);
        void _getUBX_ACK(byte *msg_ID);

}