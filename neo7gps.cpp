/**
 * @file neo7gps.cpp
 * 
 * @author Joseph Telaak
 * 
 * @brief Access to the NEO 7 GPS Module
 * 
 * @version 0.1
 * 
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "neo7gps.h"

byte* settings;

/**
 * @brief Construct a new NEO7GPS::NEO7GPS object
 * 
 * @param RX RX Pin for software serial
 * @param TX TX Pin for software serial
 * @param BAUD Baud rate
 * @param Mode GPS Mode
 * @param Data_Rate GPS Data Rate
 * @param GLL_Sentence Sentence
 * @param GSA_Sentence Sentence
 * @param GSV_Sentence Sentence
 * @param RMC_Sentence Sentence
 * @param VTG_Sentence Sentence
 */

NEO7GPS::NEO7GPS(const int RX, const int TX, 
        const BAUD_RATE BAUD, const NAV_MODE Mode, const DATA_RATE Data_Rate,
        const NMEA_MESSAGE GLL_Sentence, const NMEA_MESSAGE GSA_Sentence, 
        const NMEA_MESSAGE GSV_Sentence, const NMEA_MESSAGE RMC_Sentence,
        const NMEA_MESSAGE VTG_Sentence) {

    #ifdef DEBUG
        Serial.begin(115200);
    #endif

    settings = {Mode, Data_Rate[0], Data_Rate[1], BAUD[0], BAUD[1], BAUD[2], BAUD[3], GLL_Sentence, GSA_Sentence, GSV_Sentence, RMC_Sentence, VTG_Sentence};

}

/**
 * @brief Setup GPS
 * 
 */

void NEO7GPS::setup() {
    _beginSerial(BAUD[1], RX, TX);
    _configure(settings);

}

/**
 * @brief Return the serial pointer
 * 
 * @return SoftwareSerial* 
 */

SoftwareSerial* NEO7GPS::getSerial() {
    return gps_serial;

}

/**
 * @brief Start the software serial communication to the module
 * 
 * @param setting Baud setting byte
 * @param RX RX pin
 * @param TX TX pin
 */

NEO7GPS::_beginSerial(BAUD_RATE setting, const int RX, const int TX) {
    gps_serial = new SoftwareSerial(RX, TX);

    switch (setting[1]) {
        case 0x12:
            gps_serial.begin(4800);
            break;

        case 0x4B:
            gps_serial.begin(19200);
            break;

        case 0x96:
            gps_serial.begin(38400);
            break;

        case 0xE1:
            gps_serial.begin(57600);
            break;

        case 0xC2:
            gps_serial.begin(115200);
            break;

        case 0x84:
            gps_serial.begin(230400);
            break;

        default:
            break;

    }
}

/**
 * @brief COnfigure the module based on the settings
 * 
 * @param settings Settings byte array
 */

void NEO7GPS::_configure(byte *settings) {
    byte gps_set_success = 0;
    
    #ifdef DEBUG
        Serial.println("Configuring u-Blox GPS initial state...");
    #endif

    // Generate the configuration string for Navigation Mode
    byte set_nav[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, *settings, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    _checksum(&set_nav[2], sizeof(set_nav) - 4);

    // Generate the configuration string for Data Rate
    byte set_data_rate[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, settings[1], settings[2], 0x01, 0x00, 0x01, 0x00, 0x00, 0x00};
    _checksum(&set_data_rate[2], sizeof(set_data_rate) - 4);

    // Generate the configuration string for Baud Rate
    byte set_port_rate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, settings[3], settings[4], settings[5], 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    _checksum(&set_port_rate[2], sizeof(set_port_rate) - 4);

    byte setGLL[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
    byte setGSA[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
    byte setGSV[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
    byte setRMC[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
    byte setVTG[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

    delay(2500);

    while(gps_set_success < 3) {
        #ifdef DEBUG
            Serial.print("Setting Navigation Mode... ");
        #endif
        
        _sendUBS(&set_nav[0], sizeof(set_nav));  //Send UBX Packet
        gps_set_success += _getUBX_ACK(&set_nav[2]); //Passes Class ID and Message ID to the ACK Receive function
        
        if (gps_set_success == 5) {
            gps_set_success -= 4;
            setBaud(settings[4]);

            delay(1500);
            
            byte lower_port_rate[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA2, 0xB5};
            _sendUBS(lower_port_rate, sizeof(lower_port_rate));
            
            gps_serial.begin(9600);
            delay(2000);

        }

        if (gps_set_success == 6) gps_set_success -= 4;
        if (gps_set_success == 10) gps_status[0] = true;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("Navigation mode configuration failed.");
    #endif
    
    gps_set_success = 0;

    while(gps_set_success < 3) {
        #ifdef DEBUG
            Serial.print("Setting Data Update Rate... ");
        #endif
        
        _sendUBS(&set_data_rate[0], sizeof(set_data_rate));  //Send UBX Packet
        gps_set_success += _getUBX_ACK(&set_data_rate[2]); //Passes Class ID and Message ID to the ACK Receive function
        
        if (gps_set_success == 10) gps_status[1] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;
    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("Data update mode configuration failed.");
    #endif

    gps_set_success = 0;

    while(gps_set_success < 3 && settings[6] == 0x00) {
        #ifdef DEBUG
            Serial.print("Deactivating NMEA GLL Messages ");
        #endif
        
        _sendUBS(setGLL, sizeof(setGLL));
        gps_set_success += _getUBX_ACK(&setGLL[2]);
        
        if (gps_set_success == 10) gps_status[2] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("NMEA GLL Message Deactivation Failed!");
    #endif

    gps_set_success = 0;

    while(gps_set_success < 3 && settings[7] == 0x00) {
        #ifdef DEBUG
            Serial.print("Deactivating NMEA GSA Messages ");
        #endif
        
        _sendUBS(setGSA, sizeof(setGSA));
        gps_set_success += _getUBX_ACK(&setGSA[2]);
        
        if (gps_set_success == 10) gps_status[3] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("NMEA GSA Message Deactivation Failed!");
    #endif

    gps_set_success = 0;

    while(gps_set_success < 3 && settings[8] == 0x00) {
        #ifdef DEBUG
            Serial.print("Deactivating NMEA GSV Messages ");
        #endif
        
        _sendUBS(setGSV, sizeof(setGSV));
        gps_set_success += _getUBX_ACK(&setGSV[2]);
        
        if (gps_set_success == 10) gps_status[4] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("NMEA GSV Message Deactivation Failed!");
    #endif

    gps_set_success = 0;

    while(gps_set_success < 3 && settings[9] == 0x00) {
        #ifdef DEBUG
            Serial.print("Deactivating NMEA RMC Messages ");
        #endif
        
        _sendUBS(setRMC, sizeof(setRMC));
        gps_set_success += _getUBX_ACK(&setRMC[2]);
        
        if (gps_set_success == 10) gps_status[5] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("NMEA RMC Message Deactivation Failed!");
    #endif

    gps_set_success = 0;

    while(gps_set_success < 3 && settings[10] == 0x00) {
        #ifdef DEBUG
            Serial.print("Deactivating NMEA VTG Messages ");
        #endif
        
        _sendUBS(setVTG, sizeof(setVTG));
        gps_set_success += _getUBX_ACK(&setVTG[2]);
        
        if (gps_set_success == 10) gps_status[6] = true;
        if (gps_set_success == 5 | gps_set_success == 6) gps_set_success -= 4;

    }

    #ifdef DEBUG
        if (gps_set_success == 3) Serial.println("NMEA VTG Message Deactivation Failed!");
    #endif

    gps_set_success = 0;
    if (settings[4] != 0x25) {
        #ifdef DEBUG
            Serial.print("Setting Port Baud Rate... ");
        #endif

        _sendUBS(&set_port_rate[0], sizeof(set_port_rate));
        setBaud(settings[4]);

        #ifdef DEBUG
            Serial.println("Success!");
        #endif

        delay(500);

    }
}

/**
 * @brief 
 * 
 * @param checksum_payload 
 * @param payload_size 
 */

void NEO7GPS::_checksum(byte *checksum_payload, byte payload_size) {
    byte CK_A = 0;
    byte CK_B = 0;

    for (int i = 0; i < payload_size ;i++) {
        CK_A = CK_A + *checksum_payload;
        CK_B = CK_B + CK_A;
        checksum_payload++;

    }

    *checksum_payload = CK_A;
    checksum_payload++;
    *checksum_payload = CK_B;

}

/**
 * @brief 
 * 
 * @param UBX_msg 
 * @param msg_length 
 */

void NEO7GPS::__sendUBS(byte *UBX_msg, byte msg_length) {
    for(int i = 0; i < msg_length; i++) {
        gps_serial.write(UBX_msg[i]);
        gps_serial.flush();

    }

    gps_serial.println();
    gps_serial.flush();
}

/**
 * @brief 
 * 
 * @param msg_ID 
 * @return byte 
 */

byte NEO7GPS::__getUBX_ACK(byte *msg_ID) {
    byte CK_A = 0;
    byte CK_B = 0;
    byte incoming_char;

    bool header_received = false;
    unsigned long ack_wait = millis();

    byte ack_packet[10] = {0xB5, 0x62, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    int i = 0;

    while (1) {
        if (gps_serial.available()) {
            incoming_char = gps_serial.read();

            if (incoming_char == ack_packet[i]) {
                i++;

            } else if (i > 2) {
                ack_packet[i] = incoming_char;
                i++;
            }
        }
        
        if (i > 9) break;

        if ((millis() - ack_wait) > 1500) {
            #ifdef DEBUG
                Serial.println("ACK Timeout");
            #endif

            return 5;

        }

        if (i == 4 && ack_packet[3] == 0x00) {
            #ifdef DEBUG
                Serial.println("NAK Received");
            #endif

            return 1;

        }
    }

    for (i = 2; i < 8 ;i++) {
        CK_A = CK_A + ack_packet[i];
        CK_B = CK_B + CK_A;
    }

    if (msg_ID[0] == ack_packet[6] && msg_ID[1] == ack_packet[7] && CK_A == ack_packet[8] && CK_B == ack_packet[9]) {
        #ifdef DEBUG
            Serial.println("Success!");
            Serial.print("ACK Received! ");
            printHex(ack_packet, sizeof(ack_packet));

        #endif

        return 10;

    } else {
        #ifdef DEBUG
            Serial.print("ACK Checksum Failure: ");
            printHex(ack_packet, sizeof(ack_packet));
        #endif

        delay(1000);
        return 1;

    }
}

#ifdef DEBUG

    /**
     * @brief Print the hex data
     * 
     * @param data 
     * @param length 
     */

    void printHex(uint8_t *data, uint8_t length) {
        char tmp[length*2+1];
        byte first ;

        int j=0;

        for (byte i = 0; i < length; i++) {
            first = (data[i] >> 4) | 48;
            if (first > 57) tmp[j] = first + (byte)7;
            else tmp[j] = first ;
            j++;

            first = (data[i] & 0x0F) | 48;
            if (first > 57) tmp[j] = first + (byte)7;
            else tmp[j] = first;
            j++;
        }

        tmp[length*2] = 0;
        for (byte i = 0, j = 0; i < sizeof(tmp); i++) {
            Serial.print(tmp[i]);
            if (j == 1) {
                Serial.print(" ");
                j = 0;
            } else {
                j++
            }
        }

        Serial.println();

    }

#endif