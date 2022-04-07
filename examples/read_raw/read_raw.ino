/**
 * @file read_raw.ino
 * @author Joseph Telaak
 * @brief Polls GPS
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "neo7gps.h"
#include <SoftwareSerial.h>

NEO7GPS gps();
SoftwareSerial* gpsSerial;

void setup() {
    Serial.begin(9600);

    gps.setup();
    gpsSerial = gps.getSerial();

}

void loop() {
    char recvChar;
    String sentence; 

    while(1) {
        if(gpsSerial.available()) {
            recvChar = gpsSerial.read();
            
            if(String(recvChar) == ",") {
                if(grabNext){
                    dataGPS[pointerGPS] = sentence;
                    Serial.println(dataGPS[pointerGPS]);
                    pointerGPS = pointerGPS -1;
                    
                    if(pointerGPS <1){
                        grabNext = false;
                        
                        Serial.println(dataGPS[5]);  // Time
                        Serial.print(dataGPS[4]);    
                        Serial.println(dataGPS[3]);  // North
                        Serial.print(dataGPS[2]);
                        Serial.println(dataGPS[1]);  // West
                
                    }
                }

                if(sentence == "M$GPGGA"){
                    grabNext = true; 
                    pointerGPS = 5;

                }

                sentence ="";

            } else{
                sentence = sentence + String(recvChar);

            } 
        }
    }
}
