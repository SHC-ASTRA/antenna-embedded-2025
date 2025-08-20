/**
 * @file main.cpp
 * @author David Sharpe (ds0196@uah.edu)
 * @brief description
 *
 */

//------------//
//  Includes  //
//------------//

#include <Arduino.h>
#include <LSS.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "AstraMisc.h"
#include "AstraSensors.h"
#include "AntennaMainMCU.h"


//------------//
//  Settings  //
//------------//

// Comment out to disable LED blinking
#define BLINK

#define ENABLE_MOVE

#define ANGLE_LIM 179

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 1, 4);

IPAddress bsIP(192, 168, 1, 31);

const unsigned int localPort = 42069;  // local port to listen on for UDP packets


//---------------------//
//  Component classes  //
//---------------------//

LSS myLSS(254);

SFE_UBLOX_GNSS myGNSS;

Adafruit_BNO055 bno;

EthernetUDP Udp;


//----------//
//  Timing  //
//----------//

uint32_t lastBlink = 0;
bool ledState = false;

long lastAlignment = 0;
long lastRoverPos = 0;
long lastPrint = 0;
long lastTelemetry = 0;
long lastSensorPoll = 0;


double roverlat = 0;
double roverlon = 0;
double my_pos[3] = {0};
int currentHeading = 0;
int requiredHeading = 0;

enum MoveMode {
    LIMP = 0,
    HOLD,
    TRACK
} moveMode;

bool gpsFoundOnBoot = false;


//--------------//
//  Prototypes  //
//--------------//

int calcHeading(double mylat, double mylon, double targetlat, double targetlon);
float clamp_angle(float angle);


//------------------------------------------------------------------------------------------------//
//  Setup
//------------------------------------------------------------------------------------------------//
//
//
//------------------------------------------------//
//                                                //
//      ////////    //////////    //////////      //
//    //                //        //        //    //
//    //                //        //        //    //
//      //////          //        //////////      //
//            //        //        //              //
//            //        //        //              //
//    ////////          //        //              //
//                                                //
//------------------------------------------------//
void setup() {
    //--------//
    //  Pins  //
    //--------//

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);


    //------------------//
    //  Communications  //
    //------------------//

    Serial.begin(SERIAL_BAUD);
    Serial1.begin(LSS_BAUD);

    Ethernet.init(ETHERNET_CS);
    Ethernet.begin(mac, ip);
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield was not found.");
    }
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected.");
    }
    if (Ethernet.hardwareStatus() != EthernetNoHardware && Ethernet.linkStatus() != LinkOFF) {
        Serial.println("Ethernet ready.");
    }

    Udp.begin(localPort);


    //-----------//
    //  Sensors  //
    //-----------//

    if(!bno.begin())
        Serial.println("BNO055 failed");
    else
        Serial.println("BNO055 started successfully");

    gpsFoundOnBoot = myGNSS.begin();
    if(!gpsFoundOnBoot)
        Serial.println("GPS not working");
    else
        Serial.println("GPS is working");


    // Setup for GPS (copied directly from Core)

    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.setNavigationFrequency(30);
    // Create storage for the time pulse parameters
    UBX_CFG_TP5_data_t timePulseParameters;

    // Get the time pulse parameters
    if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
    {
        Serial.println(F("getTimePulseParameters failed! not Freezing..."));
    }

    // Print the CFG TP5 version
    Serial.print(F("UBX_CFG_TP5 version: "));
    Serial.println(timePulseParameters.version);

    timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
    //timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin instead, if the module has one

    // We can configure the time pulse pin to produce a defined frequency or period
    // Here is how to set the frequency:

    // While the module is _locking_ to GNSS time, make it generate 2kHz
    timePulseParameters.freqPeriod = 2000; // Set the frequency/period to 2000Hz
    timePulseParameters.pulseLenRatio = 0x55555555; // Set the pulse ratio to 1/3 * 2^32 to produce 33:67 mark:space

    // When the module is _locked_ to GNSS time, make it generate 1kHz
    timePulseParameters.freqPeriodLock = 1000; // Set the frequency/period to 1000Hz
    timePulseParameters.pulseLenRatioLock = 0x80000000; // Set the pulse ratio to 1/2 * 2^32 to produce 50:50 mark:space

    timePulseParameters.flags.bits.active = 1; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
    timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
    timePulseParameters.flags.bits.isFreq = 1; // Tell the module that we want to set the frequency (not the period)
    timePulseParameters.flags.bits.isLength = 0; // Tell the module that pulseLenRatio is a ratio / duty cycle (* 2^-32) - not a length (in us)
    timePulseParameters.flags.bits.polarity = 1; // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

    // Now set the time pulse parameters
    if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
    {
        Serial.println(F("setTimePulseParameters failed!"));
    }
    else
    {
        Serial.println(F("Success! (setTimePulseParameters)"));
    }


    //--------------------//
    //  Misc. Components  //
    //--------------------//

	LSS::initBus(Serial2, LSS_DefaultBaud);
    moveMode = TRACK;
    myLSS.limp();  // Wait to align antenna
    // myLSS.wheel(0);  // Hold
    Serial.println("Setup finished.");
}


//------------------------------------------------------------------------------------------------//
//  Loop
//------------------------------------------------------------------------------------------------//
//
//
//-------------------------------------------------//
//                                                 //
//    /////////      //            //////////      //
//    //      //     //            //        //    //
//    //      //     //            //        //    //
//    ////////       //            //////////      //
//    //      //     //            //              //
//    //       //    //            //              //
//    /////////      //////////    //              //
//                                                 //
//-------------------------------------------------//
void loop() {
    //----------//
    //  Timers  //
    //----------//
#ifdef BLINK
    if (millis() - lastBlink > 1000) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }
#endif

    // Timeout
    if (millis() - lastRoverPos > 5000 && millis() - lastAlignment < 1000 && millis() > 5000) {
        myLSS.wheel(0);  // Stop if no rover position received in 5 seconds
    }

    // Polling sensors
    if (millis() - lastSensorPoll > 100) {
        lastSensorPoll = millis();

        // Poll BNO055 for orientation data
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        currentHeading = orientationData.orientation.x;

        // Poll GNSS for position data
        getPosition(myGNSS, my_pos);

        requiredHeading = calcHeading(my_pos[0], my_pos[1], roverlat, roverlon);

        currentHeading = clamp_angle(currentHeading);
        requiredHeading = clamp_angle(requiredHeading);
        if (requiredHeading > 170) {
            requiredHeading = 170;
        } else if (requiredHeading < -170) {
            requiredHeading = -170;
        }
    }

    // Sending info to Serial
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        Serial.println();
        Serial.print("Calibration: Sys=");
        Serial.print(system);
        Serial.print(" Gyro=");
        Serial.print(gyro);
        Serial.print(" Accel=");
        Serial.print(accel);
        Serial.print(" Mag=");
        Serial.print(mag);
        Serial.println();

        if (!gpsFoundOnBoot) {
            Serial.println("GPS not found on boot.");
        }
        Serial.print("   My position: ");
        Serial.print(my_pos[0], 7);
        Serial.print(",  ");
        Serial.print(my_pos[1], 7);
        Serial.print(" (");
        Serial.print(my_pos[2]);
        Serial.print(" sats)");
        Serial.println();

        Serial.print("Rover position: ");
        Serial.print(roverlat, 7);
        Serial.print(",  ");
        Serial.print(roverlon, 7);
        Serial.println();

        Serial.print("Required heading: ");
        Serial.print(requiredHeading);
        Serial.print("    My heading: ");
        Serial.print(currentHeading);
        Serial.println();

        Serial.print("LSS position: ");
        Serial.print(myLSS.getPosition());
        Serial.println();
    }

    // Sending info back to basestation
    if (millis() - lastTelemetry > 1000) {
        lastTelemetry = millis();

        // Full status
        uint8_t system, gyro, accel, mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        String output;
        output.reserve(128);
        char buff[16];
        output += "{ \"lat\": ";
        dtostrf(my_pos[0], 5, 7, buff);
        output += buff;
        output += ", \"lon\": ";
        dtostrf(my_pos[1], 5, 7, buff);
        output += buff;
        output += ", \"sat\": ";
        output += int(my_pos[2]);
        output += ", \"heading\": ";
        output += currentHeading;
        output += ", \"calib\": ";
        output += system * 1000 + gyro * 100 + accel * 10 + mag;
        output += " }\n";
        Udp.beginPacket(Udp.remoteIP(), 42069);
        Udp.write(output.c_str(), output.length());
        Udp.endPacket();
    }


    // Main control loop
    if (millis() - lastAlignment > 100 && millis() - lastRoverPos < 5000 && millis() > 5000) {  // Requires update <5 seconds ago
        lastAlignment = millis();

        // Check calibration
        uint8_t mag = 0;
        bno.getCalibration(nullptr, nullptr, nullptr, &mag);

        // Make LSS rotate towards the required heading
        int error = requiredHeading - currentHeading;
        if (moveMode == TRACK && mag > 0 && my_pos[2] >= 3) {  // Only move if magnetometer is calibrated and at least 3 satellites
#ifdef ENABLE_MOVE
            // Stop if within tolerance
            if (abs(error) < 5 || currentHeading > 170 || currentHeading < -170) {
                myLSS.wheel(0);
            } else if (error > 0) {
                myLSS.wheel(20);
            } else if (error < 0) {
                myLSS.wheel(-20);
            }
#else
            if (abs(error) >= 5) {
                Serial.print("Would be moving, but LSS movement disabled in code. Uncomment ENABLE_MOVE to enable movement.");
            }
#endif
        } else {
            myLSS.hold();
            Serial.println("Insufficient sensor data, please check GPS and IMU.");
            if (moveMode == LIMP)
                myLSS.limp();
        }
    }


    //-------------//
    //  UDP Input  //
    //-------------//
    
    char packetBuffer[128];  // buffer to hold incoming packet

    int packetSize = Udp.parsePacket();
    if (packetSize) {
        Serial.print("\nReceived packet from ");
        IPAddress remote = Udp.remoteIP();
        for (int i = 0; i < 4; i++) {
            Serial.print(remote[i], DEC);
            if (i < 3) {
                Serial.print(".");
            }
        }
        Serial.print(":");
        Serial.print(Udp.remotePort());

        // read the packet into packetBuffer
        Udp.read(packetBuffer, 128);
        Serial.print(" [");
        Serial.print(packetSize);
        Serial.print("] ");
        Serial.println(packetBuffer);

        String input = String(packetBuffer);
        input.trim();
        input.toLowerCase();
        std::vector<String> args = {};
        parseInput(input, args);

        if (args[0] == "reset") {
            Serial.println("Resetting LSS...");
            myLSS.reset();

        } else if (args[0] == "track" || args[0].startsWith("track")) {
            moveMode = TRACK;
            Serial.println("LSS is now in normal mode.");

        } else if (args[0] == "limp" || args[0].startsWith("limp")) {
            moveMode = LIMP;
            myLSS.limp();
            Serial.println("LSS is now in limp mode.");

        } else if (args[0] == "hold" || args[0].startsWith("hold")) {
            moveMode = HOLD;
            myLSS.hold();
            Serial.println("LSS is now in hold mode.");

        } else if (args.size() == 2) {
            double lat = args[0].toDouble();
            double lon = args[1].toDouble();
            lastRoverPos = millis();
            roverlat = lat;
            roverlon = lon;
            Serial.print("Rover: ");
            Serial.print(roverlat);
            Serial.print(", ");
            Serial.println(roverlon);
        }
    }


    //------------------//
    //  UART/USB Input  //
    //------------------//
    //
    //
    //-------------------------------------------------------//
    //                                                       //
    //      /////////    //\\        ////    //////////      //
    //    //             //  \\    //  //    //        //    //
    //    //             //    \\//    //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //    //             //            //    //        //    //
    //      /////////    //            //    //////////      //
    //                                                       //
    //-------------------------------------------------------//
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');

        input.trim();                   // Remove preceding and trailing whitespace
        std::vector<String> args = {};  // Initialize empty vector to hold separated arguments
        parseInput(input, args);   // Separate `input` by commas and place into args vector
        args[0].toLowerCase();          // Make command case-insensitive
        String command = args[0];       // To make processing code more readable

        //--------//
        //  Misc  //
        //--------//
        if (command == "ping") {
            Serial.println("pong");
        }

        else if (command == "time") {
            Serial.println(millis());
        }

        else if (command == "led") {
            if (args[1] == "on")
                digitalWrite(LED_BUILTIN, HIGH);
            else if (args[1] == "off")
                digitalWrite(LED_BUILTIN, LOW);
            else if (args[1] == "toggle") {
                ledState = !ledState;
                digitalWrite(LED_BUILTIN, ledState);
            }
        }

        else if (command == "rover" && checkArgs(args, 2)) {
            lastRoverPos = millis();
            roverlat = args[1].toDouble();
            roverlon = args[2].toDouble();
        }

        else if (command == "lss") {
            myLSS.wheel(args[1].toFloat());
            Serial.print("Sending to LSS: ");
            Serial.println(args[1].toFloat());
        }

        else if (command == "reset") {
            myLSS.reset();
        }

        else if (command == "hold") {
            myLSS.wheel(0);
        }

        else if (command == "limp") {
            myLSS.limp();
        }

        //-----------//
        //  Sensors  //
        //-----------//

        //----------//
        //  Motors  //
        //----------//
    }
}


//------------------------------------------------------------------------------------------------//
//  Function definitions
//------------------------------------------------------------------------------------------------//
//
//
//----------------------------------------------------//
//                                                    //
//    //////////    //          //      //////////    //
//    //            //\\        //    //              //
//    //            //  \\      //    //              //
//    //////        //    \\    //    //              //
//    //            //      \\  //    //              //
//    //            //        \\//    //              //
//    //            //          //      //////////    //
//                                                    //
//----------------------------------------------------//

int calcHeading(double mylat, double mylon, double targetlat, double targetlon) {
    double my_lat_r = mylat * M_PI / 180.0;
    double my_lon_r = mylon * M_PI / 180.0;
    double t_lat_r = targetlat * M_PI / 180.0;
    double t_lon_r = targetlon * M_PI / 180.0;
    double x = cos(t_lat_r) * sin(t_lon_r - my_lon_r);
    double y = cos(my_lat_r) * sin(t_lat_r) - sin(my_lat_r) * cos(t_lat_r) * cos(t_lon_r - my_lon_r);

    int deg = atan2(x, y) * 180.0 / M_PI;  // Calculate angle and convert to degrees [-180, 180]
    return (deg + 360) % 360;  // Normalize to 0-360 degrees
}

float clamp_angle(float angle) {
    angle = fmod(angle, 360.0);
    if (angle < 0) {
        angle += 360;
    }
    if (angle > 180) {
        angle -= 360;
    }
    return angle;
}
