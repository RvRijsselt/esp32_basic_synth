/*
 * Copyright (c) 2021 Marcel Licence
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dieses Programm ist Freie Software: Sie k�nnen es unter den Bedingungen
 * der GNU General Public License, wie von der Free Software Foundation,
 * Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
 * ver�ffentlichten Version, weiter verteilen und/oder modifizieren.
 *
 * Dieses Programm wird in der Hoffnung bereitgestellt, dass es n�tzlich sein wird, jedoch
 * OHNE JEDE GEW�HR,; sogar ohne die implizite
 * Gew�hr der MARKTF�HIGKEIT oder EIGNUNG F�R EINEN BESTIMMTEN ZWECK.
 * Siehe die GNU General Public License f�r weitere Einzelheiten.
 *
 * Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 * Programm erhalten haben. Wenn nicht, siehe <https://www.gnu.org/licenses/>.
 */

/*
 * pinout of ESP32 DevKit found here:
 * https://circuits4you.com/2018/12/31/esp32-devkit-esp32-wroom-gpio-pinout/
 */

#ifdef __CDT_PARSER__
#include <cdt.h>
#endif

#include "config.h"

/*
 * required include files
 * add also includes used for other modules
 * otherwise arduino generated declaration may cause errors
 */
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_ADS1X15.h>
#include "Adafruit_MPR121.h"

Adafruit_ADS1115 ads;
Adafruit_MPR121 cap = Adafruit_MPR121();

const float VPS = 4.096 / 32768.0; // volts per step

uint16_t lasttouched = 0;
uint16_t currtouched = 0;

void App_UsbMidiShortMsgReceived(uint8_t *msg)
{
    Midi_SendShortMessage(msg);
    Midi_HandleShortMsg(msg, 8);
}

void setup()
{
    /*
     * this code runs once
     */
    delay(500);

    Serial.begin(115200);

    Serial.println();

    Serial.printf("esp32_basic_synth  Copyright (C) 2021  Marcel Licence\n");
    Serial.printf("This program comes with ABSOLUTELY NO WARRANTY;\n");
    Serial.printf("This is free software, and you are welcome to redistribute it\n");
    Serial.printf("under certain conditions; \n");

    ScanI2C();

    if (!ads.begin()) {
        Serial.println("Failed to initialize ADS.");
        while (1);
    }
    if (!cap.begin(0x5B)) {
        Serial.println("MPR121 not found, check wiring?");
        while (1);
    }

    Delay_Init();
    Serial.printf("Initialize Synth Module\n");
    Synth_Init();
    Serial.printf("Initialize I2S Module\n");

    // setup_reverb();

#ifdef BLINK_LED_PIN
    Blink_Setup();
#endif

#ifdef ESP32_AUDIO_KIT
#ifdef ES8388_ENABLED
    ES8388_Setup();
#else
    ac101_setup();
#endif
#endif

    setup_i2s();
    Serial.printf("Initialize Midi Module\n");

    /*
     * setup midi module / rx port
     */
    Midi_Setup();

    Serial.printf("Turn off Wifi/Bluetooth\n");
#if 0
    setup_wifi();
#else
    WiFi.mode(WIFI_OFF);
#endif

#ifndef ESP8266
    btStop();
    // esp_wifi_deinit();
#endif

    Serial.printf("ESP.getFreeHeap() %d\n", ESP.getFreeHeap());
    Serial.printf("ESP.getMinFreeHeap() %d\n", ESP.getMinFreeHeap());
    Serial.printf("ESP.getHeapSize() %d\n", ESP.getHeapSize());
    Serial.printf("ESP.getMaxAllocHeap() %d\n", ESP.getMaxAllocHeap());

    Serial.printf("Firmware started successfully\n");

    // Enable amplifier
    pinMode(GPIO_PA_EN, OUTPUT);
    digitalWrite(GPIO_PA_EN, HIGH);

    

    pinMode(PIN_KEY_1, INPUT_PULLUP);
    pinMode(PIN_KEY_2, INPUT_PULLUP);
    pinMode(PIN_KEY_3, INPUT_PULLUP);
    //pinMode(PIN_KEY_4, INPUT_PULLUP);
    //pinMode(PIN_KEY_5, INPUT_PULLUP);
    //pinMode(PIN_KEY_6, INPUT_PULLUP);

    // Too loud!
    ES8388_SetOUT1VOL(0, 0.5);
    ES8388_SetOUT2VOL(0, 0.5);

#if 0 /* activate this line to get a tone on startup to test the DAC */
    Synth_NoteOn(0, 64, 1.0f);
#endif

#if 1 || (defined ADC_TO_MIDI_ENABLED) || (defined MIDI_VIA_USB_ENABLED)
    Core0TaskInit();
#endif
}

/*
 * Core 0
 */
/* this is used to add a task to core 0 */
TaskHandle_t Core0TaskHnd;

inline
void Core0TaskInit()
{
    /* we need a second task for the terminal output */
    xTaskCreatePinnedToCore(Core0Task, "CoreTask0", 8000, NULL, 999, &Core0TaskHnd, 0);
}

void Core0TaskSetup()
{
    /*
     * init your stuff for core0 here
     */
#ifdef ADC_TO_MIDI_ENABLED
    AdcMul_Init();
#endif

#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Setup();
#endif
}

#ifdef ADC_TO_MIDI_ENABLED
static uint8_t adc_prescaler = 0;
#endif

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float roundf(float val, int dec) 
{
    auto m = pow(10, dec);
    return (int)(val * m) / (float)m;
}

void Core0TaskLoop()
{
    ///*
    // * put your loop stuff for core0 here
    // */
    static float last0, last1, last2, last3;
    auto adc0 = roundf(ads.readADC_SingleEnded(0) * VPS, 2);
    auto adc1 = roundf(ads.readADC_SingleEnded(1) * VPS, 2);
    auto adc2 = roundf(ads.readADC_SingleEnded(2) * VPS, 2);
    auto adc3 = roundf(ads.readADC_SingleEnded(3) * VPS, 2);

    auto val = analogRead(PIN_KEY_ANALOG);
    Serial.printf("analog: %d", val);

    if (adc0 != last0 || adc1 != last1 || adc2 != last2 || adc3 != last3) {
        Serial.printf("ADC: %f, %f, %f, %f\n", adc0, adc1, adc2, adc3);
        last0 = adc0;
        last1 = adc1;
        last2 = adc2;
        last3 = adc3;

        Midi_PitchBend(0, mapfloat(adc0, 0.0, 2.2, 0, 16384));
        Synth_SetParam(4, mapfloat(adc2, 0.3, 2.3, 1.0, 0.0f));
        Synth_SetParam(9, mapfloat(adc3, 0.3, 2.3, 1, 0.0f));

        //Synth_ModulationWheel(0, mapfloat(adc0, 0, 2.2, -1.0, 1.0));

        //Delay_SetFeedback(0, adc1);
        ES8388_SetOUT1VOL(0,  mapfloat(adc1, 0, 2.2, 0, 1.0f)); 
        ES8388_SetOUT2VOL(0,  mapfloat(adc1, 0, 2.2, 0, 1.0f));
    }


    delay(100);

    // Get the currently touched pads
    currtouched = cap.touched();
    
    for (uint8_t i=0; i<12; i++) {
        // it if *is* touched and *wasnt* touched before, alert!
        if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
            Serial.print(i); Serial.println(" touched");
            Midi_NoteOn(1, 50+i*4, 64);
        }
        // if it *was* touched and now *isnt*, alert!
        if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
            Serial.print(i); Serial.println(" released");
            Midi_NoteOff(1, 50+i*4);
        }
    }

    // reset our state
    lasttouched = currtouched;

#ifdef ADC_TO_MIDI_ENABLED
#ifdef MIDI_VIA_USB_ENABLED
    adc_prescaler++;
    if (adc_prescaler > 15) /* use prescaler when USB is active because it is very time consuming */
#endif /* MIDI_VIA_USB_ENABLED */
    {
        adc_prescaler = 0;
        AdcMul_Process();
    }
#endif /* ADC_TO_MIDI_ENABLED */
#ifdef MIDI_VIA_USB_ENABLED
    UsbMidi_Loop();
#endif

#ifdef MCP23_MODULE_ENABLED
    MCP23_Loop();
#endif
}

void Core0Task(void *parameter)
{
    Core0TaskSetup();

    while (true)
    {
        Core0TaskLoop();

        /* this seems necessary to trigger the watchdog */
        delay(1);
        yield();
    }
}

/*
 * use this if something should happen every second
 * - you can drive a blinking LED for example
 */
inline void Loop_1Hz(void)
{
#ifdef BLINK_LED_PIN
    Blink_Process();
#endif

    //Midi_NoteOff(0, 64);
    //Midi_NoteOff(0, 74);
    //Midi_NoteOff(0, 84);
    //Midi_NoteOff(0, 94);
    //Midi_NoteOff(0, 104);
    //Midi_NoteOff(0, 114);

    //if (digitalRead(PIN_KEY_1) == LOW) {
    //    Serial.println("Key 1");
    //    Midi_NoteOn(0, 64, 64);
    //}
    //if (digitalRead(PIN_KEY_2) == LOW) {
    //    Serial.println("Key 2");
    //    Midi_NoteOn(0, 74, 64);
    //}
    //if (digitalRead(PIN_KEY_3) == LOW) {
    //    Serial.println("Key 3");
    //    Midi_NoteOn(0, 84, 64);
    //}
    //if (digitalRead(PIN_KEY_4) == LOW) {
    //    Serial.println("Key 4");
    //    Midi_NoteOn(0, 94, 64);
    //}
    //if (digitalRead(PIN_KEY_5) == LOW) {
    //    Serial.println("Key 5");
    //    Midi_NoteOn(0, 104, 64);
    //}
    //if (digitalRead(PIN_KEY_6) == LOW) {
    //    Serial.println("Key 6");
    //    Synth_NoteOn(0, 114, 1.0f);
    //}
}


/*
 * our main loop
 * - all is done in a blocking context
 * - do not block the loop otherwise you will get problems with your audio
 */
float fl_sample, fr_sample;

void loop()
{
    static uint32_t loop_cnt_1hz;
    static uint8_t loop_count_u8 = 0;

    loop_count_u8++;

    loop_cnt_1hz ++;
    if (loop_cnt_1hz >= SAMPLE_RATE)
    {
        Loop_1Hz();
        loop_cnt_1hz = 0;
    }

#ifdef ARP_MODULE_ENABLED
    Arp_Process(1);
#endif

    if (i2s_write_stereo_samples(&fl_sample, &fr_sample))
    {
        /* nothing for here */
    }
    Synth_Process(&fl_sample, &fr_sample);
    /*
     * process delay line
     */
    Delay_Process(&fl_sample, &fr_sample);

    /*
     * Midi does not required to be checked after every processed sample
     * - we divide our operation by 8
     */
    if (loop_count_u8 % 8 == 0)
    {
        Midi_Process();
#ifdef MIDI_VIA_USB_ENABLED
        UsbMidi_ProcessSync();
#endif
    }
}


/*
 * Test functions
 */

void  ScanI2C(void)
{

    Wire.begin(I2C_SDA, I2C_SCL);

    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.print(address, HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
            {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
    {
        Serial.println("No I2C devices found\n");
    }
    else
    {
        Serial.println("done\n");
    }
}

