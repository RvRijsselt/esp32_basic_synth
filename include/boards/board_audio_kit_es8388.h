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

/**
 * @file board_audio_kit_es8388.h
 * @author Marcel Licence
 * @date 22.09.2021
 *
 * @brief Board description for the ESP32 Audio Kit with the ES8388
 *
 * ES8388_CFG_I2C can be set to 1..3 to match different datasheets
 * ES8388_CFG_I2S can be set to 1..3 to match different datasheets
 *
 * 2 - coveres pinout shown in https://www.makerfabs.com/desfile/files/ESP32-A1S%20Product%20Specification.pdf
 * 3 - coveres pinout shown in https://docs.ai-thinker.com/_media/esp32-a1s_v2.3_specification.pdf
 * 4 - unknown origin
 *
 * Tested configuration: I2C: 1, I2S: 4
 * 
 * Add: 
 *  - Tested with AI Thinker ESP32 Audio Kit V2.2 2957 with ESP32-A1S 2974. 
 *    Looks like they put the wrong module on the wrong board.
 * 
 */
#include "config.h"

#ifndef BOARDS_BOARD_AUDIO_KIT_ES8388_H_
#define BOARDS_BOARD_AUDIO_KIT_ES8388_H_


#define ES8388_CFG_I2C  1
#define ES8388_CFG_I2S  4


/* on board led */
#define BLINK_LED_PIN     22                // D4
#define BLINK_LED_PIN2    19                // D5

/* headphone detection and power amp */
#define GPIO_HP_DETECT    GPIO_NUM_39       /* only low for fraction of second when plugging in */
#define GPIO_PA_EN        GPIO_NUM_21

/* on board keys */
#define PIN_KEY_ANALOG              (36)    /* need to do some resoldering first */
#define PIN_KEY_1                   (36)    
#define PIN_KEY_2                   (13)
#define PIN_KEY_3                   (19)    /* same as led 5 */
#define PIN_KEY_4                   (23)    /* i2c port, dont use! */
#define PIN_KEY_5                   (18)    /* i2c port, dont use! */
#define PIN_KEY_6                   (5)     /* i2s port, dont use! */

//#define MIDI_RX_PIN 22 /* U2RRXD */
//#define MIDI_RX_PIN 19
//#define MIDI_RX_PIN 23 /* D5 LED will blink then */
//#define LED_STRIP_PIN         12  // Not used

#if ES8388_CFG_I2C==1
#define ES8388_PIN_SDA  18
#define ES8388_PIN_SCL  23
#define I2C_SDA 18 /* I2C shared with pin header */
#define I2C_SCL 23 /* I2C shared with pin header */
#elif ES8388_CFG_I2C==2
#define ES8388_PIN_SDA  33
#define ES8388_PIN_SCL  32
#elif ES8388_CFG_I2C==2
#define ES8388_PIN_SDA  27
#define ES8388_PIN_SCL  28
#endif

#if ES8388_CFG_I2S==1
#define ES8388_PIN_DOUT 35
#define ES8388_PIN_DIN  25
#define ES8388_PIN_LRCK 26
#define ES8388_PIN_SCLK 27
#define ES8388_PIN_MCLK 0
#elif ES8388_CFG_I2S==2
#define ES8388_PIN_DOUT 35
#define ES8388_PIN_DIN  25
#define ES8388_PIN_LRCK 26
#define ES8388_PIN_SCLK 5
#define ES8388_PIN_MCLK 0
#elif ES8388_CFG_I2S==3
#define ES8388_PIN_DOUT 8
#define ES8388_PIN_DIN  6
#define ES8388_PIN_LRCK 7
#define ES8388_PIN_SCLK 5
#define ES8388_PIN_MCLK 1
#elif ES8388_CFG_I2S==4
#define ES8388_PIN_DOUT 35
#define ES8388_PIN_DIN  26
#define ES8388_PIN_LRCK 25
#define ES8388_PIN_SCLK 5
#define ES8388_PIN_MCLK 0
#endif


#define ESP32_AUDIO_KIT
#define ES8388_ENABLED
#define I2S_USE_APLL


#ifdef ADC_TO_MIDI_ENABLED
#define ADC_INPUTS  8
#define ADC_MUL_S0_PIN  23
#define ADC_MUL_S1_PIN  18
#define ADC_MUL_S2_PIN  14
#define ADC_MUL_S3_PIN  5    /* <- not used, this has not been tested */
#define ADC_MUL_SIG_PIN 12
#endif


/* map selected pins to global */
#define I2S_MCLK_PIN ES8388_PIN_MCLK
#define I2S_BCLK_PIN ES8388_PIN_SCLK
#define I2S_WCLK_PIN ES8388_PIN_LRCK
#define I2S_DOUT_PIN ES8388_PIN_DIN
#define I2S_DIN_PIN ES8388_PIN_DOUT


#endif /* BOARDS_BOARD_AUDIO_KIT_ES8388_H_ */
