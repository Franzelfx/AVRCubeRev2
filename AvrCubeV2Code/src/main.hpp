/**
 * @file main.hpp
 * @author Fabian Franz fabian.franz0596@gmail.com
 * @brief Includes, defines and function prototypes for the "Cube Project".
 * @version 09.22
 * @date 2022-09-14
 * 
 * @copyright Copyright (c) 2022
 * 
 * \code
  ______          _    _                 _                  
 |  ____|   /\   | |  | |               | |                 
 | |__     /  \  | |__| |  ______       | | ___ _ __   __ _ 
 |  __|   / /\ \ |  __  | |______|  _   | |/ _ \ '_ \ / _` |
 | |____ / ____ \| |  | |          | |__| |  __/ | | | (_| |
 |______/_/    \_\_|  |_|           \____/ \___|_| |_|\__,_|
                                                                                                           
 * \endcode
 */

#ifndef MAIN_HPP
#define MAIN_HPP

// ---------------------------------------------------------------- //
// -- Includes ---------------------------------------------------- //
// ---------------------------------------------------------------- //
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// ---------------------------------------------------------------- //
// -- Defines ----------------------------------------------------- //
// ---------------------------------------------------------------- //
#define F_CPU 8000000
#define __DELAY_BACKWARD_COMPATIBLE__
// Defines for the button pin
#define BUTTON PB2
// Defines for the connected LEDs
#define D1 PA0
#define D2 PA1
#define D3 PA2
#define D4 PA3
#define D5 PA5
#define D6 PA7
#define D7 PB1
// Defines for the SPI interface.
#define SCK PA4
#define MISO PA5
#define MOSI PA6
// Defines for the I2C interface (no hardware interface).
#define SCL PA4
#define SDA PA6
// Defines for unconnected pins
#define UNCONNECTED PB0

// General defines
#define OUTPUT 0
#define INPUT 1
#define ENABLE true
#define DISABLE false
#define ON true
#define OFF false

// Defines for control flow
#define SLEEP_THRESHOLD 60000                 // Millieconds until the device go to sleep if no action occurs
#define ANGLE_GRADIENT_THRESHOLD 25  // The threshold for the absolute motion value in LSB registers
#define DICE_STEPS_FIRST_ROUND 5    // The number of steps the dice has to roll
#define DICE_STEPS_SECOND_ROUND 5   // The number of steps the dice has to roll
#define DICE_TIME_STEPS 100         // The time step between dice rolls in ms
#define DICE_TIME_STEPS_INCREASE 10 // The time step increase between dice rolls in ms in secon round
#define ANGLETHRESHOLD 1            // The threshold for the angle in degrees
#define CALIBRATION_STEP_DELAY 500  // The delay between calibration steps in ms
#define OFFSET_CALIBRATION_STEPS 10      // The number of calibration steps
#define OFFSET_CALIBRATION_STEP_DELAY 10 // The delay between offset calibration steps in ms
#define FORCE_SLEEP_STEPTIME 500         // The delay until new number
#define FORCE_SLEEP_TIME 6 * FORCE_SLEEP_STEPTIME // When button is hold this amout of milliseconds the device will go to sleep
#define MOTION_MEASURING_REPETITIONS 5 // The number of repetitions for the motion measuring

// Defines for I2C
#define I2C_DELAY 10 // 10 us -> 100 kHz
#define SDA_ON (PORTA |= (1 << SDA))
#define SDA_OFF (PORTA &= ~(1 << SDA))
#define SCL_READ (PINA & (1 << SCL))
#define SDA_READ (PINA & (1 << SDA))
// Defines for Acceleration Sensor
#define MMA8653FC_ADD 0x1D
#define MMA8653FC_ADDR_READ 0x3B
#define MMA8653FC_ADDR_WRITE 0x3A
// Defines for Acceleration Sensor Registers
#define MMA8653FC_WHO_AM_I 0x0D
#define MMA8653FC_XYZ_DATA_CFG 0x0E
#define MMA8653FC_CTRL_REG1 0x2A
#define MMA8653FC_SYSMOD 0x0B    // System Mode, to control STANDBY, WAKE and SLEEP
#define MMA8653FC_OUT_X_MSB 0x01 // Most significant byte of X-axis acceleration data
#define MMA8653FC_OUT_X_LSB 0x02 // Least significant byte of X-axis acceleration data
#define MMA8653FC_OUT_Y_MSB 0x03 // Most significant byte of Y-axis acceleration data
#define MMA8653FC_OUT_Y_LSB 0x04 // Least significant byte of Y-axis acceleration data
#define MMA8653FC_OUT_Z_MSB 0x05 // Most significant byte of Z-axis acceleration data
#define MMA8653FC_OUT_Z_LSB 0x06 // Least significant byte of Z-axis acceleration data

// ---------------------------------------------------------------- //
// -- Function Prototypes ----------------------------------------- //
// ---------------------------------------------------------------- //
/**
 * @brief Set the LED Pins eigther "INPUT" or "OUTPUT"
 * 
 * @param mode OUTPUT = 0 or INPUT = 1
 */
void setLedPins(uint8_t mode);

/**
 * @brief Set the SDA pin eighter "ON" or "OFF".
 * 
 * @param mode "ON" (input pullup) or "OFF" (output low)
 */
void setSDA(bool mode);

/**
 * @brief Set the SCL pin eighter "ON" or "OFF".
 * 
 * @param mode "ON" (input pullup) or "OFF" (output low)
 */
void setSCL(bool mode);

/**
 * @brief Initialises all the AVR hardware E.g. ports, interrupts, timers, etc.
 */
void init();

/**
 * @brief Turn on all LEDs (LED1 - LED7)
 */

void allLedOn();

/**
 * @brief Turn off all LEDs (LED1 - LED7)
 */
void allLedOff();

/**
 * @brief Deeinitialises all the AVR hardware E.g. ports, interrupts, timers, etc.
 * 
 * @remarks Calling deInit(); is necessary to save power before the CPU goes to sleep.
 */
void deInit();

/**
 * @brief Show a number between 1 and 6 on the LEDs.
 * 
 * @param numberToshow The number to show on the LEDs.
 * 
 * Lremark If the number is larger than 6, no LED will be turned on.
 */
void showNumber(uint8_t numberToShow);
/**
 * @brief Show a cross on the LEDs.
 * 
 * @remark This function is used to indicate some error.
 */
void showCross();

/**
 * @brief Show a hook on the LEDs.
 * 
 * @remark This function is used to indicate some success.
 */
void showHook();

/**
 * @brief Check if all LEDs are working correctly.
 * 
 */
void testLeds();

/**
 * @brief I2C BitBang start condition.
 * 
 */
void start();

/**
 * @brief I2C BitBang stop condition.
 * 
 */
void stop();

/**
 * @brief I2C BitBang tx of one byte.
 * 
 * @param dat The byte to send.
 * 
 * @return ACK or NACK (acknowledge or no acknowledge)
 */
bool tx(uint8_t dat);

/**
 * @brief I2C BitBang rx of one byte.
 * 
 * @param ack ACK or NACK (acknowledge or no acknowledge)
 * 
 * @return The received byte.
 */
uint8_t rx(bool ack);

/**
 * @brief Read a register from the MMA8653FC.
 * 
 * @param reg The register to read.
 * 
 * @return The value of the register.
 */
uint8_t readRegister(uint8_t reg);

/**
 * @brief Write a value to a register of the MMA8653FC.
 * 
 * @param reg The register to write.
 * @param value The value to write.
 */
void writeRegister(uint8_t reg, uint8_t value);

/**
 * @brief Checks, if the I2C connection to the MMA8653FC is working.
 * 
 * @remark If the communication is not working, 
 * it will be shown on the LEDs as a cross.
 * Furthermore, the device will go to sleep.
 */
void testI2C_read();

/**
 * @brief Initialise the MMA8653FC range ans mode.
 */
void MMA8653FC_init();

/**
 * @brief Deinitialise the MMA8653FC to safe energy.
 */
void MMA8653FC_deInit();

/**
 * @brief Read the acceleration data from the MMA8653FC.
 * 
 * @param x The x-axis acceleration data (call by reference).
 * @param y The y-axis acceleration data (call by reference).
 * @param z The z-axis acceleration data (call by reference).
 * 
 * @code
 * // usage example
 * int16_t x, y, z;
 * MMA8653FC_read(&x, &y, &z);
 * @endcode
 */
void getAcceleration(int16_t* x, int16_t* y, int16_t* z);

/**
 * @brief Get's true, if the device is in motion and threshold is reached.
 * 
 * @param threshold Activation threshold in bit.
 * 
 * @return true if threshold is reached.
 */

void getAcceleration(int16_t* x, int16_t* y, int16_t* z, uint8_t repetitions);

/**
 * @brief Implemetation of a variable delay.
 * 
 * @param ms The time you want the core to not execute any code.
 */
void dynamicDelay(uint16_t ms);

/**
 * @brief Get the Angle between two acceleration values.
 * 
 * @param axis The orthogonal axis of device rotation. 
 * @param reference  The reference acceleration value of the other axis.
 * 
 * @return float Resulting angle in degree.
 * 
 * @remarks The function is used to calculate the angle between a
 *          reference acceleration value and the acceleration value
 *          of the axis where the amount of acceleration is changing.
 *          A best practise to calculate the angle would be:
 *          1. Read all the acceleration values from the sensor (x, y, z).
 *          2. Calculate the amount of motion on those axis where the 
 *             acceleration doesn't change significantly. E.g.:
 *             @code amount = sqrt(y * y + z * z); @endcode
 *          3. Calculate the angle between the amount of motion and the
 *             acceleration value of the axis where the amount of motion
 *             is changing. E.g.:
 *             @code angle = getAngle(x, amount); @endcode
 *          4. Use the angle to calculate the rotation of the device.
 *          For further information read: 
 *          https://www.nxp.com/docs/en/application-note/AN3461.pdf
 */
float getAngle(float axis, float reference);

// TODO: add documentation
void getRollNick(float* roll, float* nick);

//TODO: add documentation
int16_t getGradient(int16_t baseValue);

//TODO: add documentation
bool motionDetected(uint8_t threshold);

/**
 * @brief Shows the sprit level on the LEDs.
 * 
 * @remarks The function internally calculates the roll and the nick
 *          angle of the device. It then uses the calculated angles 
 *          to show in which direction the device is rotated.
 *          When it is at rest, only the LED in the middle will be
 *          turned on. When the device is rotated, the LEDs on the
 *          corresponding side will be turned on.
 */
void spritLevel();

/**
 * @brief Realises a random number from 1 - 6 and show it on the LEDs.
 * 
 * @remark It will first generate and show some randome numbers with
 *         a long delay between them. Then the delay gets shorter. 
 *         Finally, it will peristently show a random number on the LEDs.
 *         The function can only be stopped by external interrupt.
 */
void dice();

/**
 * @brief Short calibration of the MMA8653FC values.
 * 
 * @remarks The calibration will first check if the device can 
 *           be reached via the I2C connection. If not, it will
 *           show a cross on the LEDs and go to sleep. After that,
 *           it will read the current acceleration values and
 *           get the offset. The offset will be stored in the
 *           non persistant memory of the device. It is used
 *           to compensate the offset for angle calculation and 
 *           motion detection.
 */
void calibrate();

/**
 * @brief Set the sleep mode, disable peripherals and go to sleep.
 * 
 */
void goToSleep();

/**
 * @brief Initialises the MCU and the peripherals and then loop forever.
 * 
 * @remarks The function will first do the calibration.
 *          Then the function will loop forever, until the sleep counter is reached.
 *          If the button is presses even times, the device is in dice mode.
 *          If the button is presses odd times, the device is in sprit level mode.
 *          Every time, a motion is detected or button is pressed, the sleep counter
 *          will be reset.
 */

#endif /* MAIN_HPP */