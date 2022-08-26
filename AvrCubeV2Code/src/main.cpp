#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// Defines for AVR
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
#define SLEEP_THRESHOLD 60000 // Seconds until the device go to sleep if no action occurs
#define ACCELERATION_THRESHOLD 100 // The threshold for the absolute motion value in LSB registers
#define DICE_STEPS_FIRST_ROUND 5 // The number of steps the dice has to roll
#define DICE_STEPS_SECOND_ROUND 5 // The number of steps the dice has to roll
#define DICE_TIME_STEPS 100 // The time step between dice rolls in ms
#define DICE_TIME_STEPS_INCREASE 10 // The time step increase between dice rolls in ms in secon round
#define ANGLETHRESHOLD 5 // The threshold for the angle in degrees
#define CALIBRATION_STEP_DELAY 1000 // The delay between calibration steps in ms
#define OFFSET_CALIBRATION_STEPS 10 // The number of calibration steps
#define OFFSET_CALIBRATION_STEP_DELAY 10 // The delay between offset calibration steps in ms

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
#define MMA8653FC_SYSMOD 0x0B
#define MMA8653FC_OUT_X_MSB 0x01
#define MMA8653FC_OUT_X_LSB 0x02
#define MMA8653FC_OUT_Y_MSB 0x03
#define MMA8653FC_OUT_Y_LSB 0x04
#define MMA8653FC_OUT_Z_MSB 0x05
#define MMA8653FC_OUT_Z_LSB 0x06
#define X_AXIS MMA8653FC_OUT_X_MSB
#define Y_AXIS MMA8653FC_OUT_Y_MSB
#define Z_AXIS MMA8653FC_OUT_Z_MSB

// Global variable for interrupt.
volatile uint16_t counter = 0; // Counter for the sleep timer -> max = 65.535
volatile uint8_t button_pressed = 0;
// Global variables for sensor offset
int16_t x_offset, y_offset, z_offset;
// #TODO write offset value in sensor registers

// ---------------------------------------------------------------- //
// -- Local functions --------------------------------------------- //
// ---------------------------------------------------------------- //
int main();
void calibrate();
// Utility Functions
void setLedPins(uint8_t mode)
{
  if (mode == OUTPUT) {
    DDRA |= (1 << D1) | (1 << D2) | (1 << D3) | (1 << D4) | (1 << D5) | (1 << D6);
    DDRB |= (1 << D7);
  }
  else
  {
    DDRA &= ~(1 << D1) & ~(1 << D2) & ~(1 << D3) & ~(1 << D4) & ~(1 << D5) & ~(1 << D6);
    DDRB &= ~(1 << D7);
  }
}
void setSDA(bool mode) {
  // If SDA has to be set "on", internal pullup resistor will be enabled
  if (mode == ON) {
    DDRA &= ~(1 << SDA);
    PORTA |= (1 << SDA);
  }
  // If SDA has to be set "off", pin will be driven low
  else {
    DDRA |= (1 << SDA);
    PORTA &= ~(1 << SDA);
  }
}
void setSCL(bool mode) {
  // If SCL has to be set "on", internal pullup resistor will be enabled
  if (mode == ON) {
    DDRA &= ~(1 << SCL);
    PORTA |= (1 << SCL);
  }
  // If SCL has to be set "off", pin will be driven low
  else {
    DDRA |= (1 << SCL);
    PORTA &= ~(1 << SCL);
  }
}
void init() {
  // Setup the LED and I2C pins as outputs.
  setLedPins(OUTPUT);
  // Setup the timer to check for inactivity.
  TIMSK0 |= (1 << OCIE0A);
  TCCR0A |= (1 << WGM01);              // CTC mode
  TCCR0B |= (1 << CS00) | (1 << CS01); // Prescaler 64
  OCR0A |= 124;                        // Resulting in ca. 1ms interrupt.
  // Setup external interrupt INT0.
  DDRB &= ~(1 << BUTTON);               // Set the button pin as input.
  PORTB |= (1 << BUTTON);              // Enable pull-up resistor.
  GIMSK |= (1 << INT0);                 // Enable INT0.
  // Set unconnected pin to input pullup
  DDRB &= ~(1 << UNCONNECTED);
  PORTB |= (1 << UNCONNECTED);
  // Enable interrupts
  sei();
}
void allLedOn()
{
  PORTA |= (1 << D1) | (1 << D2) | (1 << D3) | (1 << D4) | (1 << D5) | (1 << D6);
  PORTB |= (1 << D7);
}
void allLedOff()
{
  PORTA &= ~(1 << D1) & ~(1 << D2) & ~(1 << D3) & ~(1 << D4) & ~(1 << D5) & ~(1 << D6);
  PORTB &= ~(1 << D7);
}
void deInit() {
  button_pressed = 0;
  allLedOff();
  setLedPins(INPUT);
  setSDA(INPUT);
  setSCL(INPUT);
  sei();
}
void showNumber(uint8_t numberToShow)
{
  allLedOff();
  switch (numberToShow)
  {
  case 1:
    PORTA |= (1 << D4);
    break;
  case 2:
    PORTA |= (1 << D3);
    PORTA |= (1 << D5);
    break;
  case 3:
    PORTA |= (1 << D1);
    PORTA |= (1 << D4);
    PORTB |= (1 << D7);
    break;
  case 4:
    PORTA |= (1 << D1);
    PORTA |= (1 << D2);
    PORTA |= (1 << D6);
    PORTB |= (1 << D7);
    break;
  case 5:
    PORTA |= (1 << D1);
    PORTA |= (1 << D2);
    PORTA |= (1 << D4);
    PORTA |= (1 << D6);
    PORTB |= (1 << D7);
    break;
  case 6:
    PORTA |= (1 << D1);
    PORTA |= (1 << D2);
    PORTA |= (1 << D3);
    PORTA |= (1 << D5);
    PORTA |= (1 << D6);
    PORTB |= (1 << D7);
    break;
  default:
    break;
  }
}
void showCross() {
  allLedOff();
  PORTA |= (1 << D1);
  PORTA |= (1 << D2);
  PORTA |= (1 << D4);
  PORTA |= (1 << D6);
  PORTB |= (1 << D7);
}
void showHook() {
  allLedOff();
  PORTA |= (1 << D2);
  PORTA |= (1 << D3);
  PORTA |= (1 << D4);
  PORTA |= (1 << D6);
}
void testLeds() {
  for (uint8_t i = 1; i <= 6; i++)
  {
    showNumber(i);
    _delay_ms(1000);
  }
}
// I2C BitBang Functions
void start() {
  setSDA(ON);  // Set SDA high.
  _delay_us(I2C_DELAY);
  setSCL(ON);  // Set SCL high.
  _delay_us(I2C_DELAY);
  setSDA(OFF); // Set SDA low, start condition is achieved.
  _delay_us(I2C_DELAY);
  setSCL(OFF); // Set SCL low.
  _delay_us(I2C_DELAY);
}
void stop() {
  setSDA(OFF); // Set SDA low, stop condition is achieved.
  _delay_us(I2C_DELAY);
  setSCL(ON);  // Set SCL high.
  _delay_us(I2C_DELAY);
  setSDA(ON);  // Set SDA high.
  _delay_us(I2C_DELAY);
}
bool tx(uint8_t dat) {

  for (uint8_t i = 8; i; i--) {
    (dat & 0x80) ? setSDA(ON) : setSDA(OFF); //Mask for the eigth bit
    _delay_us(I2C_DELAY);
    setSCL(ON);
    _delay_us(I2C_DELAY);
    setSCL(OFF);
    _delay_us(I2C_DELAY);
    dat <<= 1;  //Move 
  }
  setSDA(ON);
  _delay_us(I2C_DELAY);
  setSCL(ON);
  _delay_us(I2C_DELAY);
  bool ack = !SDA_READ;    // Acknowledge bit
  setSCL(OFF);
  return ack;
}
uint8_t rx(bool ack) {
  uint8_t dat = 0;
  setSDA(ON);
  for (uint8_t i = 0; i < 8; i++) {
    dat <<= 1;
    do {
      setSCL(ON);
    } while (SCL_READ == 0);  //clock stretching
    _delay_us(I2C_DELAY);
    if (SDA_READ) dat |= 1;
    _delay_us(I2C_DELAY);
    setSCL(OFF);
  }
  ack ? SDA_OFF : SDA_ON;
  setSCL(ON);
  _delay_us(I2C_DELAY);
  setSCL(OFF);
  setSDA(ON);
  return(dat);
}
uint8_t readRegister(uint8_t reg) {
  cli();
  start();
  tx(MMA8653FC_ADDR_WRITE);
  _delay_us(5 * I2C_DELAY);
  tx(reg);
  _delay_us(5 * I2C_DELAY);
  start();
  _delay_us(5 * I2C_DELAY);
  tx(MMA8653FC_ADDR_READ);
  _delay_us(5 * I2C_DELAY);
  uint8_t value = rx(false);
  stop();
  sei();
  _delay_us(5 * I2C_DELAY);
  return value;
}
void writeRegister(uint8_t reg, uint8_t value) {
  cli();
  start();
  tx(MMA8653FC_ADDR_WRITE);
  _delay_us(5 * I2C_DELAY);
  tx(reg);
  _delay_us(5 * I2C_DELAY);
  tx(value);
  _delay_us(5 * I2C_DELAY);
  stop();
  sei();
}
// MMA8653FC Functions
void testI2C_read() {
  uint8_t whoAmI = readRegister(MMA8653FC_WHO_AM_I);
  if (whoAmI == 0x5A) {
    showHook();
  }
  else {
    showCross();
  }
}
void MMA8653FC_init() {
  writeRegister(MMA8653FC_XYZ_DATA_CFG, 0x00); // 2g range
  writeRegister(MMA8653FC_CTRL_REG1, 0x01); // Active and fast read mode
}
void MMA8653FC_deInit() {
  writeRegister(MMA8653FC_CTRL_REG1, 0x00); // Standby mode
}
void getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
  int16_t x_msb = 0, x_lsb = 0, y_msb = 0, y_lsb = 0, z_msb = 0, z_lsb = 0;
  x_msb = readRegister(MMA8653FC_OUT_X_MSB);
  x_lsb = readRegister(MMA8653FC_OUT_X_LSB);
  y_msb = readRegister(MMA8653FC_OUT_Y_MSB);
  y_lsb = readRegister(MMA8653FC_OUT_Y_LSB);
  z_msb = readRegister(MMA8653FC_OUT_Z_MSB);
  z_lsb = readRegister(MMA8653FC_OUT_Z_LSB);
  *x = x_msb << 8 | x_lsb;
  *y = y_msb << 8 | y_lsb;
  *z = z_msb << 8 | z_lsb;
}
// Data processing and display
bool motionDetected() {
  int8_t abs;
  int8_t x, y;
  // Get acceleration with unmasked sign bit
  x = readRegister(MMA8653FC_OUT_X_MSB);
  y = readRegister(MMA8653FC_OUT_Y_MSB);
  x -= x_offset;
  y -= y_offset;
  abs = sqrt((float)(x * x + y * y));
  if(abs > ACCELERATION_THRESHOLD){
    return true;
  }
  else{
    return false;
  }
}
void dynamicDelay(uint16_t ms) {
  while (0 < ms)
  {
    _delay_ms(1);
    ms--;
  }
}
void dice() {
  uint16_t timeSteps = DICE_TIME_STEPS;
  uint8_t randomeNumber = 0;
  // Set the randome seed
  srand(counter);
  // First dice relatively fast
  for (uint8_t i = 0; i < DICE_STEPS_FIRST_ROUND; i++) {
    dynamicDelay(timeSteps);
    randomeNumber = (rand() % 6) + 1;
    showNumber(randomeNumber);
  }
  // Then dice slower on every loop iteration
  for (uint8_t i = 0; i < DICE_STEPS_SECOND_ROUND; i++) {
    dynamicDelay(timeSteps + DICE_TIME_STEPS_INCREASE);
    randomeNumber = (rand() % 6) + 1;
    showNumber(randomeNumber);
    timeSteps += DICE_TIME_STEPS;
  }
}
float getAngle(float axis, float gierAxis) {
    return (atan(axis / gierAxis) * 4068) / 71; // (radians * 4068) / 71
}
void spritLevel() {
  int16_t x, y, z;
  float roll, nick;
  // First we get raw data from the accelerometer
  getAcceleration(&x, &y, &z);
  // Then we substract the offset from the x and y axis, 
  // to get better relative measurements (full calibration is not implemented)
  x = (x - x_offset);
  y = (y - y_offset);
  if(z == 0)z = 1; // Avoid division by zero
  z = -abs(z); // Sensor is mounted upside down -> abs cause of tilt > 90°
  // The next step is to calculate the angle of the roll and nick
  roll = getAngle((float)x, (float)z);
  nick = getAngle((float)y, (float)z);
  // Now we display the anles with the LED'S
  allLedOff();
  // check if the device is balanced
  if (abs(nick) < ANGLETHRESHOLD && abs(roll) < ANGLETHRESHOLD) {
    PORTA |= (1 << D4);
  }
  // If it's not balanced, we check if we nick or gier
  else {
    if (abs(roll) > abs(nick))
    {
      if (roll < 0) // LED1, LED3, LED6
      {
        if (abs(nick) < ANGLETHRESHOLD)
        {
          PORTA |= (1 << D1);
          PORTA |= (1 << D3);
          PORTA |= (1 << D6);
        }
        else
        {
          if (nick < 0)
            PORTA |= (1 << D1);
          else
            PORTA |= (1 << D6);
        }
      }
      else // lED2, LED5, LED7
      {
        if (abs(nick) < ANGLETHRESHOLD)
        {
          PORTA |= (1 << D2);
          PORTA |= (1 << D5);
          PORTB |= (1 << D7);
        }
        else
        {
          if (nick < 0)
            PORTA |= (1 << D2);
          else
            PORTB |= (1 << D7);
        }
      }
    }
    else
    {
      if (nick < 0) // LED1, LED2
      {
        if (abs(roll) < ANGLETHRESHOLD)
        {
          PORTA |= (1 << D1);
          PORTA |= (1 << D2);
        }
        else
        {
          if (roll < 0)
            PORTA |= (1 << D1);
          else
            PORTA |= (1 << D2);
        }
      }
      else // LED6, LED7
      {
        if (abs(roll) < ANGLETHRESHOLD)
        {
          PORTA |= (1 << D6);
          PORTB |= (1 << D7);
        }
        else
        {
          if (roll < 0)
            PORTA |= (1 << D6);
          else
            PORTB |= 1 << D7;
        }
      }
    }
  }
}
// Calibration sequence after wakeup
void calibrate() {
  int16_t x, y, z;
  int32_t x_increment = 0, y_increment = 0, z_increment = 0;
  // Check if all LED's are working
  allLedOn();
  _delay_ms(CALIBRATION_STEP_DELAY);
  // Check if the accelerometer is working
  showNumber(1);
  _delay_ms(CALIBRATION_STEP_DELAY);
  testI2C_read();
  _delay_ms(CALIBRATION_STEP_DELAY);
  showNumber(2);
  _delay_ms(CALIBRATION_STEP_DELAY);
  // Get the sensor offset x times
  for(uint8_t i = 0; i < OFFSET_CALIBRATION_STEPS; i++) {
    getAcceleration(&x, &y, &z);
    x_increment += x;
    y_increment += y;
    z_increment += z;
    _delay_ms(OFFSET_CALIBRATION_STEP_DELAY);
  }
  // Form the mean value
  x_offset = x_increment / OFFSET_CALIBRATION_STEPS;
  y_offset = y_increment / OFFSET_CALIBRATION_STEPS;
  z_offset = z_increment / OFFSET_CALIBRATION_STEPS;
  // Indicate that the calibration is done
  showHook();
  _delay_ms(CALIBRATION_STEP_DELAY);
  button_pressed++; // causing jump in the loop
}
// Sleep routine
void goToSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  MMA8653FC_deInit();
  deInit();
  _delay_ms(10);
  sleep_mode();
}

// ---------------------------------------------------------------- //
// -- Main Function ----------------------------------------------- //
// ---------------------------------------------------------------- //
int main()
{
  // Main loop.
  init();
  MMA8653FC_init();
  // When battery is put in for the first time, buttton is not pressed.
  // But we want to do the calibration sequence, so we set button_pressed to "1".
  if (button_pressed == 0) button_pressed = 1;
  while (1)
  {
    // Right after wakeup, the button is pressed once, so we do the calibration sequence.
    if (button_pressed == 1) {
      calibrate();
    }
    else {
      // If button is pressed even times (2, 4, 6, ...), we do "dice" mode.
      if (button_pressed % 2) {
        if (motionDetected() == true) {
          dice();
        }
      }
      else {
        // If button pressed odd times, we are in "sprit level" mode
        spritLevel();
      }
      // If button is pressed odd times, we do spirit level mode.
      // Go to sleep if inactivity
      if (counter >= SLEEP_THRESHOLD) {
        goToSleep();
      }
    }
  }
}
// --------------------------------------------------------------- //
// -- Interrupts ------------------------------------------------- //
// --------------------------------------------------------------- //
ISR(INT0_vect)       // button interrupt
{
  cli();                          // disable interrupts, to prevent double counting
  button_pressed++;
  while (!(PINB & (1 << BUTTON))) // as long as button is low
  {
    _delay_ms(10);                // wait for debounce
  }
  counter = 0;                    // reset counter, cause pressing button is external activity from user
  sei();                          // enable interrupts again
  main();                         // reload the main application (causing a mode change from dice to level and vice versa)
}
ISR(TIM0_COMPA_vect) // Sleep timer overflow.
{
  counter++;
}