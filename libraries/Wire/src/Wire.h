/*
  TwoWire.h - TWI/I2C library for Arduino & Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  Modified 2012 by Todd Krein (todd@krein.org) to implement repeated starts
  Modified December 2014 by Ivan Grokhotkov (ivan@esp8266.com) - esp8266 support
  Modified April 2015 by Hrsto Gochkov (ficeto@ficeto.com) - alternative esp8266 support
  Modified November 2017 by Chuck Todd <stickbreaker on GitHub> to use ISR and increase stability.
*/

#ifndef TwoWire_h
#define TwoWire_h

#include <esp32-hal.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "Stream.h"

#define STICKBREAKER V1.0.1
#define I2C_BUFFER_LENGTH 128
typedef void(*user_onRequest)(void);
typedef void(*user_onReceive)(uint8_t*, size_t);

static uint8_t rxBuffer[I2C_BUFFER_LENGTH];
static uint16_t rxIndex;
static uint16_t rxLength;
static uint16_t rxQueued; //@stickBreaker

static uint8_t txBuffer[I2C_BUFFER_LENGTH];
static uint16_t txIndex;
static uint16_t txLength;
static uint16_t txAddress;
static uint16_t txQueued; //@stickbreaker

static uint8_t transmitting;

class TwoWire: public Stream
{
private:
    uint8_t num;
    int8_t sda;
    int8_t scl;
    i2c_t * i2c;
    i2c_err_t last_error; // @stickBreaker from esp32-hal-i2c.h
    uint16_t _timeOutMillis;

public:

  static void (*user_onRequest)(void);
  static void (*user_onReceive)(size_t);
  static void onRequestService(void);
  static void onReceiveService(uint8_t*, size_t);

    TwoWire(uint8_t bus_num);
    ~TwoWire();
    bool begin(int sda=-1, int scl=-1, uint32_t frequency=0); // returns true, if successful init of i2c bus
    bool begin(uint16_t address, int sda, int scl, uint32_t frequency);

    void setClock(uint32_t frequency); // change bus clock without initing hardware
    size_t getClock(); // current bus clock rate in hz

    void setTimeOut(uint16_t timeOutMillis); // default timeout of i2c transactions is 50ms
    uint16_t getTimeOut();

    uint8_t lastError();
    char * getErrorText(uint8_t err);

    i2c_err_t writeTransmission(uint16_t address, uint8_t* buff, uint16_t size, bool sendStop=true);
    i2c_err_t readTransmission(uint16_t address, uint8_t* buff, uint16_t size, bool sendStop=true, uint32_t *readCount=NULL);
    void beginTransmission(uint16_t address);
    void beginTransmission(uint8_t address);
    void beginTransmission(int address);
    uint8_t endTransmission(bool sendStop);
    uint8_t endTransmission(void);

    uint8_t requestFrom(uint16_t address, uint8_t size, bool sendStop);
    uint8_t requestFrom(uint16_t address, uint8_t size, uint8_t sendStop);
    uint8_t requestFrom(uint16_t address, uint8_t size);
    uint8_t requestFrom(uint8_t address, uint8_t size, uint8_t sendStop);
    uint8_t requestFrom(uint8_t address, uint8_t size);
    uint8_t requestFrom(int address, int size, int sendStop);
    uint8_t requestFrom(int address, int size);

    size_t write(uint8_t);
    size_t write(const uint8_t *, size_t);
    int available(void);
    int read(void);
    int peek(void);
    void flush(void);
    void onReceive( void (*)(size_t) );
    void onRequest( void (*)(void) );

    inline size_t write(const char * s)
    {
        return write((uint8_t*) s, strlen(s));
    }
    inline size_t write(unsigned long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t)n);
    }

    uint32_t setDebugFlags( uint32_t setBits, uint32_t resetBits);
    bool busy();
};

extern TwoWire Wire;
extern TwoWire Wire1;


/*
V1.0.2 30NOV2018 stop returning I2C_ERROR_CONTINUE on ReSTART operations, regain compatibility with Arduino libs
V1.0.1 02AUG2018 First Fix after release, Correct ReSTART handling, change Debug control, change begin()
  to a function, this allow reporting if bus cannot be initialized, Wire.begin() can be used to recover
  a hung bus busy condition.
V0.2.2 13APR2018 preserve custom SCL,SDA,Frequency when no parameters passed to begin()
V0.2.1 15MAR2018 Hardware reset, Glitch prevention, adding destructor for second i2c testing
*/
#endif
