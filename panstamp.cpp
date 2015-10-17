/**
 * Copyright (c) 2011 panStamp <contact@panstamp.com>
 * 
 * This file is part of the panStamp project.
 * 
 * panStamp  is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * panStamp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with panStamp; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer
 * Creation date: 03/03/2011
 */

#include "panstamp.h"
#include "commonregs.h"
#include "calibration.h"

#define enableIRQ_GDO0()          attachInterrupt(0, isrGDO0event, FALLING);
#define disableIRQ_GDO0()         detachInterrupt(0);

DEFINE_COMMON_REGINDEX_START()
DEFINE_COMMON_REGINDEX_END()

/**
 * Array of registers
 */
extern REGISTER* regTable[];
extern byte regTableSize;

/**
 * PANSTAMP
 *
 * Class constructor
 */
PANSTAMP::PANSTAMP(void)
{
  statusReceived = NULL;
  repeater = NULL;
}

/**
 * enableRepeater
 *
 * Enable repeater mode
 *
 * 'maxHop'  MAximum repeater count. Zero if omitted
 */
void PANSTAMP::enableRepeater(byte maxHop)
{
  if (repeater == NULL)
  {
    static REPEATER repe;
    repeater = &repe;
    repeater->init(maxHop);
  }

  if (maxHop == 0)
    repeater->stop();
}

/**
 * getRegister
 *
 * Return pointer to register with ID = regId
 *
 * 'regId'  Register ID
 */
REGISTER * getRegister(byte regId)
{
  if (regId >= regTableSize)
    return NULL;

  return regTable[regId]; 
}

/**
 * isrGDO0event
 *
 * Event on GDO0 pin (INT0)
 */
void isrGDO0event(void)
{
  // Disable interrupt
  disableIRQ_GDO0();

  if (panstamp.cc1101.rfState == RFSTATE_RX)
  {
    static CCPACKET ccPacket;
    static SWPACKET swPacket;
    REGISTER *reg;
    static bool eval = true;

    if (panstamp.cc1101.receiveData(&ccPacket) > 0)
    {
      if (ccPacket.crc_ok)
      {
        swPacket = SWPACKET(ccPacket);

        // Repeater enabled?
        if (panstamp.repeater != NULL)
          panstamp.repeater->packetHandler(&swPacket);

        // Smart encryption locally enabled?
        if (panstamp.security & 0x02)
        {
          // OK, then incoming packets must be encrypted too
          if (!(swPacket.security & 0x02))
            eval = false;
        }

        if (eval)
        {
          // Function
          switch(swPacket.function)
          {
            case SWAPFUNCT_CMD:
              // Command not addressed to us?
              if (swPacket.destAddr != panstamp.cc1101.devAddress)
                break;
              // Current version does not support data recording mode
              // so destination address and register address must be the same
              if (swPacket.destAddr != swPacket.regAddr)
                break;
              // Valid register?
              if ((reg = getRegister(swPacket.regId)) == NULL)
                break;
              // Anti-playback security enabled?
              if (panstamp.security & 0x01)
              {
                // Check received nonce
                if (panstamp.nonce != swPacket.nonce)
                {
                  // Nonce missmatch. Transmit correct nonce.
                  reg = getRegister(REGI_SECUNONCE);
                  reg->sendSwapStatus();
                  break;
                }
              }
              // Filter incorrect data lengths
              if (swPacket.value.length == reg->length)
                reg->setData(swPacket.value.data);
              else
                reg->sendSwapStatus();
              break;
            case SWAPFUNCT_QRY:
              // Only Product Code can be broadcasted
              if (swPacket.destAddr == SWAP_BCAST_ADDR)
              {
                if (swPacket.regId != REGI_PRODUCTCODE)
                  break;
              }
              // Query not addressed to us?
              else if (swPacket.destAddr != panstamp.cc1101.devAddress)
                break;
              // Current version does not support data recording mode
              // so destination address and register address must be the same
              if (swPacket.destAddr != swPacket.regAddr)
                break;
              // Valid register?
              if ((reg = getRegister(swPacket.regId)) == NULL)
                break;
              reg->getData();
              break;
            case SWAPFUNCT_STA:
              // User callback function declared?
              if (panstamp.statusReceived != NULL)
                panstamp.statusReceived(&swPacket);
              break;
            default:
              break;
          }
        }
      }
    }
  }
  // Enable interrupt
  enableIRQ_GDO0();
}

/**
 * ISR(WDT_vect)
 *
 * Watchdog ISR. Called whenever a watchdog interrupt occurs
 */
ISR(WDT_vect)
{
}

/**
 * setup_watchdog
 * 
 * 'time'	Watchdog timer value
 */
void PANSTAMP::setup_watchdog(byte time) 
{
  byte bb;

  bb = time & 7;
  if (time > 7)
    bb|= (1<<5);

  bb|= (1<<WDCE);

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);    // Enable Watchdog interrupt
}

/**
 * Timer 2 (RTC) ISR routine
 */
ISR(TIMER2_OVF_vect)
{
}

/**
 * setup_rtc
 *
 * Setup software (Timer 2) RTC
 *
 * 'time'   Timer2 prescaler
 *
 *          RTC_1S = 128 for 1 sec
 *          RTC_2S = 256 for 2 sec
 *          RTC_8S = 1024 for 8 sec
 */
void PANSTAMP::setup_rtc(byte time)
{
  // Set timer 2 to asyncronous mode (32.768KHz crystal)
  ASSR = (1 << AS2);

  TCCR2A = 0x00;  // Normal port operation
  // (256 cycles) * (prescaler) / (32.768KHz clock speed) = N sec
  TCCR2B = time;  // Timer 2 prescaler

  while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {}    // Wait for the registers to be updated    
  TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2);                     // Clear the interrupt flags

  TIMSK2 = 0x01;  // Enable timer2A overflow interrupt
}

/**
 * init
 * 
 * Initialize panStamp board
 */
void PANSTAMP::init() 
{
  // Calibrate internal RC oscillator
  rtcCrystal = rcOscCalibrate();

  // Setup CC1101
  cc1101.init();

  // Security disabled by default
  security = 0;

  // Read periodic Tx interval from EEPROM
  txInterval[0] = EEPROM.read(EEPROM_TX_INTERVAL);
  txInterval[1] = EEPROM.read(EEPROM_TX_INTERVAL + 1);

  delayMicroseconds(50);  

  // Enter RX state
  cc1101.setRxState();

  // Attach callback function for GDO0 (INT0)
  enableIRQ_GDO0();

  // Default values
  nonce = 0;
  systemState = SYSTATE_RXON;
}

/**
 * reset
 * 
 * Reset panStamp
 */
void PANSTAMP::reset() 
{
  // Tell the network that our panStamp is restarting
  systemState = SYSTATE_RESTART;
  getRegister(REGI_SYSSTATE)->sendSwapStatus();

  // Reset panStamp
  wdt_disable();  
  wdt_enable(WDTO_15MS);
  while (1) {}
}

/**
 * sleepWd
 * 
 * Put panStamp into Power-down state during "time".
 * This function uses the internal watchdog timer in order to exit (interrupt)
 * from the power-down state
 * 
 * 'time'	Sleeping time:
 *  WDTO_15MS  = 15 ms
 *  WDTO_30MS  = 30 ms
 *  WDTO_60MS  = 60 ms
 *  WDTO_120MS  = 120 ms
 *  WDTO_250MS  = 250 ms
 *  WDTO_500MS  = 500 ms
 *  WDTO_1S = 1 s
 *  WDTO_2S = 2 s
 *  WDTO_4S = 4 s
 *  WDTO_8S = 8 s
 */
void PANSTAMP::sleepWd(byte time) 
{
  // Power-down CC1101
  cc1101.setPowerDownState();
  // Power-down panStamp
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  setup_watchdog(time);
  delayMicroseconds(10);
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  // Unpower functions
  PRR = 0xFF;
  //power_all_disable();
  //clock_prescale_set(clock_div_8);
  // Enter sleep mode
  sleep_mode();

  // ZZZZZZZZ...

  // Wake-up!!
  wakeUp(false);
}

/**
 * sleepRtc
 * 
 * Put panStamp into Power-down state during "time".
 * This function uses Timer 2 connected to an external 32.768KHz crystal
 * in order to exit (interrupt) from the power-down state
 * 
 * 'time'	Sleeping time:
 *  RTC_250MS  = 250 ms
 *  RTC_500MS  = 500 ms
 *  RTC_1S = 1 s
 *  RTC_2S = 2 s
 *  RTC_8S = 8 s
 */
void PANSTAMP::sleepRtc(byte time) 
{
  // Power-down CC1101
  cc1101.setPowerDownState();
  // Power-down panStamp
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  setup_rtc(time);
  delayMicroseconds(10);
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  // Unpower functions
  PRR = 0xFF;
  // Enter sleep mode
  sleep_mode();

  // ZZZZZZZZ...

  // Wake-up!!
  wakeUp(false);
}

/**
 * wakeUp
 *
 * Wake from sleep mode
 *
 * 'rxOn' Enter RX_ON state after waking up
 */
void PANSTAMP::wakeUp(bool rxOn) 
{
  // Exit from sleep
  sleep_disable();
  //wdt_disable();
  // Re-enable functions
  //clock_prescale_set(clock_div_1);
  power_all_enable();
  // Enable ADC
  ADCSRA |= (1 << ADEN);
  
  // If 32.768 KHz crystal enabled
  if (rtcCrystal)
  {
    // Disable timer2A overflow interrupt
    TIMSK2 = 0x00;
  }

  // Reset CC1101 IC
  cc1101.wakeUp();

  if (rxOn)
    systemState = SYSTATE_RXON;
}

/**
 * goToSleep
 *
 * Sleep whilst in power-down mode. This function currently uses sleepWd in a loop
 */
void PANSTAMP::goToSleep(void)
{
  // Get the amount of seconds to sleep from the internal register
  int intInterval = txInterval[0] * 0x100 + txInterval[1];
  int i, loops;
  byte minTime;
  
  // No interval? Then return
  if (intInterval == 0)
    return;

  // Search the maximum sleep time passed as argument to sleepWd that best
  // suits our desired interval
  if (intInterval % 8 == 0)
  {
    loops = intInterval / 8;
    
    if (rtcCrystal)
      minTime = RTC_8S;
    else
      minTime = WDTO_8S;
  }
  else if (intInterval % 4 == 0)
  {
    if (rtcCrystal)
    {
      loops = intInterval / 2;
      minTime = RTC_2S;
    }
    else
    {
      loops = intInterval / 4;
      minTime = WDTO_4S;
    }
  }
  else if (intInterval % 2 == 0)
  {
    loops = intInterval / 2;
    if (rtcCrystal)    
      minTime = RTC_2S;
    else
      minTime = WDTO_2S;
  }
  else
  {
    loops = intInterval;
    if (rtcCrystal)
      minTime = RTC_1S;
    else
      minTime = WDTO_1S;
  }

  systemState = SYSTATE_RXOFF;

  // Sleep
  for (i=0 ; i<loops ; i++)
  {
    // Exit sleeping loop?
    if (systemState == SYSTATE_RXON)
      break;

    if (rtcCrystal)
      sleepRtc(minTime);
    else
      sleepWd(minTime);
  }
  systemState = SYSTATE_RXON;
}

/**
 * enterSystemState
 *
 * Enter system state
 *
 * 'state'  New system state
 */
void PANSTAMP::enterSystemState(SYSTATE state)
{
  // Enter SYNC mode (full Rx mode)
  byte newState[] = {state};
  regTable[REGI_SYSSTATE]->setData(newState);
}

/**
 * getInternalTemp
 * 
 * Read internal (ATMEGA328 only) temperature sensor
 * Reference: http://playground.arduino.cc/Main/InternalTemperatureSensor
 * 
 * Return:
 * 	Temperature in degrees Celsius
 */
long PANSTAMP::getInternalTemp(void) 
{
  unsigned int wADC;
  long t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}

/**
 * setTxInterval
 * 
 * Set interval for periodic transmissions
 * 
 * 'interval'	New periodic interval. 0 for asynchronous devices
 * 'save'     If TRUE, save parameter in EEPROM
 */
void PANSTAMP::setTxInterval(byte* interval, bool save)
{
  memcpy(txInterval, interval, sizeof(txInterval));

  // Save in EEPROM
  if (save)
  {
    EEPROM.write(EEPROM_TX_INTERVAL, interval[0]);
    EEPROM.write(EEPROM_TX_INTERVAL + 1, interval[1]);
  }
}

/**
 * setSmartPassword
 * 
 * Set Smart Encryption password
 * 
 * 'password'	Encryption password
 */
void PANSTAMP::setSmartPassword(byte* password)
{
  // Save password
  memcpy(encryptPwd, password, sizeof(encryptPwd));
  // Enable Smart Encryption
  security |= 0x02;
}

/**
 * Pre-instantiate PANSTAMP object
 */
PANSTAMP panstamp;

