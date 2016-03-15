/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/


/* RC-Servo PWM modification: switch between 0.6ms and 2.5ms pulse-width at 61Hz
   Prescaler 1024 = 15625Hz / 256Steps =  61Hz	64µs/step -> Values 15 / 32 for 1ms / 2ms
   Reload value = 0x07
   Replace this file in C:\Program Files (x86)\Arduino\libraries\GRBL
*/


#include "grbl.h"

#define RC_SERVO_SHORT     9//15       // Timer ticks for 0.6ms pulse duration  (9 for 0.6ms)
#define RC_SERVO_LONG      39//32       // Timer ticks for 2.5 ms pulse duration  (39 for 2.5ms)
//#define RC_SERVO_INVERT     1     // Uncomment to invert servo direction


#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
#endif


void spindle_init()
{
  #ifdef VARIABLE_SPINDLE
    // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
    // combined unless configured otherwise.
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
    SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #else
      #ifndef ENABLE_DUAL_AXIS
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
      #endif
    #endif
    pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  #else
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #ifndef ENABLE_DUAL_AXIS
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    #endif
  #endif

  spindle_stop();
}


uint8_t spindle_get_state()
{
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      // No spindle direction output pin. 
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #else
        if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
      #endif
    #else
      if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
        #ifdef ENABLE_DUAL_AXIS
          return(SPINDLE_STATE_CW);
        #else
          if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        #endif
      }
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { 
    #else
      if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
    #endif
      #ifdef ENABLE_DUAL_AXIS    
        return(SPINDLE_STATE_CW);
      #else
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      #endif
    }
  #endif
  return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{     // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
       #ifdef RC_SERVO_INVERT
          OCR_REGISTER = RC_SERVO_LONG;
      #else
          OCR_REGISTER = RC_SERVO_SHORT;
      #endif
}


void spindle_run(uint8_t direction, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Empty planner buffer to ensure spindle is set when programmed.
  protocol_auto_cycle_start();  //temp fix for M3 lockup
  protocol_buffer_synchronize();

  if (direction == SPINDLE_DISABLE) {

  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {
	#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN

    if (direction == SPINDLE_ENABLE_CW) {
      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
    } else {
      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
    }
	#endif

	#ifdef VARIABLE_SPINDLE

      // TODO: Install the optional capability for frequency-based output for servos.
      #define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
      #define RC_SERVO_RANGE (RC_SERVO_LONG-RC_SERVO_SHORT)

	  #ifdef CPU_MAP_ATMEGA2560
      	TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x07 | (1<<WAVE2_REGISTER) | (1<<WAVE3_REGISTER); // set to 1/1024 Prescaler
        OCR4A = 0xFFFF; // set the top 16bit value
        uint16_t current_pwm;
	  #else
        TCCRA_REGISTER = (1<<COMB_BIT) | (1<<WAVE1_REGISTER) | (1<<WAVE0_REGISTER);
        TCCRB_REGISTER = (TCCRB_REGISTER & 0b11111000) | 0x07; // set to 1/1024 Prescaler
	    uint8_t current_pwm;
	  #endif

	   if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; }
      else { 
        rpm -= SPINDLE_MIN_RPM; 
        if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
      }

      #ifdef RC_SERVO_INVERT
          current_pwm = floor( RC_SERVO_LONG - rpm*(RC_SERVO_RANGE/SPINDLE_RPM_RANGE));
          OCR_REGISTER = current_pwm;
      #else
         current_pwm = floor( rpm*(RC_SERVO_RANGE/SPINDLE_RPM_RANGE) + RC_SERVO_SHORT);
          OCR_REGISTER = current_pwm;
      #endif
	  #ifdef MINIMUM_SPINDLE_PWM
        if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
	     OCR_REGISTER = current_pwm;
      #endif
    #endif
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}
void spindle_set_state(uint8_t state, float rpm){
}
