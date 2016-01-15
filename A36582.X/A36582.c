// This is firmware for the Magnetron Current Monitor Board


#include "ETM_CRC.h"

#include <libpic30.h>
#include <adc12.h>
#include <xc.h>
#include <timer.h>
#include "A36582.h"
#include "FIRMWARE_VERSION.h"
#include "A36582_SETTINGS.h"


_FOSC(ECIO & CSW_FSCM_OFF); 
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer 
_FBORPOR(PWRT_64 & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);



void InitializeA36582(void);
void DoStateMachine(void);
void DoA36582(void);
void DoStartupLEDs(void);
void DoPostPulseProcess(void);
void ResetPulseLatches(void);
void SavePulseCountersToEEProm(void);

MagnetronCurrentMonitorGlobalData global_data_A36582;
unsigned int eeprom_read_write_failure_count;

int main(void) {
  global_data_A36582.control_state = STATE_STARTUP;
  while (1) {
    DoStateMachine();
  }
}


void DoStateMachine(void) {
  switch (global_data_A36582.control_state) {
    
  case STATE_STARTUP:
    InitializeA36582();
    _CONTROL_NOT_CONFIGURED = 0;
    _CONTROL_NOT_READY = 1;
    global_data_A36582.control_state = STATE_FLASH_LED;
    break;


  case STATE_FLASH_LED:
    _CONTROL_NOT_READY = 1;
    ResetPulseLatches();
    while (global_data_A36582.control_state == STATE_FLASH_LED) {
      DoA36582();
      DoStartupLEDs();
      
      if (global_data_A36582.led_flash_counter >= LED_STARTUP_FLASH_TIME) {
	global_data_A36582.control_state = STATE_OPERATE;
      }
    }
    break;
    
  case STATE_OPERATE:
    global_data_A36582.fast_arc_counter = 0;
    global_data_A36582.slow_arc_counter = 0;
    global_data_A36582.consecutive_arc_counter = 0;
    global_data_A36582.pulse_counter_fast = 0;
    global_data_A36582.pulse_counter_slow = 0;
    global_data_A36582.false_trigger_counter = 0;
    under_current_arc_count = 0;
    over_current_arc_count = 0;
    _FAULT_REGISTER = 0;
    _CONTROL_NOT_READY = 0;
    while (global_data_A36582.control_state == STATE_OPERATE) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
	DoPostPulseProcess();
	global_data_A36582.sample_complete = 0;
      }

      if (_FAULT_REGISTER) {
	global_data_A36582.control_state = STATE_FAULT;
      }
    }
    break;
    
    
  case STATE_FAULT:
    _CONTROL_NOT_READY = 1;
    while (global_data_A36582.control_state == STATE_FAULT) {
      DoA36582();
      if (global_data_A36582.sample_complete) {
	global_data_A36582.sample_complete = 0;
	DoPostPulseProcess();
      }
      
      if (ETMCanSlaveGetSyncMsgResetEnable()) {
	global_data_A36582.control_state = STATE_OPERATE;
      }
    }
    break;
    

  default:
    global_data_A36582.control_state = STATE_FAULT;
    break;
  }
}


void DoA36582(void) {
  ETMCanSlaveDoCan();

  if (ETMCanSlaveGetSyncMsgClearDebug()) {
    arc_this_hv_on = 0;
    global_data_A36582.pulse_this_hv_on = 0;
  }

  if (ETMCanSlaveGetComFaultStatus()) {
    _FAULT_CAN_COMMUNICATION_LATCHED = 1;
  }
  
  if (_T3IF) {
    _T3IF = 0;
    
    // 10ms has passed
    if (global_data_A36582.control_state == STATE_FLASH_LED) {
      global_data_A36582.led_flash_counter++;
    }
    
    // Run at 1 second interval
    global_data_A36582.millisecond_counter += 10;
    if (global_data_A36582.millisecond_counter >= 1000) {
      global_data_A36582.millisecond_counter = 0;
      SavePulseCountersToEEProm();
    }

    // ----------------- UPDATE LOGGING DATA ------------------------ //
    ETMCanSlaveSetDebugRegister(0, global_data_A36582.fast_arc_counter);
    ETMCanSlaveSetDebugRegister(1, global_data_A36582.slow_arc_counter);
    ETMCanSlaveSetDebugRegister(2, global_data_A36582.consecutive_arc_counter);
    //ETMCanSlaveSetDebugRegister(4, global_data_A36582.filt_int_adc_low);
    //ETMCanSlaveSetDebugRegister(5, global_data_A36582.filt_ext_adc_low);
    //ETMCanSlaveSetDebugRegister(6, global_data_A36582.filt_int_adc_high);
    //ETMCanSlaveSetDebugRegister(7, global_data_A36582.filt_ext_adc_high);
    ETMCanSlaveSetDebugRegister(8, global_data_A36582.imag_external_adc.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(9, global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(10, global_data_A36582.pulse_with_no_trigger_counter);
    ETMCanSlaveSetDebugRegister(11, global_data_A36582.minimum_pulse_period_fault_count);
    ETMCanSlaveSetDebugRegister(12, global_data_A36582.false_trigger_counter);
    //ETMCanSlaveSetDebugRegister(13, over_current_arc_count);
    //ETMCanSlaveSetDebugRegister(14, under_current_arc_count);

    *(unsigned long long*)&slave_board_data.log_data[8] = global_data_A36582.pulse_total;
    *(unsigned long*)&slave_board_data.log_data[4] = global_data_A36582.pulse_this_hv_on;
    *(unsigned long*)&slave_board_data.log_data[6] = global_data_A36582.arc_total;
    
    // Update tthe false trigger counter
    global_data_A36582.false_trigger_decrement_counter++;
    if (global_data_A36582.false_trigger_decrement_counter >= FALSE_TRIGGER_DECREMENT_10_MS_UNITS) {
      global_data_A36582.false_trigger_decrement_counter = 0;
      if (global_data_A36582.false_trigger_counter) {
	global_data_A36582.false_trigger_counter--;
      }
    }
    if (global_data_A36582.false_trigger_counter >= FALSE_TRIGGERS_FAULT_LEVEL) {
      _FAULT_FALSE_TRIGGER = 1;
    }
  }
  

  // DPARKER - THIS DID NOT WORK - IT RESET THE LATCHES WHEN THEY SHOULD NOT BE AND CAUSED FALSE ARCS TO BE DETECTED
  // However we may need some way to reset the pulse latches if they are set for a long time
  // If this happens, INT3 will not trigger and we will loose the ability to detect pulses without a trigger
  // Perhaps another counter that if there has been no trigger for the previous second, then if the latches are set they are cleared.
  
  // Alternatively we could check the state of the latches inside the interrupt.  That way the checks can't be broken by this call
  
  /*
  // Reset the pulse latches if they are set and it has been more than 2ms since the last pulse
  if ((TMR2 > TIMER_4_TIME_2_MILLISECONDS) && (PIN_PULSE_OVER_CURRENT_LATCH_4 == ILL_LATCH_SET)) {

    ResetPulseLatches();
    // DPARKER. why doesn't this clear the fault latches before a fault latch check.  You would think that the TMR2 check would prevent this.
  // Perhaps adding a long delay and then reckecking TMR2 before the ResetPulseLatches would fix the problem?
  }
  */
}


void DoStartupLEDs(void) {
  switch ((global_data_A36582.led_flash_counter >> 4) & 0b11) 
    {
      
    case 0:
      _LATA7 = !OLL_LED_ON;
      _LATG12 = !OLL_LED_ON;
      _LATG13 = !OLL_LED_ON;
      break;
      
    case 1:
      _LATA7 = OLL_LED_ON;
      _LATG12 = !OLL_LED_ON;
      _LATG13 = !OLL_LED_ON;
      break;
      
    case 2:
      _LATA7 = OLL_LED_ON;
      _LATG12 = OLL_LED_ON;
      _LATG13 = !OLL_LED_ON;
      break;
      
    case 3:
      _LATA7 = OLL_LED_ON;
      _LATG12 = OLL_LED_ON;
      _LATG13 = OLL_LED_ON;
      break;
      
    }
}


void InitializeA36582(void) {
  unsigned int pulse_data_A[7];
  unsigned int pulse_data_B[7];

  // Initialize the status register and load the inhibit and fault masks
  _CONTROL_REGISTER = 0;
  _FAULT_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;
  
  // Configure Trigger Interrupt
  _INT1IP = 7; // This must be the highest priority interrupt
  _INT1IE = 1;
  
  // Configure the "False Trigger" Interrupt
  _INT3IP = 6; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition
  _INT3IE = 1;

  // By Default, the can module will set it's interrupt Priority to 4
  
  // Initialize all I/O Registers
  TRISA = A36582_TRISA_VALUE;
  TRISB = A36582_TRISB_VALUE;
  TRISC = A36582_TRISC_VALUE;
  TRISD = A36582_TRISD_VALUE;
  TRISF = A36582_TRISF_VALUE;
  TRISG = A36582_TRISG_VALUE;


  // Initialize TMR2
  TMR2  = 0;
  _T2IF = 0;
  T2CON = T2CON_VALUE;

  
  // Initialize TMR3
  PR3   = PR3_VALUE_10_MILLISECONDS;
  TMR3  = 0;
  _T3IF = 0;
  T3CON = T3CON_VALUE;


  
  // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);
  
  // Initialize the Can module
  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_MAGNETRON_CURRENT_BOARD, _PIN_RG13, 4, _PIN_RA7, _PIN_RG12);
  ETMCanSlaveLoadConfiguration(36582, 251, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_BRANCH_REV);
  
  // Initialize the Analog input data structures
  ETMAnalogInitializeInput(&global_data_A36582.imag_internal_adc,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.25075),
			   OFFSET_ZERO,
			   ANALOG_INPUT_0,
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);
  
  ETMAnalogInitializeInput(&global_data_A36582.imag_external_adc,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.25075),
			   OFFSET_ZERO,
			   ANALOG_INPUT_1, 
			   NO_OVER_TRIP,
			   NO_UNDER_TRIP,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36582.analog_input_5v_mon,
			   MACRO_DEC_TO_SCALE_FACTOR_16(.12500),
			   OFFSET_ZERO,
			   ANALOG_INPUT_NO_CALIBRATION,
			   PWR_5V_OVER_FLT,
			   PWR_5V_UNDER_FLT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);


  // Configure SPI port, used by External ADC
  ConfigureSPI(ETM_SPI_PORT_2, ETM_DEFAULT_SPI_CON_VALUE, ETM_DEFAULT_SPI_CON2_VALUE, ETM_DEFAULT_SPI_STAT_VALUE, SPI_CLK_2_MBIT, FCY_CLK);
 
 
  //Initialize the internal ADC for Startup Power Checks
  // ---- Configure the dsPIC ADC Module ------------ //
  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O

  ADCON1 = ADCON1_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_STARTUP;     // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING_STARTUP;      // Configure the high speed ADC module based on H file parameters
  //ADCSSL = ADCSSL_SETTING_STARTUP;
  
  _ADIF = 0;
  _ADON = 1;

  while (_ADIF == 0); // Wait for 16 ADC conversions to complete;
  _ADON = 0;
  global_data_A36582.analog_input_5v_mon.filtered_adc_reading  = ADCBUF0 + ADCBUF1 + ADCBUF2 +ADCBUF3 + ADCBUF4 + ADCBUF5 + ADCBUF6 + ADCBUF7;
  global_data_A36582.analog_input_5v_mon.filtered_adc_reading += ADCBUF8 + ADCBUF9 + ADCBUFA +ADCBUFB + ADCBUFC + ADCBUFD + ADCBUFE + ADCBUFF;

  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.analog_input_5v_mon);

  if (ETMAnalogCheckOverAbsolute(&global_data_A36582.analog_input_5v_mon)) {
    _CONTROL_SELF_CHECK_ERROR = 1;
    // DPARKER use the self test bits
  }
  
  if (ETMAnalogCheckUnderAbsolute(&global_data_A36582.analog_input_5v_mon)) {
    _CONTROL_SELF_CHECK_ERROR = 1;
    // DPARKER use the self test bits
  }
  
  ADCON1 = ADCON1_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING_OPERATE;     // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING_OPERATE;      // Configure the high speed ADC module based on H file parameters
  //ADCSSL = ADCSSL_SETTING_STARTUP;
  
  _ADIF = 0;
  _ADON = 1;
  _SAMP = 1;


  // Read Data from EEPROM
  ETMEEPromReadPage(PULSE_COUNT_REGISTER_A, 7, &pulse_data_A[0]);
  ETMEEPromReadPage(PULSE_COUNT_REGISTER_B, 7, &pulse_data_B[0]);
  
  // If the data checks out, update with data
  if (pulse_data_A[6] == ETMCRC16(pulse_data_A, 12)) {
    global_data_A36582.arc_total = *(unsigned long*)&pulse_data_A[0];
    global_data_A36582.pulse_total = *(unsigned long long*)&pulse_data_A[2];
  } else if (pulse_data_B[6] == ETMCRC16(pulse_data_B, 12)) {
    global_data_A36582.arc_total = *(unsigned long*)&pulse_data_B[0];
    global_data_A36582.pulse_total = *(unsigned long long*)&pulse_data_B[2];
  } else {
    // Both EEPROM Registers were corrupted
    global_data_A36582.arc_total = *(unsigned long*)&pulse_data_A[0];
    global_data_A36582.arc_total |= 0x80000000; // Set the highest bit high to indicate an EEPROM reading error
    global_data_A36582.pulse_total = *(unsigned long long*)&pulse_data_B[2];
    global_data_A36582.pulse_total = 0x8000000000000000; // Set the highest bit high to indicate an EEPROM reading error
  }
  
  // Run a dummy conversion
  _SAMP = 0;

}

void SavePulseCountersToEEProm(void) {
  unsigned int test_data[7];

  global_data_A36582.count_crc = ETMCRC16(&global_data_A36582.arc_total, 12);

  if(global_data_A36582.next_register) {
    // Write to "Page A" 

    // Write Data to EEPROM
    ETMEEPromWritePage(PULSE_COUNT_REGISTER_A, 7, (unsigned int*)&global_data_A36582.arc_total);

    // Read Data from EEPROM
    ETMEEPromReadPage(PULSE_COUNT_REGISTER_A, 7, &test_data[0]);
    
    // If the data checks out, update next register
    if ((test_data[6] == global_data_A36582.count_crc) && (test_data[6] == ETMCRC16(test_data, 12))) {
      // The data was correctly written to the EEPROM
      global_data_A36582.next_register = 0;
    }
 
  } else  {
    // Write to "Page B"
    
    // Write Data to EEPROM
    ETMEEPromWritePage(PULSE_COUNT_REGISTER_B, 7, (unsigned int*)&global_data_A36582.arc_total);
    
    // Read Data from EEPROM
    ETMEEPromReadPage(PULSE_COUNT_REGISTER_B, 7, &test_data[0]);
    
    // If the data checks out, update next register
    if ((test_data[6] == global_data_A36582.count_crc) && (test_data[6] == ETMCRC16(test_data, 12))) {
      // The data was correctly written to the EEPROM
      global_data_A36582.next_register = 1;
    }
  }
  
}


void DoPostPulseProcess(void) {
    // Process the pulse data
  
  // Wait 40us for the conversions to complete (and the noise from the arc to dissipate)
  //__delay32(400);
  
  
  // Read the analog current level from internal ADC
  // DPARKER this should be ~zero with the new timing strategy
  global_data_A36582.imag_internal_adc.filtered_adc_reading = (ADCBUF0 << 4);
  //_LATF6 = 0;

  // Scale the readings
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_internal_adc);
  ETMAnalogScaleCalibrateADCReading(&global_data_A36582.imag_external_adc);
  
  global_data_A36582.arc_this_pulse = 0;

  // DPARKER Consider checking the analog current reading to also look for arc
  if ((PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) || (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET)) {
    // The current after the trigger was too high or too low
    _NOTICE_ARC_DETECTED = 1;
    global_data_A36582.arc_this_pulse = 1;
    
    if (PIN_PULSE_OVER_CURRENT_LATCH_1 == ILL_LATCH_SET) {
      over_current_arc_count++;
    }
    if (PIN_PULSE_OVER_CURRENT_LATCH_4 != ILL_LATCH_SET) {
      under_current_arc_count++;
    }

    global_data_A36582.arc_total++;
    arc_this_hv_on++;
    global_data_A36582.fast_arc_counter++;
    global_data_A36582.slow_arc_counter++;
    global_data_A36582.consecutive_arc_counter++;
    
  } else {
    if (global_data_A36582.consecutive_arc_counter) { 
      global_data_A36582.consecutive_arc_counter--;
    }
    
    // Filter the ADC current readings
    _NOT_LOGGED_HIGH_ENERGY = global_data_A36582.sample_energy_mode;
    if (global_data_A36582.sample_energy_mode) {
      filt_int_adc_high = RCFilterNTau(filt_int_adc_high,
				       global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
				       RC_FILTER_64_TAU);
      
      filt_ext_adc_high = RCFilterNTau(filt_ext_adc_high,
				       global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
				       RC_FILTER_64_TAU);
    } else {
      filt_int_adc_low = RCFilterNTau(filt_int_adc_low,
				      global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
				      RC_FILTER_64_TAU);
      
      filt_ext_adc_low = RCFilterNTau(filt_ext_adc_low,
				      global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
				      RC_FILTER_64_TAU);
    }
  }
  
  global_data_A36582.pulse_total++;
  global_data_A36582.pulse_this_hv_on++;
	
  // Decrement fast_arc_counter if needed
  global_data_A36582.pulse_counter_fast++;
  if (global_data_A36582.pulse_counter_fast > ARC_COUNTER_FAST_DECREMENT_INTERVAL) {
    global_data_A36582.pulse_counter_fast = 0;
    if (global_data_A36582.fast_arc_counter) {
      global_data_A36582.fast_arc_counter--;
    }
  }
	
  // Decrement slow_arc_counter if needed
  global_data_A36582.pulse_counter_slow++;
  if (global_data_A36582.pulse_counter_slow > ARC_COUNTER_SLOW_DECREMENT_INTERVAL) {
    global_data_A36582.pulse_counter_slow = 0;
    if (global_data_A36582.slow_arc_counter) {
      global_data_A36582.slow_arc_counter--;
    }
  }

  // Look for ARC faults
  if (global_data_A36582.slow_arc_counter >= ARC_COUNTER_SLOW_MAX_ARCS) {
    _FAULT_ARC_SLOW = 1;
  }

  if (global_data_A36582.fast_arc_counter >= ARC_COUNTER_FAST_MAX_ARCS) {
    _FAULT_ARC_FAST = 1;
  }
	
  if (global_data_A36582.consecutive_arc_counter >= ARC_COUNTER_CONSECUTIVE_MAX) {
    _FAULT_ARC_CONTINUOUS = 1;
  }
	
  // Reset the Latches
  ResetPulseLatches();
  if (ETMCanSlaveGetSyncMsgHighSpeedLogging()) {
    ETMCanSlaveLogPulseData(ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_LOG_0,
			    global_data_A36582.sample_index,
			    global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
			    global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
			    global_data_A36582.arc_this_pulse
			    );
  }
}



void ResetPulseLatches(void) {
  PIN_PULSE_LATCH_RESET = OLL_RESET_LATCHES;
  __delay32(20);
  PIN_PULSE_LATCH_RESET = !OLL_RESET_LATCHES;
}


//void __attribute__((interrupt, shadow, no_auto_psv)) _INT1Interrupt(void) {
void __attribute__((__interrupt__(__preprologue__("BCLR ADCON1, #1")), no_auto_psv)) _INT1Interrupt(void) {
  /*
    A sample trigger has been received
  */ 
  /*
  Nop(); //100ns
  Nop(); //200ns
  Nop(); //300ns
  Nop(); //400ns
  Nop(); //500ns
  Nop(); //600ns
  Nop(); //700ns
  Nop(); //800ns
  Nop(); //900ns
  Nop(); //1000ns
  */
  // Trigger the internal ADC to start conversion
  //_SAMP = 0;  // There Appears to be a delay of ~3 ADC Clocks between this and the sample being held (and the conversion starting)
              // I think the ADC clock is running if the ADC is on, therefor we have a sampleing error of up to +/- 1/2 ADC Clock.  Ugh!!!
  //_LATF6 = 1;
  // DPARKER delay until we are in the middle of the current pulse to sample

  // DPARKER these functions may mess with the timing
  global_data_A36582.sample_energy_mode = ETMCanSlaveGetPulseLevel();
  global_data_A36582.sample_index = ETMCanSlaveGetPulseCount();
  global_data_A36582.sample_complete = 1;

  // Check that there was enough time between pulses
  //
  if ((TMR2 <= MINIMUM_PULSE_PERIOD_T2) && (_T2IF == 0)) {
    global_data_A36582.minimum_pulse_period_fault_count++;
  }
  _T2IF = 0;
  TMR2 = 0;
  

  // Wait for the pulse energy to dissipate
  __delay32(150);

  // Read the data from port
  PIN_OUT_TP_F = 1;
  PIN_ADC_CHIP_SELECT = OLL_ADC_SELECT_CHIP;
  global_data_A36582.imag_external_adc.filtered_adc_reading = SendAndReceiveSPI(0, ETM_SPI_PORT_2);
  PIN_ADC_CHIP_SELECT = !OLL_ADC_SELECT_CHIP;
  PIN_OUT_TP_F = 0;

  _INT1IF = 0;
}
  

/*
  Figure out if there was a pulse without a trigger
  The trigger pulse is a sample pulse so it comes in the middle of current pulse.
  We should wait for 10us After this is entered.  If there has not been a trigger pulse durring that time 
  then it was a false tirgger
*/
void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void) {
  // There was trigger on INT3
  _INT3IF = 0;
  __delay32(100); // wait for 10us

  if (!global_data_A36582.sample_complete) {
    // There was a current pulse without a sample trigger within the next 10us
    global_data_A36582.pulse_with_no_trigger_counter++;
    global_data_A36582.false_trigger_counter++;
    ResetPulseLatches();
  }   
}


void __attribute__((interrupt, no_auto_psv)) _DefaultInterrupt(void) {
  // Clearly should not get here without a major problem occuring
  // DPARKER do something to save the state into a RAM location that is not re-initialized and then reset
  Nop();
  Nop();
  __asm__ ("Reset");
}


void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
     
    case 0x2200:
      global_data_A36582.arc_total = 0;
      global_data_A36582.pulse_total = 0;
      global_data_A36582.pulse_this_hv_on = 0;
      arc_this_hv_on = 0;
      break;
 
    default:
      //local_can_errors.invalid_index++;
      break;
    }
}
