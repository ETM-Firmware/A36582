#include "P1395_CAN_SLAVE.h"
#include "A36582.h"

void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;

  index_word = message_ptr->word3;
  switch (index_word) 
    {
      /*
	Place all board specific commands here
      */
      
    default:
      local_can_errors.invalid_index++;
      break;
    }
}


void ETMCanSlaveLogCustomPacketC(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_FAST_PREVIOUS_PULSE,
		     global_data_A36582.sample_index,
		     global_data_A36582.imag_internal_adc.reading_scaled_and_calibrated,
		     global_data_A36582.imag_external_adc.reading_scaled_and_calibrated,
		     _STATUS_ARC_DETECTED
		     );
}


void ETMCanSlaveLogCustomPacketD(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_FILTERED_PULSE,
		     0,
		     global_data_A36582.arc_this_hv_on,
		     global_data_A36582.filt_ext_adc_low,
		     global_data_A36582.filt_ext_adc_high
		     );
}


void ETMCanSlaveLogCustomPacketE(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(
		     // DPARKER need to confirm this pointer math
		     ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_ARCS,
		     *((unsigned int*)&global_data_A36582.arc_total + 1),          // This is the high word
		     *((unsigned int*)&global_data_A36582.arc_total),              // This is the low word
		     *((unsigned int*)&global_data_A36582.pulse_this_hv_on + 1),   // This is the high word
		     *((unsigned int*)&global_data_A36582.pulse_this_hv_on)        // This is the high word
		     );
}

void ETMCanSlaveLogCustomPacketF(void) {
  /* 
     Use this to log Board specific data packet
     This will get executed once per update cycle (1.6 seconds) and will be spaced out in time from the other log data
  */
  ETMCanSlaveLogData(ETM_CAN_DATA_LOG_REGISTER_MAGNETRON_MON_SLOW_PULSE_COUNT,
		     *((unsigned int*)&global_data_A36582.pulse_total + 3),        // This is the most significant word
		     *((unsigned int*)&global_data_A36582.pulse_total + 2),        
		     *((unsigned int*)&global_data_A36582.pulse_total + 1),        
		     *((unsigned int*)&global_data_A36582.pulse_total)             // This is the least significant word
		     );
}


  



