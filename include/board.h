#ifndef BOARD_H20220718
#define BOARD_H20220718

// Functions to manipulate the physical hardware on the PCB
namespace board {
void setup_gpio(); // Set correct pin modes, and default states where applicable
void power_on();   // Turn on the 3V3_Switched rail
void power_off();  // Turn off the 3V3_Switched rail
void open_valves();  // Open valves (blocks for ~600ms)
void close_valves(); // Close valves (blocks for ~600ms)
void fan_on();       // Turn fan on
void fan_off();      // Turn fan off
void fan_PWM_setup();
void fan_on_PWM(double mult);
void pump_on();
void pump_off();
} // namespace board

#endif
