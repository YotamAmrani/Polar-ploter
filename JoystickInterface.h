#ifndef JOYSTICK_INTERFACE_H
#define JOYSTICK_INTERFACE_H

#include "Settings.h"
#include <Arduino.h>
#include <Servo.h>


class Encoder{
  private:
  const int bit_0_pin_;
  const int bit_1_pin_;
  const int push_button_pin_;
  
  const char encoder_char_;
  unsigned long time_stamp_;
  
  byte prev_encoder_read_;
  byte new_encoder_read_;
  // negative value for clockwise spin  
  
  
  public:
  static bool is_pressed_;
  int last_direction_;
  bool flip_direction_;

  int counter_;
  int prev_counter_;
  int direction_;
  int steps_to_move_;
  Encoder(int bit_0_pin,int bit_1_pin,int push_button_pin, char encoder_char, bool flip_direction_=false);
  
  int read_encoder();
  void set_direction();
  void initialize_encoder();
  void print_current_counter();
  bool is_pressed();
  };



// JoyStick interface
void getMovementMask(int *current_steps_mask, int *current_direction_mask,int *pen_state, Encoder &encoder_a, Encoder &encoder_b);
void read_encoders_buttons();

#endif
