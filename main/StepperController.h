
#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H
#include <Arduino.h>
#include <Servo.h>

#include "Settings.h"

struct step_plan
{
  unsigned long steps_rate;
  int steps_mask;
  int direction_mask;
  bool led_state;
};

class StepperController
{
private:
  const int step_pin_[N_AXIS];
  const int dir_pin_[N_AXIS];
  const int servo_control_pin_;
  const int en_pin_;
  unsigned long move_time_stamp_;
  unsigned long steps_rate_;
  int steps_counter_[N_AXIS];
  int max_steps_[N_AXIS];
  int min_steps_[N_AXIS];
  const Servo *pen_controller_;
  int servo_value_;


public:
  /**
   * Stepper Controller constructor.
   */
  StepperController(Servo *pen_controller);

  void step(int current_step_mask, int current_direction_mask);


  /**
   * Setting the steppers directions based on the given direction mask.
   *
   * @param current_direction_mask - direction mask (integer representation)
   */
  void set_direction(int current_direction_mask);

  /**
   * Setting the steppers steps rate (i.e. the interval between one step to another)
   *
   * @param steps_rate the time interval (in microseconds) between two consequtive steps.
   */
  void set_steps_rate(unsigned long steps_rate);

  /**
   * Set all steppers in enable/disable mode.
   *
   * @param isEnabled - True to enable, false to disable.
   */
  void set_enable(bool isEnabled);

  /**
   * Set servo angle value (in the range of 0 to 255)
   *
   * @param servo_value - PWM value.
   */
  void set_servo_value(int servo_value);


    /**
   * Get The servo angle value (in the range of 0 to 255)
   * @return int - servo current angle value
   */
  int get_servo_value();

  /**
   * Moving the system one step according to the current movement and direction masks.
   * for each motor, their is a designated bit on the direction mask, and a designated bit
   * on the movement mask.
   *
   * In case that the movement bit is turned on - the motor should preform a step.
   * In case that the direction bit is turned on, the motor should move to the negative direction.
   * (otherwise, it will move to the posotive direction)
   *
   * @param steps_mask - The steps movement mask
   * @param current_direction_mask - The motors direction mask
   */
  void move_step(int steps_mask, int current_direction_mask);

  /**
   * Returns the current steps count (i.e. the position of each axis in steps)
   * @return a pointer to steps_counter_ an array of 2 counter, one per each axis.
   */
  const int *get_steps_count() const;
  // https://stackoverflow.com/questions/10716769/c-difference-between-const-positioning

  /**
   * Set the current steps count to given values. (used mainly for auto-homing phase)
   *
   * @param x_steps value to set on Left strip axis
   * @param y_steps value to set on Right strip axis
   */
  void set_steps_count(int left_steps, int right_steps);

  /**
   * Set the steppers' counters limits - setting maximal and minimal values of steps for each motor.
   * @param left_steps_max max val in mm.
   * @param right_steps_max max val in mm.
   * @param left_steps_min min val in mm.
   * @param right_steps_min min val in mm.
   */
  void set_limits(int left_steps_max, int right_steps_max, int left_steps_min, int right_steps_min);


  /**
   * Verify the current step does not cross the board limits.
   * @param current_steps_mask the step mask to be test aginst
   * @param current_direction_mask the direction 
   * @return true if the step is inside the bounding box, false elsewhwere.
   */
  bool is_in_bounding_box(int current_steps_mask, int current_direction_mask);



  /**
   * Convert the current position from the main coordinate system to the Cartesian system.
   * @param x - x coordinate value to update
   * @param y - y coordinate value to update
   * @param l1 - current L1 length (left strip in mm)
   * @param l2 - current L2 length (right strip in mm)
   * @return true if the step is inside the bounding box, false elsewhwere.
   */
  void convert_to_cartesian(int &x, int &y, const double l1, const double l2);
};

#endif
