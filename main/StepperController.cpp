#include "StepperController.h"

StepperController::StepperController(Servo *pen_controller) : step_pin_{LEFT_STEP_PIN, RIGHT_STEP_PIN},
                                         dir_pin_{LEFT_DIR_PIN, RIGHT_DIR_PIN},
                                         servo_control_pin_(SERVO_COMMAND_PIN),
                                         en_pin_(EN_PIN), steps_counter_{0, 0},
                                         max_steps_{mm_to_steps(X_MM_LIMIT, X_STEPS_PER_MM), mm_to_steps(Y_MM_LIMIT, Y_STEPS_PER_MM)},
                                         min_steps_{mm_to_steps(X_MM_MIN_LIMIT, X_STEPS_PER_MM), mm_to_steps(Y_MM_MIN_LIMIT, Y_STEPS_PER_MM)},
                                         pen_controller_(pen_controller),servo_value_(0)

{
  // Initializing values
  move_time_stamp_ = micros();
  steps_counter_[LEFT_STRIP_AXIS] = max_steps_[LEFT_STRIP_AXIS];
  steps_counter_[RIGHT_STRIP_AXIS] = max_steps_[RIGHT_STRIP_AXIS];
  steps_rate_ = STEPS_RATE;

  // Declare pins as Outputs
  pinMode(step_pin_[LEFT_STRIP_AXIS], OUTPUT);
  pinMode(dir_pin_[LEFT_STRIP_AXIS], OUTPUT);
  pinMode(step_pin_[RIGHT_STRIP_AXIS], OUTPUT);
  pinMode(dir_pin_[RIGHT_STRIP_AXIS], OUTPUT);
  pinMode(en_pin_, OUTPUT);
  this->set_enable(false);


}

/*    SETTERS    **/
void StepperController::set_steps_rate(unsigned long steps_rate)
{
  if (steps_rate >= 0)
  {
    steps_rate_ = steps_rate;
  }
}

void StepperController::set_enable(bool isEnabled)
{
  digitalWrite(EN_PIN, !isEnabled);
}

void StepperController::set_direction(int current_direction_mask)
{
  // set by the digit bits - if a bit is turned on, flip direction
  current_direction_mask = current_direction_mask ^ DIRECTION_INVERT_MASK;
  digitalWrite(dir_pin_[LEFT_STRIP_AXIS], bit_istrue(current_direction_mask, 1 << LEFT_STRIP_AXIS));
  digitalWrite(dir_pin_[RIGHT_STRIP_AXIS], bit_istrue(current_direction_mask, 1 << RIGHT_STRIP_AXIS));
}

void StepperController::set_steps_count(int x_steps, int y_steps)
{
  steps_counter_[LEFT_STRIP_AXIS] = x_steps;
  steps_counter_[RIGHT_STRIP_AXIS] = y_steps;
}

// TODO
void StepperController::set_servo_value(int servo_value)
{
  if (servo_value != servo_value_){
    int val = (servo_value_ - servo_value);
    int pulse = val/5;
    while(val != 0){
      servo_value_ -= pulse;
      val -= pulse;
      pen_controller_->write(servo_value); 
      delay(150);
    }

    // servo_value_ = servo_value;
    // pen_controller_->write(servo_value); 
    // delay(100);
  }

}

int StepperController::get_servo_value(){
  return servo_value_;
}

//int left_steps_max, int right_steps_max, int left_steps_min, int right_steps_min
void StepperController::set_limits(int left_steps_max, int right_steps_max, int left_steps_min, int right_steps_min){
  max_steps_[LEFT_STRIP_AXIS] = mm_to_steps(left_steps_max, X_STEPS_PER_MM);
  max_steps_[RIGHT_STRIP_AXIS] = mm_to_steps(right_steps_max, Y_STEPS_PER_MM);
  min_steps_[LEFT_STRIP_AXIS] = mm_to_steps(left_steps_min, X_STEPS_PER_MM);
  min_steps_[RIGHT_STRIP_AXIS] = mm_to_steps(right_steps_min, Y_STEPS_PER_MM);
  }

/*    GETTERS    **/
const int *StepperController::get_steps_count() const
{
  return steps_counter_;
}

/*    MOVEMENT METHODS    **/
void StepperController::step(int current_step_mask, int current_direction_mask)
{
#ifdef ENABLE_SOFT_LIMIT
  for (int i = 0; i < N_AXIS; ++i)
  {
    int current_step = bit_to_sign(current_direction_mask, 1 << i) * bit_istrue(current_step_mask, 1 << i);

    if (current_step + steps_counter_[i] <= max_steps_[i] && current_step + steps_counter_[i] >= min_steps_[i])
    {
      digitalWrite(step_pin_[i], bit_istrue(current_step_mask, 1 << i));
      steps_counter_[i] += bit_to_sign(current_direction_mask, 1 << i) * bit_istrue(current_step_mask, 1 << i);
    }
    // if (steps_counter_[i] + 100 > max_steps_[i])
    // {
    //   steps_rate_ = steps_rate_ * (100 - (max_steps_[i] - steps_counter_[i]) + 1);
    // }
  }

#else
  // start of pulse
  digitalWrite(step_pin_[LEFT_STRIP_AXIS], bit_istrue(current_step_mask, 1 << LEFT_STRIP_AXIS));
  digitalWrite(step_pin_[RIGHT_STRIP_AXIS], bit_istrue(current_step_mask, 1 << RIGHT_STRIP_AXIS));
  // count motors steps, flip when direction bit is on
  steps_counter_[LEFT_STRIP_AXIS] += bit_to_sign(current_direction_mask, 1 << LEFT_STRIP_AXIS) * bit_istrue(current_step_mask, 1 << LEFT_STRIP_AXIS);
  steps_counter_[RIGHT_STRIP_AXIS] += bit_to_sign(current_direction_mask, 1 << RIGHT_STRIP_AXIS) * bit_istrue(current_step_mask, 1 << RIGHT_STRIP_AXIS);

#endif
  delayMicroseconds(STEP_PULSE_LENGTH);
  // end pulse
  digitalWrite(step_pin_[LEFT_STRIP_AXIS], LOW);
  digitalWrite(step_pin_[RIGHT_STRIP_AXIS], LOW);
}

void StepperController::move_step(int steps_mask, int current_direction_mask)
{

  // turn sed pulse
  if (steps_mask && (micros() - move_time_stamp_ > steps_rate_))
  {
    set_direction(current_direction_mask);
    this->step(steps_mask, current_direction_mask); // Send step
    move_time_stamp_ = micros();                    // reset timer
    
  }
}


void StepperController::convert_to_cartesian(int &x, int &y, const double l1, const double l2){

  // calculate the current height and width
  /**
  h = ?, w = ?
  1. (dis - w)^2 + h^2 = l1^2
  2. w^2 + h^2 = l2^2
   --> (motors_distance-w)^2 - w^2 = l1^2 - l2^2
   --> motors_distance^2 -2*motors_distance*w  = l1^2 - l2^2
  **/
  
  double w = (l1*l1 - l2*l2 - double(MOTORS_DISTANCE)*MOTORS_DISTANCE)/ (-2*double(MOTORS_DISTANCE));
  double h = sqrt(l2*l2 - w*w);
  x = int(double(MOTORS_DISTANCE) - w + X_OFFSET);
  y = int(-1*(h + Y_OFFSET));
}

bool StepperController::is_in_bounding_box(int current_step_mask, int current_direction_mask){
  int x = 0, y = 0;
  // Extract the current length of the timing strips in MM
  int current_step_l1 = bit_to_sign(current_direction_mask, 1 << LEFT_STRIP_AXIS) * bit_istrue(current_step_mask, 1 << LEFT_STRIP_AXIS);
  int current_step_l2 = bit_to_sign(current_direction_mask, 1 << RIGHT_STRIP_AXIS) * bit_istrue(current_step_mask, 1 << RIGHT_STRIP_AXIS);
  double l1 = (this->get_steps_count()[LEFT_STRIP_AXIS]/X_STEPS_PER_MM) + current_step_l1;
  double l2 = (this->get_steps_count()[RIGHT_STRIP_AXIS]/Y_STEPS_PER_MM) + current_step_l2;
  
  convert_to_cartesian(x, y, l1, l2);

  if(y > MAX_Y_VALUE || y < MIN_Y_VALUE || x > MAX_X_VALUE || x < MIN_X_VALUE){
    Serial.println("out of bound!");
    return false;
  }
  return true;
}
