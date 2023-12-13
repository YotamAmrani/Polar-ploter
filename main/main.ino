#include <Arduino.h>
#include "StepperController.h"
#include "Planner.h"
#include "JoystickInterface.h"
#include "DrawingObjects.h"
#include "Settings.h"

// DEFINITIONS:
void print_current_position();

sys_state state = {IDLE, micros()};
Servo pen_controller; 
StepperController stepper_c = StepperController(&pen_controller);
int current_steps_mask = 0;
int current_direction_mask = 0;
int target[N_INSTRUCTIONS] = {0, 0, 0};
const int *current_position = stepper_c.get_steps_count();
segment_plan seg_p = {0};
Planner pl = Planner(&stepper_c, &seg_p);
int current_drawing = 0;

// USER INTERFACE OBJECTS
Encoder encoder_a = Encoder(ENCODER_A_BIT_0, ENCODER_A_BIT_1, ENCODER_A_BUTTON,'A');
Encoder encoder_b = Encoder(ENCODER_B_BIT_0, ENCODER_B_BIT_1, ENCODER_B_BUTTON,'B',true);
unsigned long test_timer = 0;
int pen_state = PEN_OFF;

void state_handler(int current_steps_mask, int pen_state, StepperController *stepper_c)
{
    // if movement was deteced
    if (current_steps_mask ||  pen_state)
    {

        stepper_c->set_enable(true);
        if (state.sys_mode == IDLE)
        {
            state.sys_mode = MOVE;
            // toggle_led(true); // turn led on
        }
        else if (state.sys_mode == PRINT)
        {
            state.sys_mode = MOVE;
            // toggle_led(true); // turn led on
            // reset the current state
            // change to move state
        }
        state.last_move_time_stamp = micros();
    }
    else
    {
        if (state.sys_mode == MOVE && (micros() - state.last_move_time_stamp) > PEN_DEBOUNCE_TIME)
        {
            state.sys_mode = IDLE;
            // stepper_c->set_enable(false);
            // toggle_led(false); // turn led off
        }
        else if (state.sys_mode == PRINT && pl.is_drawing_finished())
        {
            Serial.println("--LOG: Changing state to IDLE");
            state.sys_mode = IDLE;
            // toggle_led(false);
            state.last_move_time_stamp = micros();
        }
    }
}

void toggle_pen_state(StepperController *stepper_c,int pen_state)
{
  if(pen_state){
    int new_val = PEN_ON;
    if(stepper_c->get_servo_value() == PEN_ON){
      new_val = PEN_OFF;
    }
    stepper_c->set_servo_value(new_val);
  }

}

void auto_homing(StepperController *stepper_c)
{

    Serial.println("Auto homing! ");
    stepper_c->set_steps_rate(AUTO_HOME_STEPS_RATE);
    stepper_c->set_enable(true);
    stepper_c->set_servo_value(PEN_OFF);

    stepper_c->set_steps_count((LEFT_STRIP_INIT_MAX_LENGTH_MM-250)*LEFT_STEPS_PER_MM, (RIGHT_STRIP_INIT_MAX_LENGTH_MM-250)*RIGHT_STEPS_PER_MM);
    while(stepper_c->get_steps_count()[LEFT_STRIP_AXIS] < (LEFT_STRIP_INIT_MAX_LENGTH_MM)*LEFT_STEPS_PER_MM){
      stepper_c->move_step(3, 0);
    }
    

    while(digitalRead(LEFT_LIMIT_SW_PIN) && digitalRead(RIGHT_LIMIT_SW_PIN)){
      stepper_c->move_step(3, 3);
      Serial.println("XY");
      
    }
    
    while(digitalRead(LEFT_LIMIT_SW_PIN)){
      stepper_c->move_step(1, 1);
      Serial.println("X");
    }
    Serial.println("Moved X UP.");

    while(digitalRead(RIGHT_LIMIT_SW_PIN)){
      stepper_c->move_step(2, 2);
      Serial.println("Y");
    }
    Serial.println("Moved Y UP.");
    print_current_position();

    stepper_c->set_steps_count(520*LEFT_STEPS_PER_MM,520*RIGHT_STEPS_PER_MM);
    
    while(stepper_c->get_steps_count()[LEFT_STRIP_AXIS] < 590*LEFT_STEPS_PER_MM || stepper_c->get_steps_count()[RIGHT_STRIP_AXIS] < 590*RIGHT_STEPS_PER_MM){
      stepper_c->move_step(3, 0);
    }

    Serial.println("Moved X axis to place.");
    Serial.println("Moved Y axis to place.");
    stepper_c->set_limits(LEFT_STRIP_MAX_LENGTH_MM, RIGHT_STRIP_MAX_LENGTH_MM, LEFT_STRIP_MIN_LENGTH_MM,RIGHT_STRIP_MIN_LENGTH_MM);

    // Move to starting position with offsets 
    stepper_c->set_enable(true);

    stepper_c->set_steps_rate(STEPS_RATE);
    Serial.println("Auto homing completed successfully! ");
    print_current_position();
}

void test_draw(StepperController *stepper_c){
    stepper_c->set_enable(true);
    int counter = 0;
    for(int i = 0; i< 2500; i++){
      if(counter > 2500){
        stepper_c->step(3, 0);
        counter -= 2500;
      }
      else{
        stepper_c->step(1, 0);
      }
      counter += 1900;
      delayMicroseconds(1000);
    }
}



void convert_to_polar(double x,double y,int &l1, int &l2) {
  double h = (-1*y) - Y_OFFSET;
  double w = -1*(x - X_OFFSET - double(MOTORS_DISTANCE));

  l2 = int(sqrt(h*h+w*w))*LEFT_STEPS_PER_MM;
  w = w - double(MOTORS_DISTANCE);
  l1 = int(sqrt(h*h+w*w))*RIGHT_STEPS_PER_MM;
}

void print_current_position()
{
    Serial.println("Position: ");
    Serial.print(stepper_c.get_steps_count()[LEFT_STRIP_AXIS]);
    Serial.print(",");
    Serial.println(stepper_c.get_steps_count()[RIGHT_STRIP_AXIS]);
}


void print_current_coordinate_position(){
  int x = 0;
  int y = 0;
  stepper_c.convert_to_cartesian(x,y, double(stepper_c.get_steps_count()[LEFT_STRIP_AXIS]/LEFT_STEPS_PER_MM), double(stepper_c.get_steps_count()[RIGHT_STRIP_AXIS]/RIGHT_STEPS_PER_MM));
  Serial.print("X coordinate: ");
  Serial.print(x);
  Serial.print(",");
  Serial.print("Y coordinate: ");
  Serial.println(y);
  int l1 = 0, l2 = 0;
  convert_to_polar(x,y,l1,l2);
  Serial.print("left strip length(steps)");
  Serial.print(l1);
  Serial.print(",");
  Serial.print("right strip length(steps)");
  Serial.println(l2);
}

void initialize_auto_print(int *current_drawing)
{

    // running_time = micros();
    pl.reset_drawing();
    pl.load_drawing(&drawings[*current_drawing]);
    // toggle_led(true);
    stepper_c.set_enable(true);
    state.sys_mode = PRINT;
    Serial.println("--LOG: Changing state to PRINT");

    (*current_drawing) = (*current_drawing + 1) % NUMBER_OF_DRAWINGS;
}

void setup()
{

    Serial.begin(115200);
    /** Init Joystick input pins **/
    /** AUTO HOME**/
    pen_controller.attach(SERVO_COMMAND_PIN);

    auto_homing(&stepper_c);
    // test_draw(&stepper_c);
    state.sys_mode = IDLE;
    pinMode(LEFT_LIMIT_SW_PIN, INPUT_PULLUP);  
    pinMode(RIGHT_LIMIT_SW_PIN, INPUT_PULLUP);  
}

void loop()
{

  test_timer = micros();
    /** GET INPUT MASK **/
    current_steps_mask = 0;
    current_direction_mask = 0;
    pen_state = 0;
    
    getMovementMask(&current_steps_mask,&current_direction_mask, &pen_state, encoder_a,encoder_b);

    if(pen_state){
      print_current_position();
      print_current_coordinate_position();
    }
    
    state_handler(current_steps_mask, pen_state, &stepper_c);

    switch (state.sys_mode)
    {
    case MOVE:
        toggle_pen_state(&stepper_c, pen_state);
        if(stepper_c.is_in_bounding_box(current_steps_mask, current_direction_mask)){
          stepper_c.move_step(current_steps_mask, current_direction_mask);
        }
        
        break;
    case PRINT:
        pl.plot_drawing();
        break;
    case IDLE:
        if(micros() - state.last_move_time_stamp > PEN_PENDING_TIME && stepper_c.get_servo_value() == PEN_ON ){
            toggle_pen_state(&stepper_c, true);
        }
        if (micros() - state.last_move_time_stamp > PENDING_TIME)
        {
            initialize_auto_print(&current_drawing);
        }
        break;
    default:
        break;
    }
   test_timer = micros() - test_timer;
//   Serial.println(test_timer);
}
