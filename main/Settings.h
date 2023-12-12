#ifndef SETTINGS_H
#define SETTINGS_H

// Instructions indices
#define LEFT_STRIP_AXIS 0
#define RIGHT_STRIP_AXIS 1
#define SERVO_ANGLE 2

#define N_AXIS 2
#define N_INSTRUCTIONS 3
#define bit_istrue(x, mask) ((x & mask) != 0)
#define bit_to_sign(x, mask) (((x & mask) != 0) ? -1 : 1)
#define DIRECTION_INVERT_MASK 3

// LED
#define LED_DEBOUNCE_TIME 300

// STEPERS CONFIGURATIONS - steps pins and directions pins of left/right timing strips.
#define LEFT_STEP_PIN 5
#define RIGHT_STEP_PIN 6
#define LEFT_DIR_PIN 2
#define RIGHT_DIR_PIN 3
#define EN_PIN 8

// LIMIT SWITCHES
#define LEFT_LIMIT_SW_PIN 10
#define RIGHT_LIMIT_SW_PIN 11

// ENCODERS PINS
#define ENCODER_A_BIT_1 (A1) // input IO for gray code bit 0 
#define ENCODER_A_BIT_0 (12) // input IO for gray code bit 1
#define ENCODER_B_BIT_1 (A0) // input IO for gray code bit 0
#define ENCODER_B_BIT_0 (9) // input IO for gray code bit 1
#define ENCODER_A_BUTTON (A3)
#define ENCODER_B_BUTTON (A2) 
#define ENCODER_READ_INTERVAL 200 //microseconds
#define STEPS_PER_CLICK 300
#define DIRECTION_CHANGE_DELAY 40

// SERVO 
#define SERVO_COMMAND_PIN (13) 
#define PEN_ON 0
#define PEN_OFF 90


// SYSTEM CONFIGURATIONS
#define STEP_PULSE_LENGTH 20
#define STEPS_RATE 1500
#define LEFT_STEPS_PER_MM (12.5)
#define RIGHT_STEPS_PER_MM (12.5)

// SOFT LIMITS SETTINGS
#define ENABLE_SOFT_LIMIT 1 // uncomment to disable soft limits
#define AUTO_HOME_STEPS_RATE 2000
#define X_MM_LIMIT 1100
#define Y_MM_LIMIT 1100

#define Y_MM_MIN_LIMIT 100
#define X_MM_MIN_LIMIT 100
#define Y_MM_MAX_LIMIT 1000
#define X_MM_MAX_LIMIT 1000

#define MOTORS_DISTANCE (850) // MM distance between the motors
#define X_OFFSET (-425) // offset for the cartesian coortdinates system
#define Y_OFFSET (-410)

// BOUNDING BOX FOR THE CARTESIAN COORDINATE SYSTEM
#define MAX_X_VALUE (280)
#define MAX_Y_VALUE (300)
#define MIN_X_VALUE (-280)
#define MIN_Y_VALUE (-285)


// AUTO HOME OFFSET
#define MM_OFFSET 40

// AUTO PRINTING8
#define PENDING_TIME (1000000 * 180)
#define PEN_PENDING_TIME (1000000 * 3)

#define steps_to_mm(steps, ratio) (steps / ratio)
#define mm_to_steps(mm, ratio) (mm * ratio)

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

enum State
{
  IDLE,
  MOVE,
  PRINT
};

struct sys_state
{
  State sys_mode;
  long unsigned last_move_time_stamp;
};

#endif
