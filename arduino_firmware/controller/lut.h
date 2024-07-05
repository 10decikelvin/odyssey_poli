/**
 * @file lut.h
 * @brief Defines the lookup table for feedforward of PID.
 * @ingroup PID
 * This header file contains the definition of a lookup table (LUT) used for the feedforward component of a PID
 * controller. The LUT provides predefined output values for given input conditions, enhancing the PID controller's
 * response by compensating for known system behaviors in advance.
 */


/**
 * @struct motor
 * @brief Structure to represent motor parameters for PID feedforward.
 * @ingroup PID
 * This structure is used within the lookup table to map specific motor speed (in ticks per second) to a corresponding
 * PWM signal value and operational mode. It is a key component in implementing feedforward control in PID controllers
 * for motors.
 *
 * @param speed Motor speed in ticks per second.
 * @param pwm PWM signal value (0-255) for speed control.
 * @param mode Operational mode of the motor (0 = normal, 1 = coast).
 */
struct motor {
    int speed; // ticks per second
    int pwm;   // 0-255
    int mode;  // 0 = normal, 1 = coast
};

/**
 * @var motor lut[]
 * @brief Lookup table array for PID feedforward control.
 * @ingroup PID
 * This array of `motor` structures defines the lookup table for the feedforward component of the PID controller.
 * Each entry maps a specific motor speed to a PWM value and operational mode, allowing for immediate adjustment
 * based on the current speed requirement.
 */
motor lut[] = {
    {0,0,0},
    {60,20,0},
    {85,25,0},
    {110,30,0},
    {135,35,0},
    {160,40,0},
    {185,45,0},
    {215,50,0},
    {240,55,0},
    {265,60,0},
    {290,65,0},
    {310,70,0},
    {335,75,0},
    {360,80,0},
    {385,85,0},
    {410,90,0},
    {435,95,0},
    {455,100,0},
    {480,105,0},
    {505,110,0},
    {530,115,0},
    {540,120,0},
    {565,125,0},
    {610,130,0},
    {635,135,0},
    {650,140,0},
    {675,145,0},
    {700,150,0},
    {725,155,0},
    {770,160,0},
    {795,165,0},
    {820,170,0},
    {845,175,0},
    {850,180,0},
    {875,185,0},
    {900,190,0},
    {915,195,0},
    {920,200,0},
    {935,205,0},
    {1000,210,0},
    {1025,215,0},
    {1050,220,0},
    {1075,225,0},
    {1100,230,0},
    {1125,235,0},
    {1150,240,0},
    {1175,245,0},
    {1200,250,0},
    {1225,253,0},
    {1250,255,0}
};


