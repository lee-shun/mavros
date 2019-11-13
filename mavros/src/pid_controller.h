//
// Header Guard
//
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#define CONSTRAIN(x, lower, upper) ((x) < (lower) ? (lower) : ((x) > (upper) ? (upper) : (x)))

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    MANUAL,
    AUTOMATIC
} PIDMode;

typedef enum
{
    DIRECT,
    REVERSE
} PIDDirection;

//*********************************************************************************
// Class
//*********************************************************************************

class
    PIDControl
{
public:
    //
    // Constructor
    // Description:
    //      Initializes the PIDControl instantiation. This should be called at
    //      least once before any other PID functions are called on the
    //      instantiation.
    // Parameters:
    //      kp - Positive P gain constant value.
    //      ki - Positive I gain constant value.
    //      kd - Positive D gain constant value.
    //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be
    //          called.
    //      minOutput - Constrain PID output to this minimum value.
    //      maxOutput - Constrain PID output to this maximum value.
    //      mode - Tells how the controller should respond if the user has
    //          taken over manual control or not.
    //          MANUAL:    PID controller is off. User can manually control the
    //                     output.
    //          AUTOMATIC: PID controller is on. PID controller controls the
    //                     output.
    //      controllerDirection - The sense of direction of the controller
    //          DIRECT:  A positive setpoint gives a positive output.
    //          REVERSE: A positive setpoint gives a negative output.
    // Returns:
    //      Nothing.
    //
    PIDControl(float kp, float ki, float kd, float sampleTimeSeconds,
               float minOutput, float maxOutput, PIDMode mode,
               PIDDirection controllerDirection);

    //
    // PID Compute
    // Description:
    //      Should be called on a regular interval defined by sampleTimeSeconds.
    //      Typically, PIDSetpointSet and PIDInputSet should be called
    //      immediately before PIDCompute.
    // Parameters:
    //      None.
    // Returns:
    //      True if in AUTOMATIC. False if in MANUAL.
    //
    bool PIDCompute();

    //
    // PID Mode Set
    // Description:
    //      Sets the PID controller to a new mode. Tells how the controller
    //      should respond if the user has taken over manual control or not.
    // Parameters:
    //      mode -
    //          MANUAL:    PID controller is off. User can manually control the
    //                     output.
    //          AUTOMATIC: PID controller is on. PID controller controls the
    //                     output.
    // Returns:
    //      Nothing.
    //
    void PIDModeSet(PIDMode mode);

    //
    // PID Output Limits Set
    // Description:
    //      Sets the new output limits. The new limits are applied to the PID
    //      immediately.
    // Parameters:
    //      min - Constrain PID output to this minimum value.
    //      max - Constrain PID output to this maximum value.
    // Returns:
    //      Nothing.
    //
    void PIDOutputLimitsSet(float min, float max);

    //
    // PID Tunings Set
    // Description:
    //      Sets the new gain constant values.
    // Parameters:
    //      kp - Positive P gain constant value.
    //      ki - Positive I gain constant value.
    //      kd - Positive D gain constant value.
    // Returns:
    //      Nothing.
    //
    void PIDTuningsSet(float kp, float ki, float kd);

    //
    // PID Tuning Gain Constant P Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      kp - Positive P gain constant value.
    // Returns:
    //      Nothing.
    //
    void PIDTuningKpSet(float kp);

    //
    // PID Tuning Gain Constant I Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      ki - Positive I gain constant value.
    // Returns:
    //      Nothing.
    //
    void PIDTuningKiSet(float ki);

    //
    // PID Tuning Gain Constant D Set
    // Description:
    //      Sets the proportional gain constant value.
    // Parameters:
    //      kd - Positive D gain constant value.
    // Returns:
    //      Nothing.
    //
    void PIDTuningKdSet(float kd);

    //
    // PID Controller Direction Set
    // Description:
    //      Sets the new controller direction.
    // Parameters:
    //      controllerDirection - The sense of direction of the controller
    //          DIRECT:  A positive setpoint gives a positive output
    //          REVERSE: A positive setpoint gives a negative output
    // Returns:
    //      Nothing.
    //
    void PIDControllerDirectionSet(PIDDirection controllerDirection);

    //
    // PID Sample Time Set
    // Description:
    //      Sets the new sampling time (in seconds).
    // Parameters:
    //      sampleTimeSeconds - Interval in seconds on which PIDCompute will be
    //          called.
    // Returns:
    //      Nothing.
    //
    void PIDSampleTimeSet(float sampleTimeSeconds);

    //
    // PID Setpoint Set
    // Description:
    //      Alters the setpoint the PID controller will try to achieve.
    // Parameters:
    //      setpoint - The desired setpoint the PID controller will try to
    //          obtain.
    // Returns:
    //      Nothing.
    //
    inline void PIDSetpointSet(float pidsetpoint) { setpoint = pidsetpoint; }

    //
    // PID Input Set
    // Description:
    //      Should be called before calling PIDCompute so the PID controller
    //      will have an updated input value to work with.
    // Parameters:
    //      input - The value the controller will work with.
    // Returns:
    //      Nothing.
    //
    inline void PIDInputSet(float pidinput) { input = pidinput; }

    //
    // PID Output Get
    // Description:
    //      Typically, this function is called after PIDCompute in order to
    //      retrieve the output of the controller.
    // Parameters:
    //      None.
    // Returns:
    //      The output of the specific PID controller.
    //
    inline float PIDOutputGet() { return output; }

    //
    // PID Proportional Gain Constant Get
    // Description:
    //      Returns the proportional gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The proportional gain constant.
    //
    inline float PIDKpGet() { return dispKp; }

    //
    // PID Integral Gain Constant Get
    // Description:
    //      Returns the integral gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The integral gain constant.
    //
    inline float PIDKiGet() { return dispKi; }

    //
    // PID Derivative Gain Constant Get
    // Description:
    //      Returns the derivative gain constant value the particular
    //      controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      The derivative gain constant.
    //
    inline float PIDKdGet() { return dispKd; }

    //
    // PID Mode Get
    // Description:
    //      Returns the mode the particular controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      MANUAL or AUTOMATIC depending on what the user set the
    //      controller to.
    //
    inline PIDMode PIDModeGet() { return mode; }

    //
    // PID Direction Get
    // Description:
    //      Returns the direction the particular controller is set to.
    // Parameters:
    //      None.
    // Returns:
    //      DIRECT or REVERSE depending on what the user set the
    //      controller to.
    //
    inline PIDDirection PIDDirectionGet() { return controllerDirection; }

private:
    //
    // Input to the PID Controller
    //
    float input;

    //
    // Previous input to the PID Controller
    //
    float lastInput;

    //
    // Output of the PID Controller
    //
    float output;

    //
    // Gain constant values that were passed by the user
    // These are for display purposes
    //
    float dispKp;
    float dispKi;
    float dispKd;

    //
    // Gain constant values that the controller alters for
    // its own use
    //
    float alteredKp;
    float alteredKi;
    float alteredKd;

    //
    // The Integral Term
    //
    float iTerm;

    //
    // The interval (in seconds) on which the PID controller
    // will be called
    //
    float sampleTime;

    //
    // The values that the output will be constrained to
    //
    float outMin;
    float outMax;

    //
    // The user chosen operating point
    //
    float setpoint;

    //
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    //
    PIDDirection controllerDirection;

    //
    // Tells how the controller should respond if the user has
    // taken over manual control or not
    // MANUAL:    PID controller is off.
    // AUTOMATIC: PID controller is on.
    //
    PIDMode mode;
};

//*********************************************************************************
// Public Class Functions
//*********************************************************************************

PIDControl::
    PIDControl(float kp, float ki, float kd, float sampleTimeSeconds, float minOutput,
               float maxOutput, PIDMode mode, PIDDirection controllerDirection)
{
    controllerDirection = controllerDirection;
    mode = mode;
    iTerm = 0.0f;
    input = 0.0f;
    lastInput = 0.0f;
    output = 0.0f;
    setpoint = 0.0f;

    if (sampleTimeSeconds > 0.0f)
    {
        sampleTime = sampleTimeSeconds;
    }
    else
    {
        // If the passed parameter was incorrect, set to 1 second
        sampleTime = 1.0f;
    }

    PIDOutputLimitsSet(minOutput, maxOutput);
    PIDTuningsSet(kp, ki, kd);
}

bool PIDControl::
    PIDCompute()
{
    float error, dInput;

    if (mode == MANUAL)
    {
        return false;
    }

    // The classic PID error term
    error = setpoint - input;

    // Compute the integral term separately ahead of time
    iTerm += alteredKi * error;

    // Constrain the integrator to make sure it does not exceed output bounds
    iTerm = CONSTRAIN(iTerm, outMin, outMax);

    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = input - lastInput;

    // Run all the terms together to get the overall output
    output = alteredKp * error + iTerm - alteredKd * dInput;

    // Bound the output
    output = CONSTRAIN(output, outMin, outMax);

    // Make the current input the former input
    lastInput = input;

    return true;
}

void PIDControl::
    PIDModeSet(PIDMode mode)
{
    // If the mode changed from MANUAL to AUTOMATIC
    if (mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        iTerm = output;
        lastInput = input;

        // Constrain the integrator to make sure it does not exceed output bounds
        iTerm = CONSTRAIN(iTerm, outMin, outMax);
    }

    mode = mode;
}

void PIDControl::
    PIDOutputLimitsSet(float min, float max)
{
    // Check if the parameters are valid
    if (min >= max)
    {
        return;
    }

    // Save the parameters
    outMin = min;
    outMax = max;

    // If in automatic, apply the new constraints
    if (mode == AUTOMATIC)
    {
        output = CONSTRAIN(output, min, max);
        iTerm = CONSTRAIN(iTerm, min, max);
    }
}

void PIDControl::
    PIDTuningsSet(float kp, float ki, float kd)
{
    // Check if the parameters are valid
    if (kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }

    // Save the parameters for displaying purposes
    dispKp = kp;
    dispKi = ki;
    dispKd = kd;

    // Alter the parameters for PID
    alteredKp = kp;
    alteredKi = ki * sampleTime;
    alteredKd = kd / sampleTime;

    // Apply reverse direction to the altered values if necessary
    if (controllerDirection == REVERSE)
    {
        alteredKp = -(alteredKp);
        alteredKi = -(alteredKi);
        alteredKd = -(alteredKd);
    }
}

void PIDControl::
    PIDTuningKpSet(float kp)
{
    PIDTuningsSet(kp, dispKi, dispKd);
}

void PIDControl::
    PIDTuningKiSet(float ki)
{
    PIDTuningsSet(dispKp, ki, dispKd);
}

void PIDControl::
    PIDTuningKdSet(float kd)
{
    PIDTuningsSet(dispKp, dispKi, kd);
}

void PIDControl::
    PIDControllerDirectionSet(PIDDirection controllerDirection)
{
    // If in automatic mode and the controller's sense of direction is reversed
    if (mode == AUTOMATIC && controllerDirection == REVERSE)
    {
        // Reverse sense of direction of PID gain constants
        alteredKp = -(alteredKp);
        alteredKi = -(alteredKi);
        alteredKd = -(alteredKd);
    }

    controllerDirection = controllerDirection;
}

void PIDControl::
    PIDSampleTimeSet(float sampleTimeSeconds)
{
    float ratio;

    if (sampleTimeSeconds > 0.0f)
    {
        // Find the ratio of change and apply to the altered values
        ratio = sampleTimeSeconds / sampleTime;
        alteredKi *= ratio;
        alteredKd /= ratio;

        // Save the new sampling time
        sampleTime = sampleTimeSeconds;
    }
}

#endif // PID_CONTROLLER_H
