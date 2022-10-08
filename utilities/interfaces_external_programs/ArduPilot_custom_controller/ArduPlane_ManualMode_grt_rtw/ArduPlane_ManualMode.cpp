/*
 * ArduPlane_ManualMode.cpp
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "ArduPlane_ManualMode".
 *
 * Model version              : 1.407
 * Simulink Coder version : 9.0 (R2018b) 24-May-2018
 * C++ source code generated on : Fri Oct  7 22:45:40 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "ArduPlane_ManualMode.h"
#include "ArduPlane_ManualMode_private.h"

/* Model step function */
void ArduPlane_ManualModeModelClass::step()
{
  real_T y[8];
  int32_T i;

  /* SignalConversion: '<S3>/Signal Conversion2' incorporates:
   *  Constant: '<S5>/num signals'
   *  Inport: '<Root>/cmd'
   *  Inport: '<Root>/measure'
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[0].signals[0] = ArduPlane_ManualMode_U.cmd.roll;
  ArduPlane_ManualMode_Y.logs[0].signals[1] = ArduPlane_ManualMode_U.cmd.pitch;
  ArduPlane_ManualMode_Y.logs[0].signals[2] = ArduPlane_ManualMode_U.cmd.yaw;
  ArduPlane_ManualMode_Y.logs[0].signals[3] = ArduPlane_ManualMode_U.cmd.thr;
  ArduPlane_ManualMode_Y.logs[0].signals[4] =
    ArduPlane_ManualMode_U.measure.omega_Kb[0];
  ArduPlane_ManualMode_Y.logs[0].signals[7] =
    ArduPlane_ManualMode_U.measure.EulerAngles[0];
  ArduPlane_ManualMode_Y.logs[0].signals[10] =
    ArduPlane_ManualMode_U.measure.V_Kg[0];
  ArduPlane_ManualMode_Y.logs[0].signals[5] =
    ArduPlane_ManualMode_U.measure.omega_Kb[1];
  ArduPlane_ManualMode_Y.logs[0].signals[8] =
    ArduPlane_ManualMode_U.measure.EulerAngles[1];
  ArduPlane_ManualMode_Y.logs[0].signals[11] =
    ArduPlane_ManualMode_U.measure.V_Kg[1];
  ArduPlane_ManualMode_Y.logs[0].signals[6] =
    ArduPlane_ManualMode_U.measure.omega_Kb[2];
  ArduPlane_ManualMode_Y.logs[0].signals[9] =
    ArduPlane_ManualMode_U.measure.EulerAngles[2];
  ArduPlane_ManualMode_Y.logs[0].signals[12] =
    ArduPlane_ManualMode_U.measure.V_Kg[2];
  ArduPlane_ManualMode_Y.logs[0].signals[13] = 0.0F;
  ArduPlane_ManualMode_Y.logs[0].num_signals = 14U;
  ArduPlane_ManualMode_Y.logs[0].batch_name[0] = 77U;
  ArduPlane_ManualMode_Y.logs[0].batch_name[1] = 76U;
  ArduPlane_ManualMode_Y.logs[0].batch_name[2] = 49U;
  ArduPlane_ManualMode_Y.logs[0].batch_name[3] = 1U;

  /* SignalConversion: '<S3>/Signal Conversion1' incorporates:
   *  Constant: '<S6>/num signals'
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[1].signals[0] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[1] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[2] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[3] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[4] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[5] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[6] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[7] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[8] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[9] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[10] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[11] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[12] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].signals[13] = 0.0F;
  ArduPlane_ManualMode_Y.logs[1].num_signals = 1U;
  ArduPlane_ManualMode_Y.logs[1].batch_name[0] = 77U;
  ArduPlane_ManualMode_Y.logs[1].batch_name[1] = 76U;
  ArduPlane_ManualMode_Y.logs[1].batch_name[2] = 50U;
  ArduPlane_ManualMode_Y.logs[1].batch_name[3] = 1U;

  /* SignalConversion: '<S3>/Signal Conversion' incorporates:
   *  Constant: '<S7>/num signals'
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[2].signals[0] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[1] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[2] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[3] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[4] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[5] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[6] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[7] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[8] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[9] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[10] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[11] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[12] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].signals[13] = 0.0F;
  ArduPlane_ManualMode_Y.logs[2].num_signals = 0U;
  ArduPlane_ManualMode_Y.logs[2].batch_name[0] = 77U;
  ArduPlane_ManualMode_Y.logs[2].batch_name[1] = 76U;
  ArduPlane_ManualMode_Y.logs[2].batch_name[2] = 51U;
  ArduPlane_ManualMode_Y.logs[2].batch_name[3] = 1U;

  /* SignalConversion: '<S3>/Signal Conversion3' incorporates:
   *  Constant: '<S8>/num signals'
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[3].signals[0] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[1] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[2] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[3] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[4] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[5] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[6] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[7] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[8] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[9] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[10] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[11] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[12] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].signals[13] = 0.0F;
  ArduPlane_ManualMode_Y.logs[3].num_signals = 0U;
  ArduPlane_ManualMode_Y.logs[3].batch_name[0] = 77U;
  ArduPlane_ManualMode_Y.logs[3].batch_name[1] = 76U;
  ArduPlane_ManualMode_Y.logs[3].batch_name[2] = 52U;
  ArduPlane_ManualMode_Y.logs[3].batch_name[3] = 1U;

  /* SignalConversion: '<S3>/Signal Conversion4' incorporates:
   *  Constant: '<S9>/num signals'
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[4].signals[0] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[1] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[2] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[3] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[4] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[5] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[6] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[7] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[8] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[9] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[10] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[11] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[12] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].signals[13] = 0.0F;
  ArduPlane_ManualMode_Y.logs[4].num_signals = 0U;
  for (i = 0; i < 42; i++) {
    /* SignalConversion: '<S3>/Signal Conversion2' incorporates:
     *  Constant: '<S5>/signal names'
     *  Outport: '<Root>/logs'
     */
    ArduPlane_ManualMode_Y.logs[0].signal_names[i] =
      ArduPlane_ManualMode_ConstP.pooled1[i];

    /* SignalConversion: '<S3>/Signal Conversion1' incorporates:
     *  Constant: '<S6>/signal names'
     *  Outport: '<Root>/logs'
     */
    ArduPlane_ManualMode_Y.logs[1].signal_names[i] =
      ArduPlane_ManualMode_ConstP.pooled1[i];

    /* SignalConversion: '<S3>/Signal Conversion' incorporates:
     *  Constant: '<S7>/signal names'
     *  Outport: '<Root>/logs'
     */
    ArduPlane_ManualMode_Y.logs[2].signal_names[i] =
      ArduPlane_ManualMode_ConstP.pooled1[i];

    /* SignalConversion: '<S3>/Signal Conversion3' incorporates:
     *  Constant: '<S8>/signal names'
     *  Outport: '<Root>/logs'
     */
    ArduPlane_ManualMode_Y.logs[3].signal_names[i] =
      ArduPlane_ManualMode_ConstP.pooled1[i];

    /* SignalConversion: '<S3>/Signal Conversion4' incorporates:
     *  Constant: '<S9>/signal names'
     *  Outport: '<Root>/logs'
     */
    ArduPlane_ManualMode_Y.logs[4].signal_names[i] =
      ArduPlane_ManualMode_ConstP.pooled1[i];
  }

  /* SignalConversion: '<S3>/Signal Conversion4' incorporates:
   *  Outport: '<Root>/logs'
   */
  ArduPlane_ManualMode_Y.logs[4].batch_name[0] = 77U;
  ArduPlane_ManualMode_Y.logs[4].batch_name[1] = 76U;
  ArduPlane_ManualMode_Y.logs[4].batch_name[2] = 53U;
  ArduPlane_ManualMode_Y.logs[4].batch_name[3] = 1U;

  /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
   *  Inport: '<Root>/cmd'
   *  Inport: '<Root>/measure'
   */
  y[0] = ArduPlane_ManualMode_U.cmd.roll;
  y[1] = ArduPlane_ManualMode_U.cmd.pitch;
  y[2] = ArduPlane_ManualMode_U.cmd.thr;
  y[3] = ArduPlane_ManualMode_U.cmd.yaw;
  y[4] = ArduPlane_ManualMode_U.measure.rangefinder[0];
  y[5] = ArduPlane_ManualMode_U.measure.rangefinder[1];
  y[6] = ArduPlane_ManualMode_U.measure.rangefinder[2];
  y[7] = ArduPlane_ManualMode_U.measure.rangefinder[3];

  /* Outport: '<Root>/channels' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  for (i = 0; i < 8; i++) {
    ArduPlane_ManualMode_Y.channels[i] = (real32_T)y[i];
  }

  /* End of Outport: '<Root>/channels' */
}

/* Model initialize function */
void ArduPlane_ManualModeModelClass::initialize()
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(getRTM(), (NULL));

  /* external inputs */
  (void)memset(&ArduPlane_ManualMode_U, 0, sizeof(ExtU_ArduPlane_ManualMode_T));

  /* external outputs */
  (void) memset((void *)&ArduPlane_ManualMode_Y, 0,
                sizeof(ExtY_ArduPlane_ManualMode_T));

  {
    int32_T i;

    /* ConstCode for Outport: '<Root>/function_channels' */
    for (i = 0; i < 8; i++) {
      ArduPlane_ManualMode_Y.function_channels[i] =
        ArduPlane_ManualMode_ConstB.DataTypeConversion1[i];
    }

    /* End of ConstCode for Outport: '<Root>/function_channels' */
  }
}

/* Model terminate function */
void ArduPlane_ManualModeModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
ArduPlane_ManualModeModelClass::ArduPlane_ManualModeModelClass()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
ArduPlane_ManualModeModelClass::~ArduPlane_ManualModeModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_ArduPlane_ManualMode_T * ArduPlane_ManualModeModelClass::getRTM()
{
  return (&ArduPlane_ManualMode_M);
}
