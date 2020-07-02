#include "abag.h"

/* functions to initialize structs of DataBlockContainer's */
void initialize_abagState(abagState_t * abagState) {
  abagState->eBarDelay[0] = 0.0L;
  abagState->delayIndex_access = 0;
  abagState->gain_access = 0.0L;
  abagState->bias_access = 0.0L;
  abagState->signedErr_access = 0.0L;
  abagState->eBar_access = 0.0L;
}

/* definitions of contained functions and schedules */
void fb_sched(double const * error_sign, double * actuation, double const * gain, double const * bias) {
  /* data block declarations */
  double gainedErrSgn_ge_sgn = 0.0L;
  double actuation_access = 0.0L;

  /* fixed data flow schedule */
  gainedErrSgn_ge_sgn = (*gain) * (*error_sign);
  actuation_access = +(*bias) + (gainedErrSgn_ge_sgn);

  /* saturation saturate_actuation */
  if      (actuation_access > 1.0) actuation_access = 1.0;
  else if (actuation_access < -1.0) actuation_access = -1.0;

  /* update output ports */
  *actuation = actuation_access;
}

void errSign(double const * input, double * output) {
  *output = (double)( (0 < *input) - (*input < 0) );
}

void errSignFilter(double const * filterFactor, double const * signedErr, double const * lowPassErrPrev, double * lowPassErr) {
  *lowPassErr = (*filterFactor) * (*lowPassErrPrev) + (1 - (*filterFactor)) * (*signedErr);
}

void delayEbar(double const * input, int * rearIndex, double delay[]) {
  delay[(*rearIndex)++] = *input;
  if (*rearIndex == 1) {
    *rearIndex = 0;
  }
}

void biasAdapter_sched(double const * e_bar, double const * bias_threshold, double const * bias_step, double * adapted_bias) {
  /* data block declarations */
  double decision_access = 0.0L;
  double bias_step_access = 0.0L;

  /* fixed data flow schedule */

  /* decision map biasAdaptDecision */
  if      (*e_bar < -*bias_threshold) decision_access = (double) -1.;
  else if (*e_bar > *bias_threshold) decision_access = (double) 1.;
  else decision_access = (double) 0.;

  bias_step_access = (*bias_step) * (decision_access);
  *adapted_bias = +(*adapted_bias) + (bias_step_access);

  /* saturation saturateBias */
  if      (*adapted_bias > 1.) *adapted_bias = 1.;
  else if (*adapted_bias < -1.) *adapted_bias = -1.;

  /* update output ports */
}

void gainAdapater_sched(double const * e_bar, double const * gain_threshold, double const * gain_step, double * adapted_gain) {
  /* data block declarations */
  double decision_access = 0.0L;
  double gain_step_access = 0.0L;

  /* fixed data flow schedule */

  /* decision map gainDecision */
  if      (*e_bar < -*gain_threshold) decision_access = (double) 1.;
  else if (*e_bar > *gain_threshold) decision_access = (double) 1.;
  else decision_access = (double) -1.;

  gain_step_access = (*gain_step) * (decision_access);
  *adapted_gain = +(*adapted_gain) + (gain_step_access);

  /* saturation saturateGain */
  if      (*adapted_gain > 1.) *adapted_gain = 1.;
  else if (*adapted_gain < -1.) *adapted_gain = -1.;

  /* update output ports */
}


/* definitions of root schedules */
void abag_sched(abagState_t * abagState, double const * error, double * actuation, double const * alpha, double const * biasThreshold, double const * biasStep, double const * gainThreshold, double const * gainStep) {
  /* data block declarations */

  /* fixed data flow schedule */
  errSign(error, &(abagState->signedErr_access));
  errSignFilter(alpha, &(abagState->signedErr_access), &(abagState->eBarDelay[abagState->delayIndex_access]), &(abagState->eBar_access));
  biasAdapter_sched(&(abagState->eBar_access), biasThreshold, biasStep, &(abagState->bias_access));
  gainAdapater_sched(&(abagState->eBar_access), gainThreshold, gainStep, &(abagState->gain_access));
  fb_sched(&(abagState->signedErr_access), actuation, &(abagState->gain_access), &(abagState->bias_access));
  delayEbar(&(abagState->eBar_access), &(abagState->delayIndex_access), abagState->eBarDelay);

  /* update output ports */
}
