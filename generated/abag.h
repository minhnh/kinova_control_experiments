#ifndef ABAG_H
#define ABAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* include external headers */

/* struct declarations */
typedef struct abagState_st {
  double eBarDelay_access[1];
  int delayRearIndex_access;
  double gain_access;
  double bias_access;
  double signedErr_access;
  double eBar_access;
} abagState_t;
void initialize_abagState(abagState_t *);


/* declarations of contained functions and schedules */
void fb_sched(double const * error_sign, double * actuation, double const * gain, double const * bias);
void errSign(double const * input, double * output);
void errSignFilter(double const * filterFactor, double const * signedErr, double const * lowPassErrPrev, double * lowPassErr);
void delayEbar(double const * input, int * rearIndex, double delay[]);
void biasAdapter_sched(double const * e_bar, double const * bias_threshold, double const * bias_step, double * adapted_bias);
void gainAdapater_sched(double const * e_bar, double const * gain_threshold, double const * gain_step, double * adapted_gain);

/* declarations of root schedules */
void abag_sched(abagState_t * abagState, double const * error, double * actuation, double const * alpha, double const * biasThreshold, double const * biasStep, double const * gainThreshold, double const * gainStep);

#ifdef __cplusplus
}
#endif

#endif
