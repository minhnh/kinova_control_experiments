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
void fb_sched(const double * error_sign, double * actuation, const double * gain, const double * bias);
void errSign(const double * input, double * output);
void errSignFilter(const double * filterFactor, const double * signedErr, const double * lowPassErrPrev, double * lowPassErr);
void delayEbar(const double * input, int * rearIndex, double delay[]);
void biasAdapter_sched(const double * e_bar, const double * bias_threshold, const double * bias_step, double * adapted_bias);
void gainAdapater_sched(const double * e_bar, const double * gain_threshold, const double * gain_step, double * adapted_gain);

/* declarations of root schedules */
void abag_sched(abagState_t * abagState, const double * error, double * actuation, const double * alpha, const double * biasThreshold, const double * biasStep, const double * gainThreshold, const double * gainStep);

#ifdef __cplusplus
}
#endif

#endif
