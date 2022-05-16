# forms an element of UBX-NAV-SIG 
uint8 QUALITY_NO_SIGNAL = 0
uint8 QUALITY_SEARCHING_SIGNAL = 1
uint8 QUALITY_ACQUIRED_SIGNAL = 2
uint8 QUALITY_DETECTED_UNUSABLE_SIGNAL = 3
uint8 QUALITY_LOCKED_SIGNAL = 4

uint8 CORRECTION_SOURCE_NO_SIGNAL = 0
uint8 CORRECTION_SOURCE_SBAS = 1
uint8 CORRECTION_SOURCE_BEIDOU = 2
uint8 CORRECTION_SOURCE_RTCM2 = 3
uint8 CORRECTION_SOURCE_RTCM3_OSR = 4
uint8 CORRECTION_SOURCE_RTCM3_SSR = 5
uint8 CORRECTION_SOURCE_QZSS_SLAS = 6
uint8 CORRECTION_SOURCE_SPARTN = 7
uint8 CORRECTION_SOURCE_CLAS = 8

uint8 IONO_MODEL_NONE = 0
uint8 IONO_MODEL_KLOBUCHAR_GPU = 1
uint8 IONO_MODEL_SBAS = 2
uint8 IONO_MODEL_KLOBUCHAR_BEIDOU = 3
uint8 IONO_MODEL_DUAL_FREQ = 4

uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_UNHEALTHY = 2


uint8 gnss_id
uint8 sv_id
uint8 sig_id
uint8 freq_id
int16 pr_res
uint8 cno
uint8 quality_ind
uint8 corr_source
uint8 iono_model

uint8 flag_signal_health 
bool flag_pr_smoothed
bool flag_pr_used
bool flag_cr_used
bool flag_do_used
bool flag_pr_corr_used
bool flag_cr_corr_used
bool flag_do_corr_used
