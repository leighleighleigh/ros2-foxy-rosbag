# this message contains a UBX-NAV-SVIN (0x01 0x3b) record
# Navigation survey-in parameters relevant to the GPS Base station 
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version         # Message version (0x0 for the F9P) 
uint32 i_tow          # ms - GPS time of week 
uint32 dur            # s - elapsed time of the survey
int32  mean_x         # cm - Current survey-in mean position X axis, ECEF coordinate frame 
int32  mean_y         # cm - Current survey-in mean position Y axis, ECEF coordinate frame 
int32  mean_z         # cm - Current survey-in mean position X axis, ECEF coordinate frame 
int8   mean_x_hp      # 0.1mm - high-precision component of mean_x. combined to form a high 
                      #         precision position, x = mean_x + (0.01 * mean_x_hp)
int8   mean_y_hp
int8   mean_z_hp 
uint32 mean_acc       # 0.1mm - Current survey-in mean position accuracy !!
uint32 obs            # Number of observations used in the survey 
bool   valid          # Position validity flag, true = survey was valid
bool   active         # Progress flag, true = in-progress