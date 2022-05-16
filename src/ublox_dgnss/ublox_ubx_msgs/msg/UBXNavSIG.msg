# this message contains a UBX-NAV-SIG (0x01 0x43) record
# containing information about the satellite signals aquired
std_msgs/Header header # Header timestamp should be acquisition time

uint32 i_tow          # ms - GPS time of week 
uint8 version         
uint8 num_sigs         # number of signals 
NavSignal[] signals   