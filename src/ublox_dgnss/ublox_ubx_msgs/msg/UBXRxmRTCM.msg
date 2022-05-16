# this message contains a UBX-RXM-RTCM (0x02 0x32) message 
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version          # RTCM version, eg 0x02
bool flags_msg_used
bool flags_crc_failed
uint16 sub_type         # Message subtype of u-blox proprietary RTCM msgs 
uint16 ref_station_id      # Reference station ID 
uint16 msg_type         # Message type 