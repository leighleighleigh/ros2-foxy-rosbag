// Copyright 2021 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_RXM_RTCM_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_RXM_RTCM_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::rxm::rtcm
{
class RxmRtcmPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_RXM;
  static const msg_id_t MSG_ID = UBX_RXM_RTCM;

  u1_t version;     // RTCM version, eg 0x02
  x1_t flags;       // Input status flags group 
  u2_t subType;     // Message subtype of u-blox proprietary RTCM msgs 
  u2_t refStation;  // Reference station ID 
  u2_t msgType;     // Message type 
  // From flags
  bool flags_msgUsed;
  bool flags_crcFailed;

public:
  RxmRtcmPayload()
  : UBXPayload(MSG_CLASS, MSG_ID) {}
  RxmRtcmPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);
    version = buf_offset<u1_t>(&payload_, 0);
    flags = buf_offset<x1_t>(&payload_, 1);
    subType = buf_offset<u2_t>(&payload_, 2);
    refStation = buf_offset<u2_t>(&payload_, 4);
    msgType = buf_offset<u2_t>(&payload_, 6);
    // Evaluate flags
    flags_msgUsed = (flags >> 2) > 0;
    flags_crcFailed = (flags & 1) > 0;
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << "version: " << version;
    oss << " flag_msgUsed: " << flags_msgUsed;
    oss << " flag_crcFailed:" << flags_crcFailed;
    oss << " subType: " << subType;
    oss << " refStation: " << refStation;
    oss << " msgType: " << msgType;
    return oss.str();
  }
};
}  // namespace ubx::rxm::rtcm
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_RXM_RTCM_HPP_
