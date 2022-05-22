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

#ifndef UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_
#define UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include <string>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"

namespace ubx::nav::sig
{
class NavSIGPayload : UBXPayload
{
public:
  static const msg_class_t MSG_CLASS = UBX_NAV;
  static const msg_id_t MSG_ID = UBX_NAV_SIG;

  u4_t iTOW;      
  u1_t version;    
  u1_t numSigs;      
  // These arrays are numSigs in length
  u1_t gnssId[256];    
  u1_t svId[256];    
  u1_t sigId[256];
  u1_t freqId[256];
  i2_t prRes[256];
  u1_t cno[256];
  u1_t qualityInd[256];
  u1_t corrSource[256];
  u1_t ionoModel[256];
  x2_t sigFlags[256];



public:
  NavSIGPayload()
  : UBXPayload(MSG_CLASS, MSG_ID) {}
  NavSIGPayload(ch_t * payload_polled, u2_t size)
  : UBXPayload(MSG_CLASS, MSG_ID)
  {
    payload_.clear();
    payload_.reserve(size);
    payload_.resize(size);
    memcpy(payload_.data(), payload_polled, size);

    iTOW = buf_offset<u4_t>(&payload_, 0);
    version = buf_offset<u1_t>(&payload_,4);
    numSigs = buf_offset<u1_t>(&payload_,5);
    int n;
    for(n=0;n<numSigs;n++)
    {
      gnssId[n] = buf_offset<u1_t>(&payload_,8+(n*16));
      svId[n] = buf_offset<u1_t>(&payload_,9+(n*16));
      sigId[n] = buf_offset<u1_t>(&payload_,10+(n*16));
      freqId[n] = buf_offset<u1_t>(&payload_,11+(n*16));
      prRes[n] = buf_offset<i2_t>(&payload_,12+(n*16));
      cno[n] = buf_offset<u1_t>(&payload_,14+(n*16));
      qualityInd[n] = buf_offset<u1_t>(&payload_,15+(n*16));
      corrSource[n] = buf_offset<u1_t>(&payload_,16+(n*16));
      ionoModel[n] = buf_offset<u1_t>(&payload_,17+(n*16));
      sigFlags[n] = buf_offset<x2_t>(&payload_,18+(n*16));
    }
  }
  std::tuple<u1_t *, size_t> make_poll_payload()
  {
    payload_.clear();
    return std::make_tuple(payload_.data(), payload_.size());
  }
  std::string to_string()
  {
    std::ostringstream oss;
    oss << " iTOW: " << iTOW;
    oss << "version: " << version;
    oss << " numSigs: " << numSigs;
    return oss.str();
  }
};
}  // namespace ubx::nav::sig
#endif  // UBLOX_DGNSS_NODE__UBX__NAV__UBX_NAV_SIG_HPP_
