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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_

#include <unistd.h>
#include <memory>
#include <tuple>
#include "ublox_dgnss_node/ubx/ubx.hpp"
#include "ublox_dgnss_node/ubx/utils.hpp"
// More RXM types would be made in headers in the ./rxm folder
#include "ublox_dgnss_node/ubx/rxm/ubx_rxm_rtcm.hpp"

namespace ubx::rxm
{

typedef UBXFrameComms<rxm::rtcm::RxmRtcmPayload, usb::Connection> UBXRxmRTCMFrameComms;
// More RXM types would go here...

class UbxRxm
{
private:
  std::shared_ptr<usb::Connection> usbc_;

  std::shared_ptr<UBXRxmRTCMFrameComms> rtcm_;
  // More messages would go here...

public:
  explicit UbxRxm(std::shared_ptr<usb::Connection> usbc)
  {
    usbc_ = usbc;
    rtcm_ = std::make_shared<UBXRxmRTCMFrameComms>(usbc_);
  }

  // More shared_ptr's would be added here...
  std::shared_ptr<UBXRxmRTCMFrameComms> rtcm()
  {
    return rtcm_;
  }

  void frame(std::shared_ptr<ubx::Frame> frame)
  {
    switch (frame->msg_id) {
      case ubx::UBX_RXM_RTCM:
        rtcm_->frame(frame);
        break;

      // More cases would be added here...
      default:
        // break;
        throw UbxValueException("unknown UBX_RXM msg_id: " + ubx::to_hex(frame->msg_id));
    }
  }
};
}  // namespace ubx::rxm

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_RXM_HPP_
