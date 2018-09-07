/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <dart/dynamics/Frame.hpp>

#include "KinematicsFeatures.hh"

namespace ignition {
namespace physics {
namespace dartsim {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the
  if (_id.IsWorld())
  {
    std::cerr << "[ignition::physics::dartsim::KinematicFeatures] Given a "
              << "FrameID belonging to the world. This should not be possible! "
              << "Please report this bug!\n";
    assert(false);
    return data;
  }

  const dart::dynamics::Frame *frame = this->frames.at(_id.ID());

  data.pose = frame->getWorldTransform();
  data.linearVelocity = frame->getLinearVelocity();
  data.angularVelocity = frame->getAngularVelocity();
  data.linearAcceleration = frame->getLinearAcceleration();
  data.angularAcceleration = frame->getAngularAcceleration();

  return data;
}

}
}
}
