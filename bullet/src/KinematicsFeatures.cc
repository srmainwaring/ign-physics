#include <ignition/common/Console.hh>
#include "KinematicsFeatures.hh"

namespace ignition {
namespace physics {
namespace bullet {

/////////////////////////////////////////////////
FrameData3d KinematicsFeatures::FrameDataRelativeToWorld(
    const FrameID &_id) const
{
  FrameData3d data;

  // The feature system should never send us the world ID.
  if (_id.IsWorld())
  {
    ignerr << "Given a FrameID belonging to the world. This should not be "
           << "possible! Please report this bug!\n";
    assert(false);
    return data;
  }

  const auto linkID = _id.ID();

  if (this->links.find(linkID) == this->links.end())
  {
    ignerr << "Given a FrameID not belonging to a link.\n";
    return data;
  }
  const auto &linkInfo = this->links.at(linkID);
  const auto &model = linkInfo->link;

  btTransform trans;
  model->getMotionState()->getWorldTransform(trans);
  btVector3 pos = trans.getOrigin();
  btMatrix3x3 mat = trans.getBasis();

  auto eigenMat = convert(mat);
  auto eigenVec = convert(pos);

  data.pose.linear() = eigenMat;
  data.pose.translation() = eigenVec;

  // Add base velocities
  btVector3 omega = model->getAngularVelocity();
  btVector3 vel = model->getLinearVelocity();
  if (linkInfo->name == "rod_1")
  {
    // ignerr << "angular " << omega[0] << " " << omega[1] << " " << omega[2] << std::endl;
    // ignerr << "linear " << vel[0] << " " << vel[1] << " " << vel[2] << std::endl;
  }

  // Transform to world frame
  // const auto matBaseToWorld = mat.inverse();
  // omega = matBaseToWorld * omega;
  // vel = matBaseToWorld * vel;

  // mat = trans.inverse().getBasis();
  // pos = trans.inverse().getOrigin();
  
  // btMatrix3x3 skewPosition(0, -pos[3], pos[2], pos[3], 0, -pos[1], -pos[2], pos[1], 0);
  // vel = mat * vel + skewPosition * mat  * omega;
  // omega = mat * vel;

  data.linearVelocity = convert(vel);
  data.angularVelocity = convert(omega);

  // \todo(anyone) compute frame accelerations

  return data;
}

}
}
}
