/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef IGNITION_PHYSICS_DETAIL_CAPSULESHAPE_HH_
#define IGNITION_PHYSICS_DETAIL_CAPSULESHAPE_HH_

#include <string>

#include <ignition/physics/CapsuleShape.hh>

namespace ignition
{
  namespace physics
  {
    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::GetRadius() const -> Scalar
    {
      return this->template Interface<GetCapsuleShapeProperties>()
          ->GetCapsuleShapeRadius(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto GetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::GetHeight() const -> Scalar
    {
      return this->template Interface<GetCapsuleShapeProperties>()
          ->GetCapsuleShapeHeight(this->identity);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::SetRadius(Scalar _radius)
    {
      this->template Interface<SetCapsuleShapeProperties>()
          ->SetCapsuleShapeRadius(this->identity, _radius);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    void SetCapsuleShapeProperties::CapsuleShape<PolicyT, FeaturesT>
    ::SetHeight(Scalar _height)
    {
      this->template Interface<SetCapsuleShapeProperties>()
          ->SetCapsuleShapeHeight(this->identity, _height);
    }

    /////////////////////////////////////////////////
    template <typename PolicyT, typename FeaturesT>
    auto AttachCapsuleShapeFeature::Link<PolicyT, FeaturesT>
    ::AttachCapsuleShape(
        const std::string &_name,
        Scalar _radius,
        Scalar _height,
        const PoseType &_pose) -> ShapePtrType
    {
      return ShapePtrType(this->pimpl,
            this->template Interface<AttachCapsuleShapeFeature>()
                ->AttachCapsuleShape(
                            this->identity, _name, _radius, _height, _pose));
    }
  }
}

#endif
