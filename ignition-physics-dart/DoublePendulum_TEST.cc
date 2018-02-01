/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <ignition/math/PID.hh>

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/SpecializedPluginPtr.hh>

#include <ignition/physics/ForwardStep.hh>

#include <utils/test_config.h>


using PhysicsPlugin = ignition::common::SpecializedPluginPtr<
    ignition::physics::ForwardStep,
    ignition::physics::SetState>;

/////////////////////////////////////////////////
TEST(DoublePendulum, Step)
{
  std::string projectPath = PROJECT_BINARY_PATH;

  ignition::common::SystemPaths sp;
  sp.AddPluginPaths(projectPath + "/ignition-physics-dart");
  std::string path = sp.FindSharedLibrary("ignition-physics-dart");

  ignition::common::PluginLoader loader;
  loader.LoadLibrary(path);

  auto pluginNames = loader.PluginsImplementing(
                        "::ignition::physics::DoublePendulum");
  ASSERT_FALSE(pluginNames.empty());
  for (const std::string & name : pluginNames)
  {
    std::cerr << "DoublePendulum plugin: " << name << std::endl;
  }
  const std::string pluginName = *pluginNames.begin();
  std::cerr << "         using plugin: " << pluginName << std::endl;
  PhysicsPlugin plugin = loader.Instantiate(pluginName);

  ASSERT_TRUE(plugin);

  ignition::physics::ForwardStep *step =
      plugin->QueryInterface<ignition::physics::ForwardStep>();

  ignition::physics::ForwardStep::State state;
  ignition::physics::ForwardStep::Output output;
  ignition::physics::ForwardStep::Input input;

  const std::chrono::duration<double> dt(std::chrono::milliseconds(1));
  ignition::physics::TimeStep &timeStep =
      input.Get<ignition::physics::TimeStep>();
  timeStep.dt = dt.count();

  ignition::physics::GeneralizedParameters &efforts =
      input.Get<ignition::physics::GeneralizedParameters>();
  efforts.dofs.push_back(0);
  efforts.dofs.push_back(1);
  efforts.forces.push_back(0.0);
  efforts.forces.push_back(0.0);

  // No input on the first step, let's just see the output
  step->Step(output, state, input);

  ASSERT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions0 = output.Get<ignition::physics::JointPositions>();

  // the double pendulum is initially fully inverted
  // and angles are defined as zero in this state
  double angle00 = positions0.positions[positions0.dofs[0]];
  double angle01 = positions0.positions[positions0.dofs[1]];
  EXPECT_NEAR(0.0, angle00, 1e-6);
  EXPECT_NEAR(0.0, angle01, 1e-6);

  // set target with joint1 still inverted, but joint2 pointed down
  // this is also an equilibrium position
  const double target10 = 0.0;
  const double target11 = IGN_PI;

  // PID gains tuned in gazebo
  ignition::math::PID pid0(100, 0, 10);
  ignition::math::PID pid1(10, 0, 5);
  const std::chrono::duration<double> settleTime(std::chrono::seconds(4));
  unsigned int settleSteps = settleTime / dt;
  for (unsigned int i = 0; i < settleSteps; ++i)
  {
    auto positions = output.Get<ignition::physics::JointPositions>();
    double error0 = positions.positions[positions.dofs[0]] - target10;
    double error1 = positions.positions[positions.dofs[1]] - target11;

    efforts.forces[0] = pid0.Update(error0, dt);
    efforts.forces[1] = pid1.Update(error1, dt);

    step->Step(output, state, input);
  }

  // expect joints are near target positions
  ASSERT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions1 = output.Get<ignition::physics::JointPositions>();
  double angle10 = positions1.positions[positions1.dofs[0]];
  double angle11 = positions1.positions[positions1.dofs[1]];
  EXPECT_NEAR(target10, angle10, 1e-5);
  EXPECT_NEAR(target11, angle11, 1e-3);

  // test recording the state and repeatability
  ignition::physics::ForwardStep::State bookmark = state;
  std::vector<double> errorHistory0;
  std::vector<double> errorHistory1;

  // The states are reset, but the outputs haven't been recomputed.
  // so reset the PID's, zero the inputs and take one more Step
  // so the outputs will be computed.
  const double target20 = 0.0;
  const double target21 = -IGN_PI;
  pid0.Reset();
  pid1.Reset();
  efforts.forces[0] = 0;
  efforts.forces[1] = 0;
  step->Step(output, state, input);

  for (unsigned int i = 0; i < settleSteps; ++i)
  {
    auto positions = output.Get<ignition::physics::JointPositions>();
    double error0 = positions.positions[positions.dofs[0]] - target20;
    double error1 = positions.positions[positions.dofs[1]] - target21;
    errorHistory0.push_back(error0);
    errorHistory1.push_back(error1);

    efforts.forces[0] = pid0.Update(error0, dt);
    efforts.forces[1] = pid1.Update(error1, dt);

    step->Step(output, state, input);
  }

  // expect joints are near target positions again
  ASSERT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions2 = output.Get<ignition::physics::JointPositions>();
  double angle20 = positions2.positions[positions2.dofs[0]];
  double angle21 = positions2.positions[positions2.dofs[1]];
  EXPECT_NEAR(target20, angle20, 1e-4);
  EXPECT_NEAR(target21, angle21, 1e-3);

  // Go back to the bookmarked state and run through the steps again.
  ignition::physics::SetState *setState =
      plugin->QueryInterface<ignition::physics::SetState>();
  ASSERT_TRUE(setState);
  setState->SetStateTo(bookmark);

  // The states are reset, but the outputs haven't been recomputed.
  // so reset the PID's, zero the inputs and take one more Step
  // so the outputs will be computed.
  pid0.Reset();
  pid1.Reset();
  efforts.forces[0] = 0;
  efforts.forces[1] = 0;
  step->Step(output, state, input);

  for (unsigned int i = 0; i < settleSteps; ++i)
  {
    auto positions = output.Get<ignition::physics::JointPositions>();
    double error0 = positions.positions[positions.dofs[0]] - target20;
    double error1 = positions.positions[positions.dofs[1]] - target21;
    EXPECT_DOUBLE_EQ(error0, errorHistory0[i]);
    EXPECT_DOUBLE_EQ(error1, errorHistory1[i]);

    efforts.forces[0] = pid0.Update(error0, dt);
    efforts.forces[1] = pid1.Update(error1, dt);

    step->Step(output, state, input);
  }

  // expect joints are near target positions again
  ASSERT_TRUE(output.Has<ignition::physics::JointPositions>());
  auto positions3 = output.Get<ignition::physics::JointPositions>();
  double angle30 = positions3.positions[positions3.dofs[0]];
  double angle31 = positions3.positions[positions3.dofs[1]];
  EXPECT_DOUBLE_EQ(angle20, angle30);
  EXPECT_DOUBLE_EQ(angle21, angle31);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
