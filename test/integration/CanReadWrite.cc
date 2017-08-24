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

#define IGNITION_UNITTEST_EXPECTDATA_ACCESS

#include "ignition/physics/SpecifyData.hh"
#include "ignition/physics/CanReadData.hh"
#include "ignition/physics/CanWriteData.hh"

#include "utils/TestDataTypes.hh"


class SomeClass
    : public ignition::physics::CanReadRequiredData<SomeClass, RequireStringBoolChar>,
      public ignition::physics::CanReadExpectedData<SomeClass, RequireStringBoolChar>,
      public ignition::physics::CanWriteExpectedData<SomeClass, RequireIntDouble>
{
  public: StringData sdata;
  public: BoolData bdata;
  public: CharData cdata;
  public: IntData idata;
  public: FloatData fdata;

  public: void Read(const StringData& _sdata)
  {
    sdata = _sdata;
  }

  public: void Read(const BoolData& _bdata)
  {
    bdata = _bdata;
  }

  public: void Read(const CharData& _cdata)
  {
    cdata = _cdata;
  }

  public: void Read(const IntData& _idata)
  {
    idata = _idata;
  }

  public: void Read(const FloatData& _fdata)
  {
    fdata = _fdata;
  }

  public: void Write(IntData& _idata) const
  {
    _idata.myInt = 67;
  }

  public: void Write(DoubleData& _ddata) const
  {
    _ddata.myDouble = 7.2;
  }

  public: void Write(StringData& _sdata) const
  {
    _sdata.myString = "seventy-seven";
  }

  public: void Write(CharData& _cdata) const
  {
    _cdata.myChar = '8';
  }
};

TEST(SpecifyData, ReadData)
{
  ignition::physics::CompositeData input;
  input.Get<StringData>().myString = "89";
  input.Get<BoolData>().myBool = false;
  input.Get<CharData>().myChar = 'd';
  input.Get<IntData>().myInt = 92;
  input.Get<FloatData>().myFloat = 93.5;
  input.ResetQueries();

  SomeClass something;
  something.ReadRequiredData(input);
  EXPECT_EQ("89", something.sdata.myString);
  EXPECT_FALSE(something.bdata.myBool);
  EXPECT_EQ('d', something.cdata.myChar);
  EXPECT_EQ(55, something.idata.myInt);
  EXPECT_NEAR(9.5, something.fdata.myFloat, 1e-8);

  something.ReadExpectedData(input);
  EXPECT_EQ(92, something.idata.myInt);
  EXPECT_NEAR(93.5, something.fdata.myFloat, 1e-8);

  ignition::physics::CompositeData output;
  something.WriteExpectedData(output);
  EXPECT_EQ(67, output.Get<IntData>().myInt);
  EXPECT_NEAR(7.2, output.Get<DoubleData>().myDouble, 1e-8);
  EXPECT_EQ("seventy-seven", output.Get<StringData>().myString);
  EXPECT_EQ('8', output.Get<CharData>().myChar);
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
