/*
 * Copyright 2019 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

#include "sdf/sdf.hh"

#include "test_config.h"

const std::string SDF_TEST_FILE =
  sdf::testing::TestFile("integration", "custom_elems_attrs.sdf");

/////////////////////////////////////////////////
TEST(SDFParser, CustomElements)
{
  sdf::Root root;
  EXPECT_TRUE(root.Load(SDF_TEST_FILE).empty());

  const sdf::World *world = root.WorldByIndex(0);

  std::string worldName = world->Name();
  EXPECT_EQ("W", worldName);

  // Use of sdf::World::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr worldElement = world->Element();

  // Use of sdf::ElementPtr::GetAttribute()
  sdf::ParamPtr typeParam = worldElement->GetAttribute("mysim:type");
  std::string simType;
  // Use of sdf::ParamPtr::Get<T>()
  typeParam->Get<std::string>(simType);
  EXPECT_EQ("2d", simType);

  const sdf::Model *model = world->ModelByIndex(0);

  const sdf::Link *link1 = model->LinkByIndex(0);
  // Use of sdf::Link::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr link1Element = link1->Element();

  // Use of sdf::ElementPtr::Get<T>() to obtain the value of an attribute
  auto customAttrStr = link1Element->Get<std::string>("mysim:custom_attr_str");
  EXPECT_EQ("A", customAttrStr);

  auto customAttrInt = link1Element->Get<int>("mysim:custom_attr_int");
  EXPECT_EQ(5, customAttrInt);

  // Use of sdf::Model::Element() to obtain an sdf::ElementPtr object
  sdf::ElementPtr modelElement = model->Element();

  sdf::ElementPtr transmission = modelElement->GetElement("mysim:transmission");
  auto transmissionName = transmission->Get<std::string>("name");
  EXPECT_EQ("simple_trans", transmissionName);

  auto transmissionType = transmission->Get<std::string>("mysim:type");
  EXPECT_EQ("transmission_interface/SimpleTransmission", transmissionType);

  sdf::ElementPtr tranJointElement = transmission->GetElement("mysim:joint");
  auto tranJointName = tranJointElement->Get<std::string>("name");
  EXPECT_EQ("J1", tranJointName);

  sdf::ElementPtr transHwInterfaceElement =
      tranJointElement->GetElement("mysim:hardwareInterface");

  // Use of sdf::ElementPtr::Get<T>() to obtain the value of a child element
  auto tranHwInterface =
      tranJointElement->Get<std::string>("mysim:hardwareInterface");
  EXPECT_EQ("EffortJointInterface", tranHwInterface);
}
