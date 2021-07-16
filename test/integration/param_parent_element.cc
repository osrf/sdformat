/*
 * Copyright 2021 Open Source Robotics Foundation
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
#include <ignition/math/Pose3.hh>

#include "sdf/Element.hh"
#include "sdf/Param.hh"
#include "test_config.h"

//////////////////////////////////////////////////
TEST(ParamParentElement, SettingParentElement)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
  parentElement->SetName("ParentElement");

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  doubleParam.SetParentElement(parentElement);

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  ASSERT_EQ("ParentElement", doubleParam.GetParentElement()->GetName());

  // Set a new parent Element
  sdf::ElementPtr newParentElement = std::make_shared<sdf::Element>();
  newParentElement->SetName("NewParentElement");

  doubleParam.SetParentElement(newParentElement);
  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  ASSERT_EQ("NewParentElement", doubleParam.GetParentElement()->GetName());

  // Remove the parent Element
  doubleParam.SetParentElement(nullptr);
  EXPECT_EQ(nullptr, doubleParam.GetParentElement());
}

//////////////////////////////////////////////////
TEST(ParamParentElement, CopyConstructor)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
  parentElement->SetName("ParentElement");

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  doubleParam.SetParentElement(parentElement);

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  ASSERT_EQ("ParentElement", doubleParam.GetParentElement()->GetName());

  sdf::Param newParam(doubleParam);
  ASSERT_NE(nullptr, newParam.GetParentElement());
  EXPECT_EQ("ParentElement", newParam.GetParentElement()->GetName());
}

//////////////////////////////////////////////////
TEST(ParamParentElement, EqualOperator)
{
  sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
  parentElement->SetName("ParentElement");

  sdf::Param doubleParam("key", "double", "1.0", false, "description");
  doubleParam.SetParentElement(parentElement);

  ASSERT_NE(nullptr, doubleParam.GetParentElement());
  ASSERT_EQ("ParentElement", doubleParam.GetParentElement()->GetName());

  sdf::Param newParam = doubleParam;
  ASSERT_NE(nullptr, newParam.GetParentElement());
  EXPECT_EQ("ParentElement", newParam.GetParentElement()->GetName());
}

//////////////////////////////////////////////////
TEST(ParamParentElement, DestroyParentElementAfterConstruct)
{
  sdf::ParamPtr param = nullptr;

  {
    sdf::ElementPtr parentElement = std::make_shared<sdf::Element>();
    parentElement->SetName("ParentElement");

    param = std::make_shared<sdf::Param>(
        "key", "double", "1.0", false, "description");
    param->SetParentElement(parentElement);
  }

  ASSERT_NE(nullptr, param);
  EXPECT_EQ(nullptr, param->GetParentElement());
}
