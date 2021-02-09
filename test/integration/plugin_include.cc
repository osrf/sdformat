/*
 * Copyright 2017 Open Source Robotics Foundation
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
#include <string>
#include "sdf/sdf.hh"

#include "test_config.h"

////////////////////////////////////////
// Test that plugin child elements are available even when nested in an include
TEST(PluginInclude, PluginChildElements)
{
  const auto MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "box");

  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<include>"
    << "  <uri>" + MODEL_PATH + "</uri>"
    << "  <plugin name='example' filename='libexample.so'>"
    << "    <user attribute='attribute' />"
    << "    <value attribute='boolean'>true</value>"
    << "    <uri>some_uri</uri>"
    << "  </plugin>"
    << "</include>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_TRUE(sdf::readString(stream.str(), sdfParsed));

  // Plugin attributes
  sdf::ElementPtr plugin =
      sdfParsed->Root()->GetElement("model")->GetElement("plugin");
  EXPECT_TRUE(plugin->HasAttribute("name"));
  EXPECT_EQ(plugin->GetAttribute("name")->GetAsString(), "example");
  EXPECT_TRUE(plugin->HasAttribute("filename"));
  EXPECT_EQ(plugin->GetAttribute("filename")->GetAsString(), "libexample.so");

  // 1st child element
  ASSERT_TRUE(plugin->HasElement("user"));
  {
    sdf::ElementPtr user = plugin->GetElement("user");
    EXPECT_TRUE(user->HasAttribute("attribute"));
    EXPECT_EQ(user->GetAttribute("attribute")->GetAsString(), "attribute");
  }

  // 2nd child element
  ASSERT_TRUE(plugin->HasElement("value"));
  {
    sdf::ElementPtr value = plugin->GetElement("value");
    EXPECT_TRUE(value->HasAttribute("attribute"));
    EXPECT_EQ(value->GetAttribute("attribute")->GetAsString(), "boolean");
    EXPECT_TRUE(value->Get<bool>(""));
  }
  EXPECT_TRUE(plugin->Get<bool>("value"));

  // 3rd child element
  ASSERT_TRUE(plugin->HasElement("uri"));
  EXPECT_EQ(plugin->Get<std::string>("uri"), "some_uri");
}

////////////////////////////////////////
// Test that missing required plugin attributes are detected
TEST(PluginInclude, PluginMissingFilename)
{
  const auto MODEL_PATH =
    sdf::testing::TestFile("integration", "model", "box");

  std::ostringstream stream;
  stream
    << "<sdf version='" << SDF_VERSION << "'>"
    << "<include>"
    << "  <uri>" + MODEL_PATH + "</uri>"
    << "  <plugin name='example' />"
    << "</include>"
    << "</sdf>";

  sdf::SDFPtr sdfParsed(new sdf::SDF());
  sdf::init(sdfParsed);
  ASSERT_FALSE(sdf::readString(stream.str(), sdfParsed));
}
