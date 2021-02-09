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
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <ignition/utilities/ExtraTestMacros.hh>

#include "sdf/parser.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/sdf_config.h"
#include "test_config.h"

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

static const std::string g_sdfVersion(" --force-version " +
  std::string(SDF_VERSION_FULL));
static const std::string g_ignCommand(std::string(IGN_PATH) + "/ign");

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(check, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // Check a good SDF file
  {
    std::string path = pathBase +"/box_plane_low_friction_test.world";

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check a bad SDF file
  {
    std::string path = pathBase +"/box_bad_test.world";

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Required attribute"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (world)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_duplicate.sdf";

    // Check world_duplicate.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: World with name[default] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (model, light)
  // that have duplicate names.
  {
    std::string path = pathBase +"/world_sibling_same_names.sdf";

    // Check world_sibling_same_names.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Non-unique names"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (link)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_links.sdf";

    // Check model_duplicate_links.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: link with name[link] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_duplicate_joints.sdf";

    // Check model_duplicate_joints.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: joint with name[joint] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (link, joint)
  // that have duplicate names.
  {
    std::string path = pathBase +"/model_link_joint_same_name.sdf";

    // Check model_link_joint_same_name.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Non-unique names"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (collision)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_collisions.sdf";

    // Check link_duplicate_sibling_collisions.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: collision with name[collision] "
                          "already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (visual)
  // that have duplicate names.
  {
    std::string path = pathBase +"/link_duplicate_sibling_visuals.sdf";

    // Check link_duplicate_sibling_visuals.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: visual with name[visual] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with cousin elements of the same type (collision)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_collisions.sdf";

    // Check link_duplicate_cousin_collisions.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with cousin elements of the same type (visual)
  // that have duplicate names. This is a valid file.
  {
    std::string path = pathBase +"/link_duplicate_cousin_visuals.sdf";

    // Check link_duplicate_cousin_visuals.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a joint with an invalid child link.
  {
    std::string path = pathBase +"/joint_invalid_child.sdf";

    // Check joint_invalid_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Child link with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_child]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with an invalid parent link.
  {
    std::string path = pathBase +"/joint_invalid_parent.sdf";

    // Check joint_invalid_parent.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: parent link with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_parent]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with identical parent and child.
  {
    std::string path = pathBase +"/joint_invalid_parent_same_as_child.sdf";

    // Check joint_invalid_parent_same_as_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Joint with name[joint] must "
                          "specify different link names for parent and child, "
                          "while [link] was specified for both."),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a child link.
  {
    std::string path = pathBase +"/joint_child_world.sdf";

    // Check joint_child_world.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Joint with name[joint] specified invalid "
                          "child link [world]."),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a parent link.
  // This is a valid file.
  {
    std::string path = pathBase +"/joint_parent_world.sdf";

    // Check joint_parent_world.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with the second link specified as the canonical link.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_canonical_link.sdf";

    // Check model_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with an invalid link specified as the canonical link.
  {
    std::string path = pathBase +"/model_invalid_canonical_link.sdf";

    // Check model_invalid_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: canonical_link with name[link3] not found in "
                          "model with name[model_invalid_canonical_link]."),
              std::string::npos) << output;
  }

  // Check an SDF file with an invalid model without links.
  {
    std::string path = pathBase +"/model_without_links.sdf";

    // Check model_without_links.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an SDF file with a nested model.
  {
    std::string path = pathBase +"/nested_model.sdf";

    // Check nested_model.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link.
  {
    std::string path = pathBase +"/nested_canonical_link.sdf";

    // Check nested_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link
  // that is explicitly specified by //model/@canonical_link using ::
  // syntax.
  {
    std::string path = pathBase +"/nested_invalid_explicit_canonical_link.sdf";

    // Check nested_invalid_explicit_canonical_link.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: canonical_link with name[nested::link] not "
                          "found in model with name[top]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an invalid SDF file that uses reserved names.
  {
    std::string path = pathBase +"/model_invalid_reserved_names.sdf";

    // Check model_invalid_reserved_names.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: The supplied link name [world] is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: The supplied link name [__link__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: The supplied visual name [__visual__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: The supplied collision name [__collision__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: The supplied joint name [__joint__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: The supplied frame name [__frame__] "
                          "is reserved."),
              std::string::npos) << output;
  }

  // Check that validity checks are disabled inside <plugin> elements
  {
    std::string path = pathBase +"/ignore_sdf_in_plugin.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check that validity checks are disabled inside namespaced elements
  {
    std::string path = pathBase +"/ignore_sdf_in_namespaced_elements.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to.sdf";

    // Check model_frame_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to joints.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to_joint.sdf";

    // Check model_frame_attached_to_joint.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to a nested model.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_attached_to_nested_model.sdf";

    // Check model_frame_attached_to_nested_model.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid attached_to attributes.
  {
    std::string path = pathBase +"/model_frame_invalid_attached_to.sdf";

    // Check model_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: attached_to name[A] specified by frame with "
                          "name[F3] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: attached_to name[F4] is identical to frame "
                          "name[F4], causing a graph cycle in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its FrameAttachedTo graph
  {
    std::string path = pathBase +"/model_frame_invalid_attached_to_cycle.sdf";

    // Check model_frame_invalid_attached_to_cycle.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(std::string::npos, output.find(
      "FrameAttachedToGraph cycle detected, already visited vertex [F1]."))
        << output;
    EXPECT_NE(std::string::npos, output.find(
      "FrameAttachedToGraph cycle detected, already visited vertex [F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/world_frame_attached_to.sdf";

    // Check world_frame_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid attached_to attributes.
  {
    std::string path = pathBase +"/world_frame_invalid_attached_to.sdf";

    // Check world_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: attached_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: attached_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with links using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_link_relative_to.sdf";

    // Check model_link_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model links with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_link_relative_to.sdf";

    // Check model_invalid_link_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: relative_to name[A] specified by link with "
                          "name[L] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[self_cycle] is identical to "
                          "link name[self_cycle], causing a graph cycle in "
                          "model with name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with nested_models using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_nested_model_relative_to.sdf";

    // Check model_nested_model_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an invalid SDF file with a joint that specifies a child link
  // within a sibling nested model using the unsupported :: syntax.
  {
    std::string path = pathBase +"/model_invalid_nested_joint_child.sdf";

    // Check model_invalid_nested_joint_child.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: Child link with name[M::C] specified by "
                          "joint with name[J] not found in model with "
                          "name[model_invalid_nested_joint_child]."),
              std::string::npos) << output;
  }

  // Check an SDF file with joints using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_joint_relative_to.sdf";

    // Check model_joint_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model joints with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_joint_relative_to.sdf";

    // Check model_invalid_joint_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: relative_to name[A] specified by joint with "
                          "name[J] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[Jcycle] is identical to "
                          "joint name[Jcycle], causing a graph cycle in "
                          "model with name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with model frames using the relative_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_relative_to.sdf";

    // Check model_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames relative_to joints.
  // This is a valid file.
  {
    std::string path = pathBase +"/model_frame_relative_to_joint.sdf";

    // Check model_frame_relative_to_joint.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid relative_to attributes.
  {
    std::string path = pathBase +"/model_invalid_frame_relative_to.sdf";

    // Check model_invalid_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: relative_to name[A] specified by frame with "
                          "name[F] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[cycle] is identical to "
                          "frame name[cycle], causing a graph cycle in model "
                          "with name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its PoseRelativeTo graph
  {
    std::string path = pathBase +"/model_invalid_frame_relative_to_cycle.sdf";

    // Check model_invalid_frame_relative_to_cycle.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(std::string::npos, output.find(
      "PoseRelativeToGraph cycle detected, already visited vertex [F1]."))
        << output;
    EXPECT_NE(std::string::npos, output.find(
      "PoseRelativeToGraph cycle detected, already visited vertex [F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    std::string path = pathBase +"/world_frame_relative_to.sdf";

    // Check world_frame_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid relative_to attributes.
  {
    std::string path = pathBase +"/world_frame_invalid_relative_to.sdf";

    // Check world_frame_invalid_relative_to.sdf
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_NE(output.find("Error: relative_to name[A] specified by model with "
                          "name[M] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[cycle] is identical "
                          "to model name[cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Error: relative_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
  }
}

/////////////////////////////////////////////////
TEST(check_model_sdf, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/integration/model/box";

  // Check a good SDF file by passing the absolute path
  {
    std::string path = pathBase +"/model.sdf";

    std::string output =
      custom_exec_str(g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }

  // Check a good SDF file from the same folder by passing a relative path
  {
    std::string path = "model.sdf";

    std::string output =
      custom_exec_str("cd " + pathBase + " && " +
                      g_ignCommand + " sdf -k " + path + g_sdfVersion);
    EXPECT_EQ("Valid.\n", output);
  }
}

/////////////////////////////////////////////////
TEST(describe, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  // Get the description
  std::string output =
    custom_exec_str(g_ignCommand + " sdf -d " + g_sdfVersion);
  EXPECT_FALSE(output.empty());

  // The first line should start with the following text.
  EXPECT_EQ(0u, output.find("<element name ='sdf' required ='1'"));
}

/////////////////////////////////////////////////
TEST(print, IGN_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase += "/test/sdf";

  // Check a good SDF file
  {
    std::string path = pathBase +"/box_plane_low_friction_test.world";
    sdf::SDFPtr sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));
    EXPECT_TRUE(sdf::readFile(path, sdf));

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -p " + path + g_sdfVersion);
    EXPECT_EQ(sdf->Root()->ToString(""), output);
  }

  // Check a bad SDF file
  {
    std::string path = pathBase +"/box_bad_test.world";

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(g_ignCommand + " sdf -p " + path + g_sdfVersion);
    EXPECT_TRUE(output.find("Required attribute") != std::string::npos);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  // Set IGN_CONFIG_PATH to the directory where the .yaml configuration file
  // is located.
  sdf::testing::setenv("IGN_CONFIG_PATH", IGN_CONFIG_PATH);

  // Make sure that we load the library recently built and not the one installed
  // in your system. This is done by placing the the current build directory
  // first in the LD_LIBRARY_PATH environment variable.
  //
  // We need to keep the existing LD_LIBRARY_PATH so that libsdformat.so can
  // find its dependency.
#ifndef _WIN32
  std::string testLibraryPath = IGN_TEST_LIBRARY_PATH;

  std::string currentLibraryPath;
  if (sdf::testing::env("LD_LIBRARY_PATH", currentLibraryPath))
  {
    testLibraryPath = testLibraryPath + ":" + currentLibraryPath;
  }

  sdf::testing::setenv("LD_LIBRARY_PATH", testLibraryPath);
#endif

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
