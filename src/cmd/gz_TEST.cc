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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <gz/utils/ExtraTestMacros.hh>
#include <gz/utils/Environment.hh>

#include "sdf/Error.hh"
#include "sdf/Filesystem.hh"
#include "sdf/parser.hh"
#include "sdf/SDFImpl.hh"
#include "sdf/config.hh"
#include "test_config.hh"
#include "test_utils.hh"

#ifdef _WIN32
  #define popen  _popen
  #define pclose _pclose
#endif

// DETAIL_GZ_CONFIG_PATH is compiler definition set in CMake.
#define GZ_CONFIG_PATH DETAIL_GZ_CONFIG_PATH

static std::string SdfVersion()
{
  return " --force-version " + std::string(SDF_VERSION_FULL);
}

static std::string GzCommand()
{
  return std::string(GZ_PATH);
}

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
TEST(checkUnrecognizedElements, SDF)
{
  // Check an SDFormat file with unrecognized elements
  {
    const auto path =
      sdf::testing::TestFile("sdf", "unrecognized_elements.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(std::string::npos, output.find(
        "XML Attribute[some_attribute] in element[model] not defined in SDF."))
      << output;
    EXPECT_NE(std::string::npos, output.find(
        "XML Element[not_a_link_element], child of element[link], not "
        "defined in SDF."))
      << output;
    EXPECT_NE(std::string::npos, output.find(
        "XML Element[not_a_model_element], child of element[model], not "
        "defined in SDF."))
      << output;
    EXPECT_NE(std::string::npos, output.find(
        "XML Element[not_an_sdf_element], child of element[sdf], not "
        "defined in SDF."))
      << output;
    EXPECT_NE(std::string::npos, output.find("Valid."))
      << output;
  }

  // Check an SDFormat file with unrecognized elements with XML namespaces
  {
    const auto path =
      sdf::testing::TestFile("sdf", "unrecognized_elements_with_namespace.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(std::string::npos, output.find(
        "XML Attribute[some_attribute] in element[model] not defined in SDF."))
      << output;
    EXPECT_EQ(std::string::npos, output.find(
        "XML Element["))
      << output;
    EXPECT_NE(std::string::npos, output.find("Valid."))
      << output;
  }
}

/////////////////////////////////////////////////
TEST(warningsPolicy, SDF)
{
  const auto path = sdf::testing::TestFile("sdf", "warnings_policy.sdf");
  const std::string warning =
      "Inertial was used with auto=true for the link named compound_link";

  // Preserve today's default: warnings are printed and parsing succeeds.
  std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
  EXPECT_PRED2(sdf::testing::contains, output, warning);
  EXPECT_PRED2(sdf::testing::contains, output, "Valid.");

  // ERR promotes the warning into the returned Errors collection, so check
  // fails before printing Valid.
  output = custom_exec_str(GzCommand() + " sdf -k " + path +
      " --warnings-policy err" + SdfVersion());
  EXPECT_PRED2(sdf::testing::contains, output, warning);
  EXPECT_PRED2(sdf::testing::notContains, output, "Valid.");

  // LOG keeps the parse successful but moves the warning to debug output.
  output = custom_exec_str(GzCommand() + " sdf -k " + path +
      " --warnings-policy log" + SdfVersion());
  EXPECT_PRED2(sdf::testing::notContains, output, warning);
  EXPECT_PRED2(sdf::testing::contains, output, "Valid.");

  // Print uses the same parser policy rather than a separate/default config.
  output = custom_exec_str(GzCommand() + " sdf -p " + path +
      " --warnings-policy log" + SdfVersion());
  EXPECT_PRED2(sdf::testing::notContains, output, warning);
  EXPECT_PRED2(sdf::testing::contains, output,
               "<model name='compound_model'>");
}


/////////////////////////////////////////////////
TEST(check, SDF)
{
  // Check a good SDF file
  {
    const auto path =
      sdf::testing::TestFile("sdf", "box_plane_low_friction_test.world");

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check a bad SDF file
  {
    const auto path =
      sdf::testing::TestFile("sdf", "box_bad_test.world");

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Required attribute"), std::string::npos)
      << output;
  }

  // Check an SDF file with sibling elements of the same type (world)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "world_duplicate.sdf");

    // Check world_duplicate.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("World with name[default] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (model, light)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "world_sibling_same_names.sdf");

    // Check world_sibling_same_names.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Non-unique names"), std::string::npos)
      << output;
  }
  // Check an SDF world file is allowed to have duplicate plugin names
  {
    const auto path =
      sdf::testing::TestFile("sdf", "world_duplicate_plugins.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with sibling elements of the same type (link)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_duplicate_links.sdf");

    // Check model_duplicate_links.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("link with name[link] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (joint)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_duplicate_joints.sdf");

    // Check model_duplicate_joints.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("joint with name[joint] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of different types (link, joint)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_link_joint_same_name.sdf");

    // Check model_link_joint_same_name.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Non-unique names"), std::string::npos)
      << output;
  }

  // Check an SDF model file is allowed to have duplicate plugin names
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_duplicate_plugins.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with sibling elements of the same type (collision)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "link_duplicate_sibling_collisions.sdf");

    // Check link_duplicate_sibling_collisions.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("collision with name[collision] "
                          "already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with sibling elements of the same type (visual)
  // that have duplicate names.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "link_duplicate_sibling_visuals.sdf");

    // Check link_duplicate_sibling_visuals.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("visual with name[visual] already exists."),
              std::string::npos) << output;
  }

  // Check an SDF file with cousin elements of the same type (collision)
  // that have duplicate names. This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "link_duplicate_cousin_collisions.sdf");

    // Check link_duplicate_cousin_collisions.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with cousin elements of the same type (visual)
  // that have duplicate names. This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "link_duplicate_cousin_visuals.sdf");

    // Check link_duplicate_cousin_visuals.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a joint with an invalid child link.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_invalid_child.sdf");

    // Check joint_invalid_child.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Child frame with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_child]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with an invalid parent link.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_invalid_parent.sdf");

    // Check joint_invalid_parent.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("parent frame with name[invalid] specified by "
                          "joint with name[joint] not found in model with "
                          "name[joint_invalid_parent]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint that names itself as the child frame.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_invalid_self_child.sdf");

    // Check joint_invalid_self_child.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("FrameAttachedToGraph cycle detected, already "
                          "visited vertex [joint_invalid_self_child::self]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint that names itself as the parent frame.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_invalid_self_parent.sdf");

    // Check joint_invalid_self_parent.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("joint with name[self] in model with "
                          "name[joint_invalid_self_parent] must not specify "
                          "its own name as the parent frame."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with identical parent and child.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "joint_invalid_parent_same_as_child.sdf");

    // Check joint_invalid_parent_same_as_child.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Joint with name[joint] must "
                          "specify different frame names for parent and child, "
                          "while [link] was specified for both."),
              std::string::npos) << output;
  }

  // Check an SDF file with a joint with parent parent frame that resolves
  // to the same value as the child.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "joint_invalid_resolved_parent_same_as_child.sdf");

    // Check joint_invalid_resolved_parent_same_as_child.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("specified parent frame [J1] and child frame [L2] "
                          "that both resolve to [L2], but they should resolve "
                          "to different values."),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a child link.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_child_world.sdf");

    // Check joint_child_world.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("Joint with name[joint] specified invalid "
                          "child link [world]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("Child frame with name[world] specified by joint "
                          "with name[joint] not found in model"),
              std::string::npos) << output;
  }

  // Check an SDF file with the world specified as a parent link.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_parent_world.sdf");

    // Check joint_parent_world.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a frame specified as the joint child.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_child_frame.sdf");

    // Check joint_child_frame.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a frame specified as the joint parent.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_parent_frame.sdf");

    // Check joint_parent_frame.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with the infinite values for joint axis limits.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "joint_axis_infinite_limits.sdf");

    // Check joint_axis_infinite_limits.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with the second link specified as the canonical link.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_canonical_link.sdf");

    // Check model_canonical_link.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with an invalid link specified as the canonical link.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "model_invalid_canonical_link.sdf");

    // Check model_invalid_canonical_link.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("canonical_link with name[link3] not found in "
                          "model with name[model_invalid_canonical_link]."),
              std::string::npos) << output;
  }

  // Check an SDF file with an invalid model without links.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_without_links.sdf");

    // Check model_without_links.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an SDF file with a nested model.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "nested_model.sdf");

    // Check nested_model.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "nested_canonical_link.sdf");

    // Check nested_canonical_link.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that has a nested canonical link
  // that is explicitly specified by //model/@canonical_link using ::
  // syntax.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "nested_explicit_canonical_link.sdf");

    // Check nested_explicit_canonical_link.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with a model that a nested model without a link.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "nested_without_links_invalid.sdf");

    // Check nested_without_links_invalid.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("A model must have at least one link."),
              std::string::npos) << output;
  }

  // Check an invalid SDF file that uses reserved names.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_reserved_names.sdf");

    // Check model_invalid_reserved_names.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("The supplied link name [world] is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied link name [__link__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied visual name [__visual__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied collision name [__collision__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied joint name [__joint__] "
                          "is reserved."),
              std::string::npos) << output;
    EXPECT_NE(output.find("The supplied frame name [__frame__] "
                          "is reserved."),
              std::string::npos) << output;
  }

  // Check that validity checks are disabled inside <plugin> elements
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "ignore_sdf_in_plugin.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check that validity checks are disabled inside namespaced elements
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "ignore_sdf_in_namespaced_elements.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_attached_to.sdf");

    // Check model_frame_attached_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to joints.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_attached_to_joint.sdf");

    // Check model_frame_attached_to_joint.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames attached_to a nested model.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_attached_to_nested_model.sdf");

    // Check model_frame_attached_to_nested_model.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid attached_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_invalid_attached_to.sdf");

    // Check model_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("attached_to name[A] specified by frame with "
                          "name[F3] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("attached_to name[F4] is identical to frame "
                          "name[F4], causing a graph cycle in model with "
                          "name[model_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its FrameAttachedTo graph
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_invalid_attached_to_cycle.sdf");

    // Check model_frame_invalid_attached_to_cycle.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(std::string::npos,
        output.find("FrameAttachedToGraph cycle detected, already visited "
                    "vertex [model_frame_invalid_attached_to_cycle::F1]."))
        << output;
    EXPECT_NE(std::string::npos,
        output.find("FrameAttachedToGraph cycle detected, already visited "
                    "vertex [model_frame_invalid_attached_to_cycle::F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "world_frame_attached_to.sdf");

    // Check world_frame_attached_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid attached_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "world_frame_invalid_attached_to.sdf");

    // Check world_frame_invalid_attached_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("attached_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("attached_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_attached_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with links using the relative_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_link_relative_to.sdf");

    // Check model_link_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model links with invalid relative_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_link_relative_to.sdf");

    // Check model_invalid_link_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("relative_to name[A] specified by link with "
                          "name[L] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[self_cycle] is identical to "
                          "link name[self_cycle], causing a graph cycle in "
                          "model with name[model_invalid_link_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with nested_models using the relative_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_nested_model_relative_to.sdf");

    // Check model_nested_model_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with nested_models using nested links/frames as joint
  // parent or child frames.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "joint_nested_parent_child.sdf");

    // Check model_nested_model_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with joints using the relative_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_joint_relative_to.sdf");

    // Check model_joint_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model joints with invalid relative_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_joint_relative_to.sdf");

    // Check model_invalid_joint_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("relative_to name[A] specified by joint with "
                          "name[J] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[Jcycle] is identical to "
                          "joint name[Jcycle], causing a graph cycle in "
                          "model with name[model_invalid_joint_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with model frames using the relative_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_relative_to.sdf");

    // Check model_frame_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames relative_to joints.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_frame_relative_to_joint.sdf");

    // Check model_frame_relative_to_joint.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with model frames with invalid relative_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_frame_relative_to.sdf");

    // Check model_invalid_frame_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("relative_to name[A] specified by frame with "
                          "name[F] does not match a nested model, link, "
                          "joint, or frame name in model with "
                          "name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[cycle] is identical to "
                          "frame name[cycle], causing a graph cycle in model "
                          "with name[model_invalid_frame_relative_to]."),
              std::string::npos) << output;
  }

  // Check an SDF file with a cycle in its PoseRelativeTo graph
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_frame_relative_to_cycle.sdf");

    // Check model_invalid_frame_relative_to_cycle.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(std::string::npos,
        output.find("PoseRelativeToGraph cycle detected, already visited "
                    "vertex [model_invalid_frame_relative_to_cycle::F1]."))
        << output;
    EXPECT_NE(std::string::npos,
        output.find("PoseRelativeToGraph cycle detected, already visited "
                    "vertex [model_invalid_frame_relative_to_cycle::F2]."))
        << output;
  }

  // Check an SDF file with model frames using the attached_to attribute.
  // This is a valid file.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "world_frame_relative_to.sdf");

    // Check world_frame_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF file with world frames with invalid relative_to attributes.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "world_frame_invalid_relative_to.sdf");

    // Check world_frame_invalid_relative_to.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(output.find("relative_to name[A] specified by model with "
                          "name[M] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[cycle] is identical "
                          "to model name[cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[A] specified by frame with "
                          "name[F] does not match a model or frame "
                          "name in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
    EXPECT_NE(output.find("relative_to name[self_cycle] is identical "
                          "to frame name[self_cycle], causing a graph cycle "
                          "in world with "
                          "name[world_frame_invalid_relative_to]."),
              std::string::npos) << output;
  }
  // Check an SDF file with an invalid frame specified as the placement_frame
  // attribute.
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_placement_frame.sdf");

    // Check model_invalid_placement_frame.sdf
    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(
        output.find("unable to find unique frame with name [link3] in graph"),
        std::string::npos)
        << output;
  }

  // Check an SDF file with an valid nested model cross references
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "nested_model_cross_references.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }

  // Check an SDF model file with an invalid usage of __root__
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_root_reference.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [relative_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a"
                    " value of attribute [attached_to]"),
        std::string::npos)
        << output;
  }
  // Check an SDF world file with an invalid usage of __root__
  {
    // Set SDF_PATH so that included models can be found
    gz::utils::setenv(
      "SDF_PATH", sdf::testing::TestFile("integration", "model"));

    const auto path =
      sdf::testing::TestFile("sdf",
          "world_invalid_root_reference.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [relative_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [attached_to]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [placement_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [canonical_link]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of attribute [parent_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find("'__root__' is reserved; it cannot be used as a "
                    "value of element [placement_frame]"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find(
            "The supplied joint child name [__root__] is not valid"),
        std::string::npos)
        << output;
    EXPECT_NE(
        output.find(
            "The supplied joint parent name [__root__] is not valid"),
        std::string::npos)
        << output;
  }
  // Check an SDF world file with an valid usage of __root__
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "world_valid_root_reference.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output) << output;
  }
  // Check an SDF with an invalid relative frame at the top level model
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "model_invalid_top_level_frame.sdf");

    std::string output =
        custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_NE(
        output.find("Attribute //pose[@relative_to] of top level model must be "
                    "left empty, found //pose[@relative_to='some_frame']."),
        std::string::npos) << output;
  }
}

/////////////////////////////////////////////////
TEST(check_shapes_sdf, SDF)
{
  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "shapes.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output);
  }

  {
    const auto path =
      sdf::testing::TestFile("sdf",
          "shapes_world.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output);
  }
}

/////////////////////////////////////////////////
TEST(check_model_sdf, SDF)
{
  // Check a good SDF file by passing the absolute path
  {
    const auto path =
      sdf::testing::TestFile("integration",
          "model", "box", "model.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output);
  }

  // Check a good SDF file from the same folder by passing a relative path
  {
    const auto pathBase = sdf::testing::TestFile("integration",
        "model", "box");
    const auto path = "model.sdf";

    std::string output =
      custom_exec_str("cd " + pathBase + " && " +
                      GzCommand() + " sdf -k " + path + SdfVersion());
    EXPECT_EQ("Valid.\n", output);
  }
}

/////////////////////////////////////////////////
TEST(describe, SDF)
{
  // Get the description
  std::string output =
    custom_exec_str(GzCommand() + " sdf -d " + SdfVersion());
  EXPECT_FALSE(output.empty());

  // The first line should start with the following text.
  EXPECT_EQ(0u, output.find("<element name ='sdf' required ='1'"));
}

/////////////////////////////////////////////////
TEST(print, SDF)
{
  // Check a good SDF file
  {
    const auto path =
      sdf::testing::TestFile("sdf", "box_plane_low_friction_test.world");
    sdf::SDFPtr sdf(new sdf::SDF());
    EXPECT_TRUE(sdf::init(sdf));
    EXPECT_TRUE(sdf::readFile(path, sdf));

    // Check box_plane_low_friction_test.world
    std::string output =
      custom_exec_str(GzCommand() + " sdf -p " + path + SdfVersion());
    EXPECT_EQ(sdf->Root()->ToString(""), output);
  }

  // Check a bad SDF file
  {
    const auto path =
      sdf::testing::TestFile("sdf", "box_bad_test.world");

    // Check box_bad_test.world
    std::string output =
      custom_exec_str(GzCommand() + " sdf -p " + path + SdfVersion());
    EXPECT_TRUE(output.find("Required attribute") != std::string::npos)
      << output;
  }
}

/////////////////////////////////////////////////
TEST(print_rotations_in_degrees, SDF)
{
  const std::string path =
      sdf::testing::TestFile("sdf", "rotations_in_degrees.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true'>1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_rotations_in_radians, SDF)
{
  const std::string path =
      sdf::testing::TestFile("sdf", "rotations_in_radians.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose>1 2 3 0.523756 0.785241 -1.04735</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.0087</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.0087</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_rotations_in_quaternions, SDF)
{
  const auto path = sdf::testing::TestFile(
      "sdf", "rotations_in_quaternions.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose rotation_format='quat_xyzw'>"
               "1 2 3   0.391948 0.200425 -0.532046 0.723279</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_includes_rotations_in_degrees, SDF)
{
  // Set SDF_PATH so that included models can be found
  gz::utils::setenv(
    "SDF_PATH", sdf::testing::SourceFile("test", "integration", "model"));
  const std::string path =
      sdf::testing::TestFile("sdf", "includes_rotations_in_degrees.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true'>1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_includes_rotations_in_radians, SDF)
{
  // Set SDF_PATH so that included models can be found
  gz::utils::setenv(
    "SDF_PATH", sdf::testing::SourceFile("test", "integration", "model"));
  const std::string path =
      sdf::testing::TestFile("sdf", "includes_rotations_in_radians.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose>1 2 3 0.523756 0.785241 -1.04735</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.0087</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.0087</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_includes_rotations_in_quaternions, SDF)
{
  // Set SDF_PATH so that included models can be found
  gz::utils::setenv(
    "SDF_PATH", sdf::testing::TestFile("integration", "model"));
  const auto path = sdf::testing::TestFile(
      "sdf", "includes_rotations_in_quaternions.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose rotation_format='quat_xyzw'>"
               "1 2 3   0.391948 0.200425 -0.532046 0.723279</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_rotations_in_unnormalized_degrees, SDF)
{
  const std::string path =
      sdf::testing::TestFile("sdf", "rotations_in_unnormalized_degrees.sdf");

  // Default printing
  // Unnormalized degree values cannot be returned as is, as its string is
  // returned by parsing the pose value, whenever a parent Element Attribute,
  // or PrintConfig is used.
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true'>1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.991 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.991 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_rotations_in_unnormalized_radians, SDF)
{
  const std::string path =
      sdf::testing::TestFile("sdf", "rotations_in_unnormalized_radians.sdf");

  // Default printing
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose>1 2 3 0.523755 0.785251 -1.04736</pose>");

  // Printing with in_degrees
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.9915 -60.009</pose>");

  // Printing with snap_to_degrees 5
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // Printing with snap_to_degrees 2
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 2 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 44.9915 -60</pose>");

  // Printing with snap_to_degrees 20
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 20 "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.9915 -60</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.008
  output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --snap-to-degrees 5 "
      + "--snap-tolerance 0.008 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.9915 -60.009</pose>");

  // Printing with snap_to_degrees 5, snap_tolerance 0.01
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      "--snap-tolerance 0.01 " + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(shuffled_cmd_flags, SDF)
{
  const std::string path =
      sdf::testing::TestFile("sdf", "rotations_in_unnormalized_radians.sdf");

  // -p PATH --degrees
  std::string output = custom_exec_str(
      GzCommand() + " sdf -p --precision 6 " + path + " --degrees "
      + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.9915 -60.009</pose>");

  // --degrees -p PATH
  output = custom_exec_str(
      GzCommand() + " sdf --degrees -p --precision 6 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30.009 44.9915 -60.009</pose>");

  // -p PATH --snap-to-degrees ARG
  output = custom_exec_str(
      GzCommand() + " sdf -p " + path + " --snap-to-degrees 5 " +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // -p --snap-to-degrees ARG PATH
  output = custom_exec_str(
      GzCommand() + " sdf -p --snap-to-degrees 5 " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");

  // --snap-to-degrees ARG -p PATH
  output = custom_exec_str(
      GzCommand() + " sdf --snap-to-degrees 5 -p " + path + SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 45 -60</pose>");
}

/////////////////////////////////////////////////
TEST(print_snap_to_degrees_tolerance_too_high, SDF)
{
  const std::string path = sdf::testing::TestFile(
      "sdf",
      "rotations_in_degrees_high_snap_tolerance.sdf");

  std::string output = custom_exec_str(
      GzCommand() + " sdf -p " + path +
      " --snap-to-degrees 5 " + " --snap-tolerance 4" +
      SdfVersion());
  ASSERT_FALSE(output.empty());
  EXPECT_PRED2(sdf::testing::contains, output,
               "<pose degrees='true' rotation_format='euler_rpy'>"
               "1 2 3   30 50 60</pose>");
}

/////////////////////////////////////////////////
TEST(print_auto_inertial, SDF)
{
  const auto path = sdf::testing::TestFile("sdf", "inertial_stats_auto.sdf");

  {
    // Print without --expand-auto-inertials
    // expect no <mass> or <inertia> elements
    std::string output = custom_exec_str(
        GzCommand() + " sdf -p " + path +
        SdfVersion());
    ASSERT_FALSE(output.empty());
    EXPECT_PRED2(sdf::testing::notContains, output, "<mass>");
    EXPECT_PRED2(sdf::testing::notContains, output, "<inertia>");
  }

  {
    // Print with --expand-auto-inertials
    // expect <mass> and <inertia> elements
    std::string output = custom_exec_str(
        GzCommand() + " sdf -p " + path +
        " --expand-auto-inertials " +
        SdfVersion());
    ASSERT_FALSE(output.empty());
    EXPECT_PRED2(sdf::testing::contains, output, "<mass>");
    EXPECT_PRED2(sdf::testing::contains, output, "<inertia>");
  }
}

/////////////////////////////////////////////////
TEST(GraphCmd, WorldPoseRelativeTo)
{
  // world pose relative_to graph
  const std::string path =
    sdf::testing::TestFile("sdf", "world_relative_to_nested_reference.sdf");

  const std::string output =
    custom_exec_str(GzCommand() + " sdf -g pose " + path + SdfVersion());

  std::stringstream expected;
  expected << "digraph {\n"
    << "  0 [label=\"__root__ (0)\"];\n"
    << "  1 [label=\"world (1)\"];\n"
    << "  2 [label=\"M1 (2)\"];\n"
    << "  3 [label=\"M1::__model__ (3)\"];\n"
    << "  4 [label=\"M1::L1 (4)\"];\n"
    << "  5 [label=\"M1::L2 (5)\"];\n"
    << "  6 [label=\"M1::J1 (6)\"];\n"
    << "  7 [label=\"M1::F1 (7)\"];\n"
    << "  8 [label=\"M1::CM1 (8)\"];\n"
    << "  9 [label=\"M1::CM1::__model__ (9)\"];\n"
    << "  10 [label=\"M1::CM1::L (10)\"];\n"
    << "  11 [label=\"F2 (11)\"];\n"
    << "  12 [label=\"F3 (12)\"];\n"
    << "  13 [label=\"F4 (13)\"];\n"
    << "  14 [label=\"F5 (14)\"];\n"
    << "  15 [label=\"F6 (15)\"];\n"
    << "  16 [label=\"F7 (16)\"];\n"
    << "  0 -> 1 [label=1];\n"
    << "  2 -> 3 [label=0];\n"
    << "  8 -> 9 [label=0];\n"
    << "  9 -> 10 [label=1];\n"
    << "  3 -> 4 [label=1];\n"
    << "  3 -> 5 [label=1];\n"
    << "  5 -> 6 [label=1];\n"
    << "  5 -> 7 [label=1];\n"
    << "  3 -> 8 [label=1];\n"
    << "  1 -> 2 [label=1];\n"
    << "  2 -> 11 [label=1];\n"
    << "  4 -> 12 [label=1];\n"
    << "  6 -> 13 [label=1];\n"
    << "  7 -> 14 [label=1];\n"
    << "  8 -> 15 [label=1];\n"
    << "  10 -> 16 [label=1];\n"
    << "}";
  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, ModelPoseRelativeTo)
{
  const auto path =
    sdf::testing::TestFile("sdf", "model_relative_to_nested_reference.sdf");

  const std::string output =
    custom_exec_str(GzCommand() + " sdf -g pose " + path + SdfVersion());

  std::stringstream expected;
  expected << "digraph {\n"
    << "  0 [label=\"__root__ (0)\"];\n"
    << "  1 [label=\"parent_model (1)\"];\n"
    << "  2 [label=\"parent_model::__model__ (2)\"];\n"
    << "  3 [label=\"parent_model::L (3)\"];\n"
    << "  4 [label=\"parent_model::M1 (4)\"];\n"
    << "  5 [label=\"parent_model::M1::__model__ (5)\"];\n"
    << "  6 [label=\"parent_model::M1::L1 (6)\"];\n"
    << "  7 [label=\"parent_model::M1::L2 (7)\"];\n"
    << "  8 [label=\"parent_model::M1::J1 (8)\"];\n"
    << "  9 [label=\"parent_model::M1::F1 (9)\"];\n"
    << "  10 [label=\"parent_model::M1::CM1 (10)\"];\n"
    << "  11 [label=\"parent_model::M1::CM1::__model__ (11)\"];\n"
    << "  12 [label=\"parent_model::M1::CM1::L (12)\"];\n"
    << "  13 [label=\"parent_model::M2 (13)\"];\n"
    << "  14 [label=\"parent_model::M2::__model__ (14)\"];\n"
    << "  15 [label=\"parent_model::M2::L (15)\"];\n"
    << "  16 [label=\"parent_model::M3 (16)\"];\n"
    << "  17 [label=\"parent_model::M3::__model__ (17)\"];\n"
    << "  18 [label=\"parent_model::M3::L (18)\"];\n"
    << "  19 [label=\"parent_model::M4 (19)\"];\n"
    << "  20 [label=\"parent_model::M4::__model__ (20)\"];\n"
    << "  21 [label=\"parent_model::M4::L (21)\"];\n"
    << "  22 [label=\"parent_model::M5 (22)\"];\n"
    << "  23 [label=\"parent_model::M5::__model__ (23)\"];\n"
    << "  24 [label=\"parent_model::M5::L (24)\"];\n"
    << "  25 [label=\"parent_model::M6 (25)\"];\n"
    << "  26 [label=\"parent_model::M6::__model__ (26)\"];\n"
    << "  27 [label=\"parent_model::M6::L (27)\"];\n"
    << "  28 [label=\"parent_model::M7 (28)\"];\n"
    << "  29 [label=\"parent_model::M7::__model__ (29)\"];\n"
    << "  30 [label=\"parent_model::M7::L (30)\"];\n"
    << "  1 -> 2 [label=0];\n"
    << "  4 -> 5 [label=0];\n"
    << "  10 -> 11 [label=0];\n"
    << "  11 -> 12 [label=1];\n"
    << "  5 -> 6 [label=1];\n"
    << "  5 -> 7 [label=1];\n"
    << "  7 -> 8 [label=1];\n"
    << "  7 -> 9 [label=1];\n"
    << "  5 -> 10 [label=1];\n"
    << "  13 -> 14 [label=0];\n"
    << "  14 -> 15 [label=1];\n"
    << "  16 -> 17 [label=0];\n"
    << "  17 -> 18 [label=1];\n"
    << "  19 -> 20 [label=0];\n"
    << "  20 -> 21 [label=1];\n"
    << "  22 -> 23 [label=0];\n"
    << "  23 -> 24 [label=1];\n"
    << "  25 -> 26 [label=0];\n"
    << "  26 -> 27 [label=1];\n"
    << "  28 -> 29 [label=0];\n"
    << "  29 -> 30 [label=1];\n"
    << "  2 -> 3 [label=1];\n"
    << "  2 -> 4 [label=1];\n"
    << "  4 -> 13 [label=1];\n"
    << "  6 -> 16 [label=1];\n"
    << "  8 -> 19 [label=1];\n"
    << "  9 -> 22 [label=1];\n"
    << "  10 -> 25 [label=1];\n"
    << "  12 -> 28 [label=1];\n"
    << "  0 -> 1 [label=1];\n"
    << "}";

  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, WorldFrameAttachedTo)
{
  const auto path =
    sdf::testing::TestFile("sdf", "world_nested_frame_attached_to.sdf");
  const std::string output =
      custom_exec_str(GzCommand() + " sdf -g frame " + path + SdfVersion());

  std::stringstream expected;

  expected << "digraph {\n"
           << "  0 [label=\"world (0)\"];\n"
           << "  1 [label=\"M1 (1)\"];\n"
           << "  2 [label=\"M1::__model__ (2)\"];\n"
           << "  3 [label=\"M1::L (3)\"];\n"
           << "  4 [label=\"M1::J1 (4)\"];\n"
           << "  5 [label=\"M1::F0 (5)\"];\n"
           << "  6 [label=\"M1::M2 (6)\"];\n"
           << "  7 [label=\"M1::M2::__model__ (7)\"];\n"
           << "  8 [label=\"M1::M2::L (8)\"];\n"
           << "  9 [label=\"world_frame (9)\"];\n"
           << "  10 [label=\"F0 (10)\"];\n"
           << "  11 [label=\"F1 (11)\"];\n"
           << "  12 [label=\"F2 (12)\"];\n"
           << "  13 [label=\"F3 (13)\"];\n"
           << "  14 [label=\"F4 (14)\"];\n"
           << "  15 [label=\"F5 (15)\"];\n"
           << "  16 [label=\"F6 (16)\"];\n"
           << "  1 -> 2 [label=0];\n"
           << "  6 -> 7 [label=0];\n"
           << "  7 -> 8 [label=1];\n"
           << "  4 -> 5 [label=1];\n"
           << "  5 -> 8 [label=1];\n"
           << "  2 -> 3 [label=1];\n"
           << "  9 -> 0 [label=1];\n"
           << "  10 -> 0 [label=1];\n"
           << "  11 -> 1 [label=1];\n"
           << "  12 -> 3 [label=1];\n"
           << "  13 -> 6 [label=1];\n"
           << "  14 -> 8 [label=1];\n"
           << "  15 -> 5 [label=1];\n"
           << "  16 -> 4 [label=1];\n"
           << "}";
  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

/////////////////////////////////////////////////
TEST(GraphCmd, ModelFrameAttachedTo)
{
  const auto path =
    sdf::testing::TestFile("sdf", "model_nested_frame_attached_to.sdf");

  const std::string output =
      custom_exec_str(GzCommand() + " sdf -g frame " + path + SdfVersion());

  std::stringstream expected;

  expected
      << "digraph {\n"
      << "  0 [label=\"__root__ (0)\"];\n"
      << "  1 [label=\"nested_model_frame_attached_to (1)\"];\n"
      << "  2 [label=\"nested_model_frame_attached_to::__model__ (2)\"];\n"
      << "  3 [label=\"nested_model_frame_attached_to::L (3)\"];\n"
      << "  4 [label=\"nested_model_frame_attached_to::F0 (4)\"];\n"
      << "  5 [label=\"nested_model_frame_attached_to::F1 (5)\"];\n"
      << "  6 [label=\"nested_model_frame_attached_to::F2 (6)\"];\n"
      << "  7 [label=\"nested_model_frame_attached_to::F3 (7)\"];\n"
      << "  8 [label=\"nested_model_frame_attached_to::F4 (8)\"];\n"
      << "  9 [label=\"nested_model_frame_attached_to::M1 (9)\"];\n"
      << "  10 [label=\"nested_model_frame_attached_to::M1::__model__ "
         "(10)\"];\n"
      << "  11 [label=\"nested_model_frame_attached_to::M1::L (11)\"];\n"
      << "  12 [label=\"nested_model_frame_attached_to::M1::J1 (12)\"];\n"
      << "  13 [label=\"nested_model_frame_attached_to::M1::F (13)\"];\n"
      << "  14 [label=\"nested_model_frame_attached_to::M1::M2 (14)\"];\n"
      << "  15 [label=\"nested_model_frame_attached_to::M1::M2::__model__ "
         "(15)\"];\n"
      << "  16 [label=\"nested_model_frame_attached_to::M1::M2::L (16)\"];\n"
      << "  1 -> 2 [label=0];\n"
      << "  9 -> 10 [label=0];\n"
      << "  14 -> 15 [label=0];\n"
      << "  15 -> 16 [label=1];\n"
      << "  12 -> 13 [label=1];\n"
      << "  13 -> 14 [label=1];\n"
      << "  10 -> 11 [label=1];\n"
      << "  4 -> 9 [label=1];\n"
      << "  5 -> 11 [label=1];\n"
      << "  6 -> 5 [label=1];\n"
      << "  7 -> 13 [label=1];\n"
      << "  8 -> 12 [label=1];\n"
      << "  2 -> 3 [label=1];\n"
      << "}";

  EXPECT_EQ(sdf::trim(expected.str()), sdf::trim(output));
}

// Disable on arm
#if !defined __ARM_ARCH
/////////////////////////////////////////////////
TEST(inertial_stats, SDF)
{
  std::string expectedOutput =
    "Inertial statistics for model: test_model\n"
    "---\n"
    "Total mass of the model: 24\n"
    "---\n"
    "Centre of mass in model frame: \n"
    "X: 0\n"
    "Y: 0\n"
    "Z: 0\n"
    "---\n"
    "Moment of inertia matrix: \n"
    "304  0    0    \n"
    "0    304  0    \n"
    "0    0    604  \n"
    "---\n";

  // Check a good SDF file by passing the absolute path
  {
    const auto path = sdf::testing::TestFile("sdf", "inertial_stats.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf --inertial-stats " +
                      path + SdfVersion());
    EXPECT_EQ(expectedOutput, output);
  }

  // Check a good SDF file from the same folder by passing a relative path
  {
    std::string path = "inertial_stats_auto.sdf";
    const auto pathBase = sdf::testing::TestFile("sdf");

    std::string output =
      custom_exec_str("cd " + pathBase + " && " +
                      GzCommand() + " sdf --inertial-stats " +
                      path + SdfVersion());
    EXPECT_EQ(expectedOutput, output);
  }

  // Check a good SDF file with auto-inertials and explicit mass
  // from the same folder by passing a relative path
  {
    std::string path = "inertial_stats_auto_mass.sdf";
    const auto pathBase = sdf::testing::TestFile("sdf");

    std::string output =
      custom_exec_str("cd " + pathBase + " && " +
                      GzCommand() + " sdf --inertial-stats " +
                      path + SdfVersion());
    EXPECT_EQ(expectedOutput, output);
  }

  expectedOutput =
          "Error Code " +
          std::to_string(static_cast<int>(
              sdf::ErrorCode::LINK_INERTIA_INVALID)) +
          ": Msg: A link named link has invalid inertia.\n\n"
          "Inertial statistics for model: model\n"
          "---\n"
          "Total mass of the model: 0\n"
          "---\n"
          "Centre of mass in model frame: \n"
          "X: 0\n"
          "Y: 0\n"
          "Z: 0\n"
          "---\n"
          "Moment of inertia matrix: \n"
          "0  0  0  \n"
          "0  0  0  \n"
          "0  0  0  \n"
          "---\n";

  // Check an invalid SDF file by passing the absolute path
  {
    const auto path = sdf::testing::TestFile("sdf", "inertial_invalid.sdf");

    std::string output =
      custom_exec_str(GzCommand() + " sdf --inertial-stats " +
                      path + SdfVersion());
    EXPECT_EQ(expectedOutput, output);
  }

  expectedOutput =
          "Error: Expected a model file but received a world file.\n";
  // Check a valid world file.
  {
    const auto path =
      sdf::testing::TestFile("sdf", "box_plane_low_friction_test.world");

    std::string output =
      custom_exec_str(GzCommand() + " sdf --inertial-stats " +
                      path + SdfVersion());
    EXPECT_EQ(expectedOutput, output);
  }
}
// #if !defined __ARM_ARCH
#endif

//////////////////////////////////////////////////
/// \brief Check help message and bash completion script for consistent flags
TEST(HelpVsCompletionFlags, GZ_UTILS_TEST_DISABLED_ON_WIN32(SDF))
{
  // Flags in help message
  std::string helpOutput = custom_exec_str(GzCommand() + " sdf --help");

  // Call the output function in the bash completion script
  const auto scriptPath =
    sdf::testing::SourceFile("src", "cmd", "sdf.bash_completion.sh");

  // Equivalent to:
  // sh -c "bash -c \". /path/to/sdf.bash_completion.sh; _gz_sdf_flags\""
  std::string cmd = "bash -c \". " + scriptPath +
    "; _gz_sdf_flags\"";
  std::string scriptOutput = custom_exec_str(cmd);

  // Tokenize script output
  std::istringstream iss(scriptOutput);
  std::vector<std::string> flags((std::istream_iterator<std::string>(iss)),
    std::istream_iterator<std::string>());

  EXPECT_GT(flags.size(), 0u);

  // Match each flag in script output with help message
  for (const auto &flag : flags)
  {
    EXPECT_NE(std::string::npos, helpOutput.find(flag)) << helpOutput;
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  // Set GZ_CONFIG_PATH to the directory where the .yaml configuration file
  // is located.
  gz::utils::setenv("GZ_CONFIG_PATH", GZ_CONFIG_PATH);

  // Make sure that we load the library recently built and not the one installed
  // in your system. This is done by placing the the current build directory
  // first in the LD_LIBRARY_PATH environment variable.
  //
  // We need to keep the existing LD_LIBRARY_PATH so that libsdformat.so can
  // find its dependency.
#ifndef _WIN32
  std::string testLibraryPath = GZ_TEST_LIBRARY_PATH;

  std::string currentLibraryPath;
  if (gz::utils::env("LD_LIBRARY_PATH", currentLibraryPath))
  {
    testLibraryPath = testLibraryPath + ":" + currentLibraryPath;
  }

  gz::utils::setenv("LD_LIBRARY_PATH", testLibraryPath);
#endif

  // temporarily set HOME
  std::string homeDir;
  sdf::testing::TestTmpPath(homeDir);

#ifdef _WIN32
  gz::utils::setenv("HOMEPATH", homeDir);
#else
  gz::utils::setenv("HOME", homeDir);
#endif

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
