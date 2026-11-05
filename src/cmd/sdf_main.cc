/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>

#include "sdf/config.hh"
#include "sdf/ParserConfig.hh"
#include "gz.hh"

using namespace sdf;

//////////////////////////////////////////////////
/// \brief Enumeration of available commands
enum class SdfCommand
{
  kNone,
  kSdfCheck,
  kSdfDescribe,
  kSdfGraph,
  kSdfPrintInertialStats,
  kSdfPrint
};

//////////////////////////////////////////////////
/// \brief Structure to hold all available topic options
struct SdfOptions
{
  /// \brief Command to execute
  SdfCommand command{SdfCommand::kNone};

  /// \brief Path to the SDFormat file to print or check
  std::string filepath{""};

  /// \brief Version of the SDFormat specification to describe
  std::string version{""};

  /// \brief Type of SDFormat graph to print:
  /// * "frame" for FrameAttachedToGraph
  /// * "pose" for PoseRelativeToGraph
  std::string graphType{""};

  /// \brief When true, preserve included tags when printing converted arg
  /// (does not preserve merge-includes)
  bool preserveIncludes{false};

  /// \brief When true, print pose rotations in degrees.
  bool degrees{false};

  /// \brief When true, auto-computed inertial values will be printed.
  bool expandAutoInertials{false};

  /// \brief Output stream precision for floating point numbers.
  int precision{0};

  /// \brief If set, printed rotations are snapped to specified degree
  /// intervals.
  int snapToDegrees{0};

  /// \brief Printed rotations are snapped if they are less than this specified
  /// tolerance.
  double snapTolerance{0.01};

  /// \\brief Policy used to handle parser warnings for check / print.
  sdf::EnforcementPolicy warningsPolicy{sdf::EnforcementPolicy::WARN};
};

//////////////////////////////////////////////////
/// \brief Callback fired when options are successfully parsed
void runSdfCommand(const SdfOptions &_opt)
{
  switch(_opt.command)
  {
    case SdfCommand::kSdfCheck:
      cmdCheck(_opt.filepath.c_str(), _opt.warningsPolicy);
      break;
    case SdfCommand::kSdfDescribe:
      cmdDescribe(_opt.version.c_str());
      break;
    case SdfCommand::kSdfGraph:
      cmdGraph(_opt.graphType.c_str(), _opt.filepath.c_str());
      break;
    case SdfCommand::kSdfPrintInertialStats:
      cmdInertialStats(_opt.filepath.c_str());
      break;
    case SdfCommand::kSdfPrint:
      cmdPrint(_opt.filepath.c_str(), _opt.degrees, _opt.snapToDegrees,
               _opt.snapTolerance, _opt.preserveIncludes, _opt.precision,
               _opt.expandAutoInertials, _opt.warningsPolicy);
      break;
    case SdfCommand::kNone:
    default:
      // In the event that there is no command, display help
      throw CLI::CallForHelp();
  }
}

//////////////////////////////////////////////////
void addSdfFlags(CLI::App &_app)
{
  auto opt = std::make_shared<SdfOptions>();

  auto filepathOpt =
    _app.add_option("filepath", opt->filepath,
                    "Path to an SDFormat file.");

  auto command = _app.add_option_group("command", "Command to be executed.");

  command->add_flag_callback("-k,--check",
    [opt](){
      opt->command = SdfCommand::kSdfCheck;
    },
    "Check if an SDFormat file is valid.")
    ->needs(filepathOpt);

  auto describeCmd = command->add_option_function<std::string>("-d,--describe",
    [opt](const std::string &_version){
      opt->command = SdfCommand::kSdfDescribe;
      opt->version = _version;
    },
    "Print the aggregated SDFormat spec description. Latest version ("
    SDF_PROTOCOL_VERSION ")")
    ->expected(0, 1)
    ->default_val(SDF_PROTOCOL_VERSION);

  auto graphCmd = command->add_option_function<std::string>("-g,--graph",
    [opt](const std::string &_graphType){
      opt->command = SdfCommand::kSdfGraph;
      opt->graphType = _graphType;
    },
    "Print the PoseRelativeTo or FrameAttachedTo graph by following\n"
    "with either pose or frame as argument respectively.\n"
    "(WARNING: This is for advanced use only and the output may change\n"
    "without any promise of stability)")
    ->needs(filepathOpt)
    ->check(CLI::IsMember({"pose", "frame"}));

  auto inertialStatsCmd = command->add_flag_callback("--inertial-stats",
    [opt](){
      opt->command = SdfCommand::kSdfPrintInertialStats;
    },
    "Prints moment of inertia, centre of mass, and total mass from a model "
    "sdf file.")
    ->needs(filepathOpt);

  auto printCmd = command->add_flag_callback("-p,--print",
    [opt](){
      opt->command = SdfCommand::kSdfPrint;
    },
    "Print converted arg. Note the quaternion representation of the\n"
    "rotational part of poses and unit vectors will be normalized.")
    ->needs(filepathOpt);

  auto warningsPolicyOpt = _app.add_option_function<std::string>(
    "--warnings-policy",
    [opt](const std::string &_policy)
    {
      if (_policy == "err")
      {
        opt->warningsPolicy = sdf::EnforcementPolicy::ERR;
      }
      else if (_policy == "log")
      {
        opt->warningsPolicy = sdf::EnforcementPolicy::LOG;
      }
      else
      {
        opt->warningsPolicy = sdf::EnforcementPolicy::WARN;
      }
    },
    "Handle parser warnings as errors, warnings, or debug logs.")
    ->check(CLI::IsMember({"err", "warn", "log"}));
  warningsPolicyOpt->excludes(describeCmd);
  warningsPolicyOpt->excludes(graphCmd);
  warningsPolicyOpt->excludes(inertialStatsCmd);

  _app.add_flag("-i,--preserve-includes", opt->preserveIncludes,
      "Preserve included tags when printing converted arg (does "
      "not preserve merge-includes).")
      ->needs(printCmd);

  _app.add_flag("--degrees", opt->degrees,
      "Printed pose rotations are will be in degrees.")
      ->needs(printCmd);

  _app.add_flag("--expand-auto-inertials", opt->expandAutoInertials,
      "Auto-computed inertial values will be printed.")
      ->needs(printCmd);

  _app.add_option("--precision", opt->precision,
      "Set the output stream precision for floating point "
      "numbers.")
      ->needs(printCmd);

  _app.add_option("--snap-to-degrees", opt->snapToDegrees,
      "Printed rotations are snapped to specified degree "
      "intervals.")
      ->needs(printCmd);

  _app.add_option("--snap-tolerance", opt->snapTolerance,
      "Printed rotations are snapped if they are within this "
      "specified tolerance.")
      ->needs(printCmd);

  _app.callback([opt](){runSdfCommand(*opt); });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Utilities for SDFormat files."};

  app.add_flag_callback("-v,--version", [](){
      std::cout << SDF_VERSION_FULL << std::endl;
      throw CLI::Success();
    },
    "Print the current library version");

  // Dummy flags handled by gz-tools
  app.add_flag("--force-version", "Use a specific library version.");
  app.add_flag("--versions", "Show the available versions.");

  addSdfFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
