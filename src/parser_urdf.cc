/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <algorithm>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ignition/math.hh>

#include <urdf_model/model.h>
#include <urdf_model/link.h>
#include <urdf_parser/urdf_parser.h>

#include "sdf/sdf.hh"

#include "XmlUtils.hh"
#include "SDFExtension.hh"
#include "parser_urdf.hh"

using namespace sdf;

namespace sdf {
inline namespace SDF_VERSION_NAMESPACE {

using XMLDocumentPtr = std::shared_ptr<tinyxml2::XMLDocument>;
using XMLElementPtr = std::shared_ptr<tinyxml2::XMLElement>;

typedef std::shared_ptr<SDFExtension> SDFExtensionPtr;
typedef std::map<std::string, std::vector<SDFExtensionPtr> >
  StringSDFExtensionPtrMap;

/// create SDF geometry block based on URDF
StringSDFExtensionPtrMap g_extensions;
bool g_reduceFixedJoints;
bool g_enforceLimits;
std::string g_collisionExt = "_collision";
std::string g_visualExt = "_visual";
std::string g_lumpPrefix = "_fixed_joint_lump__";
urdf::Pose g_initialRobotPose;
bool g_initialRobotPoseValid = false;
std::set<std::string> g_fixedJointsTransformedInRevoluteJoints;
std::set<std::string> g_fixedJointsTransformedInFixedJoints;
const int g_outputDecimalPrecision = 16;


/// \brief parser xml string into urdf::Vector3
/// \param[in] _key XML key where vector3 value might be
/// \param[in] _scale scalar scale for the vector3
/// \return a urdf::Vector3
urdf::Vector3 ParseVector3(tinyxml2::XMLNode *_key, double _scale = 1.0);
urdf::Vector3 ParseVector3(const std::string &_str, double _scale = 1.0);

/// insert extensions into collision geoms
void InsertSDFExtensionCollision(tinyxml2::XMLElement *_elem,
                                 const std::string &_linkName);

/// insert extensions into model
void InsertSDFExtensionRobot(tinyxml2::XMLElement *_elem);

/// insert extensions into visuals
void InsertSDFExtensionVisual(tinyxml2::XMLElement *_elem,
                              const std::string &_linkName);


/// insert extensions into joints
void InsertSDFExtensionJoint(tinyxml2::XMLElement *_elem,
                             const std::string &_jointName);

/// reduced fixed joints:  check if a fixed joint should be lumped
///   checking both the joint type and if disabledFixedJointLumping
///   option is set
bool FixedJointShouldBeReduced(urdf::JointSharedPtr _jnt);

/// reduced fixed joints:  apply transform reduction for ray sensors
///   in extensions when doing fixed joint reduction
void ReduceSDFExtensionSensorTransformReduction(
      std::vector<XMLDocumentPtr>::iterator _blobIt,
      ignition::math::Pose3d _reductionTransform);

/// reduced fixed joints:  apply transform reduction for projectors in
///   extensions when doing fixed joint reduction
void ReduceSDFExtensionProjectorTransformReduction(
      std::vector<XMLDocumentPtr>::iterator _blobIt,
      ignition::math::Pose3d _reductionTransform);


/// reduced fixed joints:  apply transform reduction to extensions
///   when doing fixed joint reduction
void ReduceSDFExtensionsTransform(SDFExtensionPtr _ge);

/// reduce fixed joints:  lump joints to parent link
void ReduceJointsToParent(urdf::LinkSharedPtr _link);

/// reduce fixed joints:  lump collisions to parent link
void ReduceCollisionsToParent(urdf::LinkSharedPtr _link);

/// reduce fixed joints:  lump visuals to parent link
void ReduceVisualsToParent(urdf::LinkSharedPtr _link);

/// reduce fixed joints:  lump inertial to parent link
void ReduceInertialToParent(urdf::LinkSharedPtr /*_link*/);

/// create SDF Collision block based on URDF
void CreateCollision(tinyxml2::XMLElement* _elem,
                     urdf::LinkConstSharedPtr _link,
                     urdf::CollisionSharedPtr _collision,
                     const std::string &_oldLinkName = std::string(""));

/// create SDF Visual block based on URDF
void CreateVisual(tinyxml2::XMLElement *_elem, urdf::LinkConstSharedPtr _link,
                  urdf::VisualSharedPtr _visual,
                  const std::string &_oldLinkName = std::string(""));

/// create SDF Joint block based on URDF
void CreateJoint(tinyxml2::XMLElement *_root, urdf::LinkConstSharedPtr _link,
                 ignition::math::Pose3d &_currentTransform);

/// insert extensions into links
void InsertSDFExtensionLink(tinyxml2::XMLElement *_elem,
                            const std::string &_linkName);

/// create visual blocks from urdf visuals
void CreateVisuals(tinyxml2::XMLElement* _elem, urdf::LinkConstSharedPtr _link);

/// create collision blocks from urdf collisions
void CreateCollisions(tinyxml2::XMLElement* _elem,
                      urdf::LinkConstSharedPtr _link);

/// create SDF Inertial block based on URDF
void CreateInertial(tinyxml2::XMLElement *_elem,
                    urdf::LinkConstSharedPtr _link);

/// append transform (pose) to the end of the xml element
void AddTransform(tinyxml2::XMLElement *_elem,
    const ignition::math::Pose3d &_transform);

/// create SDF from URDF link
void CreateSDF(tinyxml2::XMLElement *_root, urdf::LinkConstSharedPtr _link,
               const ignition::math::Pose3d &_transform);

/// create SDF Link block based on URDF
void CreateLink(tinyxml2::XMLElement *_root, urdf::LinkConstSharedPtr _link,
                ignition::math::Pose3d &_currentTransform);

/// reduced fixed joints:  apply appropriate frame updates in joint
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionJointFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in gripper
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionGripperFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in projector
/// inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionProjectorFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link);

/// reduced fixed joints:  apply appropriate frame updates in plugins
///   inside urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionPluginFrameReplace(
      std::vector<XMLDocumentPtr>::iterator _blobIt,
      urdf::LinkSharedPtr _link, const std::string &_pluginName,
      const std::string &_elementName,
      ignition::math::Pose3d _reductionTransform);

/// reduced fixed joints:  apply appropriate frame updates in urdf
///   extensions when doing fixed joint reduction
void ReduceSDFExtensionContactSensorFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link);

/// \brief reduced fixed joints:  apply appropriate updates to urdf
///   extensions when doing fixed joint reduction
///
/// Take the link's existing list of gazebo extensions, transfer them
/// into parent link.  Along the way, update local transforms by adding
/// the additional transform to parent.  Also, look through all
/// referenced link names with plugins and update references to current
/// link to the parent link. (ReduceSDFExtensionFrameReplace())
///
/// \param[in] _link pointer to urdf link, its extensions will be reduced
void ReduceSDFExtensionToParent(urdf::LinkSharedPtr _link);

/// reduced fixed joints:  apply appropriate frame updates
///   in urdf extensions when doing fixed joint reduction
void ReduceSDFExtensionFrameReplace(SDFExtensionPtr _ge,
    urdf::LinkSharedPtr _link);

/// get value from <key value="..."/> pair and return it as string
std::string GetKeyValueAsString(tinyxml2::XMLElement* _elem);

/// \brief append key value pair to the end of the xml element
/// \param[in] _elem pointer to xml element
/// \param[in] _key string containing key to add to xml element
/// \param[in] _value string containing value for the key added
void AddKeyValue(tinyxml2::XMLElement *_elem, const std::string &_key,
                 const std::string &_value);

/// \brief convert values to string
/// \param[in] _count number of values in _values array
/// \param[in] _values array of double values
/// \return a string
std::string Values2str(unsigned int _count, const double *_values);

void CreateGeometry(tinyxml2::XMLElement *_elem,
                    urdf::GeometrySharedPtr _geometry);

ignition::math::Pose3d inverseTransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
urdf::Pose TransformToParentFrame(urdf::Pose _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
ignition::math::Pose3d TransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform);

/// reduced fixed joints: transform to parent frame
ignition::math::Pose3d TransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    ignition::math::Pose3d _parentToLinkTransform);

/// reduced fixed joints: utility to copy between urdf::Pose and
///   math::Pose
ignition::math::Pose3d CopyPose(urdf::Pose _pose);

/// reduced fixed joints: utility to copy between urdf::Pose and
///   math::Pose
urdf::Pose CopyPose(ignition::math::Pose3d _pose);

////////////////////////////////////////////////////////////////////////////////
bool URDF2SDF::IsURDF(const std::string &_filename)
{
  tinyxml2::XMLDocument xmlDoc;

  if (tinyxml2::XML_SUCCESS == xmlDoc.LoadFile(_filename.c_str()))
  {
    tinyxml2::XMLPrinter printer;
    xmlDoc.Print(&printer);
    std::string urdfStr = printer.CStr();
    urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDF(urdfStr);
    return robotModel != nullptr;
  }

  return false;
}

/////////////////////////////////////////////////
urdf::Vector3 ParseVector3(const std::string &_str, double _scale)
{
  std::vector<std::string> pieces = sdf::split(_str, " ");
  std::vector<double> vals;

  for (unsigned int i = 0; i < pieces.size(); ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        vals.push_back(_scale * std::stod(pieces[i]));
      }
      catch(std::invalid_argument &)
      {
        sdferr << "xml key [" << _str
               << "][" << i << "] value [" << pieces[i]
               << "] is not a valid double from a 3-tuple\n";
        return urdf::Vector3(0, 0, 0);
      }
    }
  }

  if (vals.size() == 3)
  {
    return urdf::Vector3(vals[0], vals[1], vals[2]);
  }
  else
  {
    return urdf::Vector3(0, 0, 0);
  }
}

/////////////////////////////////////////////////
urdf::Vector3 ParseVector3(tinyxml2::XMLNode *_key, double _scale)
{
  if (_key != nullptr)
  {
    tinyxml2::XMLElement *key = _key->ToElement();
    if (key != nullptr)
    {
      return ParseVector3(GetKeyValueAsString(key), _scale);
    }
    sdferr << "key[" << _key->Value() << "] does not contain a Vector3\n";
  }
  else
  {
    sdferr << "Pointer to XML node _key is nullptr\n";
  }

  return urdf::Vector3(0, 0, 0);
}

/////////////////////////////////////////////////
/// \brief convert Vector3 to string
/// \param[in] _vector a urdf::Vector3
/// \return a string
std::string Vector32Str(const urdf::Vector3 _vector)
{
  std::stringstream ss;
  ss << _vector.x;
  ss << " ";
  ss << _vector.y;
  ss << " ";
  ss << _vector.z;
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Check and add collision to parent link
/// \param[in] _parentLink destination for _collision
/// \param[in] _name urdfdom 0.3+: urdf collision group name with lumped
///            collision info (see ReduceCollisionsToParent).
///            urdfdom 0.2: collision name with lumped
///            collision info (see ReduceCollisionsToParent).
/// \param[in] _collision move this collision to _parentLink
void ReduceCollisionToParent(urdf::LinkSharedPtr _parentLink,
                             const std::string &_name,
                             urdf::CollisionSharedPtr _collision)
{
  // added a check to see if _collision already exist in
  // _parentLink::collision_array if not, add it.
  _collision->name = _name;
  std::vector<urdf::CollisionSharedPtr>::iterator collisionIt =
    find(_parentLink->collision_array.begin(),
         _parentLink->collision_array.end(),
         _collision);
  if (collisionIt != _parentLink->collision_array.end())
  {
    sdfwarn << "attempted to add collision [" << _collision->name
            << "] to link ["
            << _parentLink->name
            << "], but it already exists in collision_array under name ["
            << (*collisionIt)->name << "]\n";
  }
  else
  {
    _parentLink->collision_array.push_back(_collision);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Check and add visual to parent link
/// \param[in] _parentLink destination for _visual
/// \param[in] _name urdfdom 0.3+: urdf visual group name with lumped
///            visual info (see ReduceVisualsToParent).
///            urdfdom 0.2: visual name with lumped
///            visual info (see ReduceVisualsToParent).
/// \param[in] _visual move this visual to _parentLink
void ReduceVisualToParent(urdf::LinkSharedPtr _parentLink,
                          const std::string &_name,
                          urdf::VisualSharedPtr _visual)
{
  // added a check to see if _visual already exist in
  // _parentLink::visual_array if not, add it.
  _visual->name = _name;
  std::vector<urdf::VisualSharedPtr>::iterator visualIt =
    find(_parentLink->visual_array.begin(),
         _parentLink->visual_array.end(),
         _visual);
  if (visualIt != _parentLink->visual_array.end())
  {
    sdfwarn << "attempted to add visual [" << _visual->name
            << "] to link ["
            << _parentLink->name
            << "], but it already exists in visual_array under name ["
            << (*visualIt)->name << "]\n";
  }
  else
  {
    _parentLink->visual_array.push_back(_visual);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// reduce fixed joints by lumping inertial, visual and
// collision elements of the child link into the parent link
void ReduceFixedJoints(tinyxml2::XMLElement *_root, urdf::LinkSharedPtr _link)
{
  // if child is attached to self by fixed _link first go up the tree,
  //   check it's children recursively
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
  {
    if (FixedJointShouldBeReduced(_link->child_links[i]->parent_joint))
    {
      ReduceFixedJoints(_root, _link->child_links[i]);
    }
  }

  // reduce this _link's stuff up the tree to parent but skip first joint
  //   if it's the world
  if (_link->getParent() && _link->getParent()->name != "world" &&
      _link->parent_joint && FixedJointShouldBeReduced(_link->parent_joint) )
  {
    sdfdbg << "Fixed Joint Reduction: extension lumping from ["
           << _link->name << "] to [" << _link->getParent()->name << "]\n";

    // lump sdf extensions to parent, (give them new reference _link names)
    ReduceSDFExtensionToParent(_link);

    // reduce _link elements to parent
    ReduceInertialToParent(_link);
    ReduceVisualsToParent(_link);
    ReduceCollisionsToParent(_link);
    ReduceJointsToParent(_link);
  }

  // continue down the tree for non-fixed joints
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
  {
    if (!FixedJointShouldBeReduced(_link->child_links[i]->parent_joint))
    {
      ReduceFixedJoints(_root, _link->child_links[i]);
    }
  }
}

// ODE dMatrix
typedef double dMatrix3[4*3];
typedef double dVector3[4];
#define _R(i, j) R[(i)*4+(j)]
#define _I(i, j) I[(i)*4+(j)]
#define dRecip(x) ((1.0f/(x)))

struct dMass;
void dMassSetZero(dMass *m);
void dMassSetParameters(dMass *m, double themass,
                        double cgx, double cgy, double cgz,
                        double I11, double I22, double I33,
                        double I12, double I13, double I23);

struct dMass
{
  double mass;
  dVector3 c;
  dMatrix3 I;

  dMass()
  {
    dMassSetZero(this);
  }

  void setZero()
  {
    dMassSetZero(this);
  }

  void setParameters(double themass, double cgx, double cgy, double cgz,
                     double I11, double I22, double I33,
                     double I12, double I13, double I23)
  {
    dMassSetParameters(this,
                       themass,
                       cgx,
                       cgy,
                       cgz,
                       I11,
                       I22,
                       I33,
                       I12,
                       I13,
                       I23);
  }
};

void dSetZero(double *a, int n)
{
  // dAASSERT (a && n >= 0);
  double *acurr = a;
  int ncurr = n;
  while (ncurr > 0)
  {
    *(acurr++) = 0;
    --ncurr;
  }
}

void dMassSetZero(dMass *m)
{
  // dAASSERT (m);
  m->mass = 0.0;
  dSetZero(m->c, sizeof(m->c) / sizeof(m->c[0]));
  dSetZero(m->I, sizeof(m->I) / sizeof(m->I[0]));
}

void dMassSetParameters(dMass *m, double themass,
                        double cgx, double cgy, double cgz,
                        double I11, double I22, double I33,
                        double I12, double I13, double I23)
{
  // dAASSERT (m);
  dMassSetZero(m);
  m->mass = themass;
  m->c[0] = cgx;
  m->c[1] = cgy;
  m->c[2] = cgz;
  m->_I(0, 0) = I11;
  m->_I(1, 1) = I22;
  m->_I(2, 2) = I33;
  m->_I(0, 1) = I12;
  m->_I(0, 2) = I13;
  m->_I(1, 2) = I23;
  m->_I(1, 0) = I12;
  m->_I(2, 0) = I13;
  m->_I(2, 1) = I23;
  // dMassCheck (m);
}

void dRFromEulerAngles(dMatrix3 R, double phi, double theta, double psi)
{
  double sphi, cphi, stheta, ctheta, spsi, cpsi;
  // dAASSERT (R);
  sphi = sin(phi);
  cphi = cos(phi);
  stheta = sin(theta);
  ctheta = cos(theta);
  spsi = sin(psi);
  cpsi = cos(psi);
  _R(0, 0) = cpsi*ctheta;
  _R(0, 1) = spsi*ctheta;
  _R(0, 2) =-stheta;
  _R(0, 3) = 0.0;
  _R(1, 0) = cpsi*stheta*sphi - spsi*cphi;
  _R(1, 1) = spsi*stheta*sphi + cpsi*cphi;
  _R(1, 2) = ctheta*sphi;
  _R(1, 3) = 0.0;
  _R(2, 0) = cpsi*stheta*cphi + spsi*sphi;
  _R(2, 1) = spsi*stheta*cphi - cpsi*sphi;
  _R(2, 2) = ctheta*cphi;
  _R(2, 3) = 0.0;
}

double _dCalcVectorDot3(const double *a, const double *b, unsigned step_a,
                        unsigned step_b)
{
  return a[0] * b[0] + a[step_a] * b[step_b] + a[2 * step_a] * b[2 * step_b];
}

double dCalcVectorDot3(const double *a, const double *b)
{
  return _dCalcVectorDot3(a, b, 1, 1);
}

double dCalcVectorDot3_41(const double *a, const double *b)
{
  return _dCalcVectorDot3(a, b, 4, 1);
}

void dMultiply0_331(double *res, const double *a, const double *b)
{
  double res_0, res_1, res_2;
  res_0 = dCalcVectorDot3(a, b);
  res_1 = dCalcVectorDot3(a + 4, b);
  res_2 = dCalcVectorDot3(a + 8, b);
  res[0] = res_0;
  res[1] = res_1;
  res[2] = res_2;
}

void dMultiply1_331(double *res, const double *a, const double *b)
{
  double res_0, res_1, res_2;
  res_0 = dCalcVectorDot3_41(a, b);
  res_1 = dCalcVectorDot3_41(a + 1, b);
  res_2 = dCalcVectorDot3_41(a + 2, b);
  res[0] = res_0;
  res[1] = res_1;
  res[2] = res_2;
}

void dMultiply0_133(double *res, const double *a, const double *b)
{
  dMultiply1_331(res, b, a);
}

void dMultiply0_333(double *res, const double *a, const double *b)
{
  dMultiply0_133(res + 0, a + 0, b);
  dMultiply0_133(res + 4, a + 4, b);
  dMultiply0_133(res + 8, a + 8, b);
}

void dMultiply2_333(double *res, const double *a, const double *b)
{
  dMultiply0_331(res + 0, b, a + 0);
  dMultiply0_331(res + 4, b, a + 4);
  dMultiply0_331(res + 8, b, a + 8);
}

void dMassRotate(dMass *m, const dMatrix3 R)
{
  // if the body is rotated by `R' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   R * I * R'
  //
  // where I is the old inertia.

  dMatrix3 t1;
  double t2[3];

  // dAASSERT (m);

  // rotate inertia matrix
  dMultiply2_333(t1, m->I, R);
  dMultiply0_333(m->I, R, t1);

  // ensure perfect symmetry
  m->_I(1, 0) = m->_I(0, 1);
  m->_I(2, 0) = m->_I(0, 2);
  m->_I(2, 1) = m->_I(1, 2);

  // rotate center of mass
  dMultiply0_331(t2, R, m->c);
  m->c[0] = t2[0];
  m->c[1] = t2[1];
  m->c[2] = t2[2];
}

void dSetCrossMatrixPlus(double *res, const double *a, unsigned skip)
{
  const double a_0 = a[0], a_1 = a[1], a_2 = a[2];
  res[1] = -a_2;
  res[2] = +a_1;
  res[skip+0] = +a_2;
  res[skip+2] = -a_0;
  res[2*skip+0] = -a_1;
  res[2*skip+1] = +a_0;
}

void dMassTranslate(dMass *m, double x, double y, double z)
{
  // if the body is translated by `a' relative to its point of reference,
  // the new inertia about the point of reference is:
  //
  //   I + mass*(crossmat(c)^2 - crossmat(c+a)^2)
  //
  // where c is the existing center of mass and I is the old inertia.

  int i, j;
  dMatrix3 ahat, chat, t1, t2;
  double a[3];

  // dAASSERT (m);

  // adjust inertia matrix
  dSetZero(chat, 12);
  dSetCrossMatrixPlus(chat, m->c, 4);
  a[0] = x + m->c[0];
  a[1] = y + m->c[1];
  a[2] = z + m->c[2];
  dSetZero(ahat, 12);
  dSetCrossMatrixPlus(ahat, a, 4);
  dMultiply0_333(t1, ahat, ahat);
  dMultiply0_333(t2, chat, chat);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      m->_I(i, j) += m->mass * (t2[i*4+j]-t1[i*4+j]);
    }
  }

  // ensure perfect symmetry
  m->_I(1, 0) = m->_I(0, 1);
  m->_I(2, 0) = m->_I(0, 2);
  m->_I(2, 1) = m->_I(1, 2);

  // adjust center of mass
  m->c[0] += x;
  m->c[1] += y;
  m->c[2] += z;
}

void dMassAdd(dMass *a, const dMass *b)
{
  int i;
  double denom = dRecip(a->mass + b->mass);
  for (i = 0; i < 3; i++)
  {
    a->c[i] = (a->c[i]*a->mass + b->c[i]*b->mass)*denom;
  }
  a->mass += b->mass;
  for (i = 0; i < 12; i++)
  {
    a->I[i] += b->I[i];
  }
}

/////////////////////////////////////////////////
/// print mass for link for debugging
void PrintMass(const std::string &_linkName, const dMass &_mass)
{
  sdfdbg << "LINK NAME: [" << _linkName << "] from dMass\n";
  sdfdbg << "     MASS: [" << _mass.mass << "]\n";
  sdfdbg << "       CG: [" << _mass.c[0] << ", " << _mass.c[1] << ", "
         << _mass.c[2] << "]\n";
  sdfdbg << "        I: [" << _mass.I[0] << ", " << _mass.I[1] << ", "
         << _mass.I[2] << "]\n";
  sdfdbg << "           [" << _mass.I[4] << ", " << _mass.I[5] << ", "
         << _mass.I[6] << "]\n";
  sdfdbg << "           [" << _mass.I[8] << ", " << _mass.I[9] << ", "
         << _mass.I[10] << "]\n";
}

/////////////////////////////////////////////////
/// print mass for link for debugging
void PrintMass(const urdf::LinkSharedPtr _link)
{
  sdfdbg << "LINK NAME: [" << _link->name << "] from dMass\n";
  sdfdbg << "     MASS: [" << _link->inertial->mass << "]\n";
  sdfdbg << "       CG: [" << _link->inertial->origin.position.x << ", "
         << _link->inertial->origin.position.y << ", "
         << _link->inertial->origin.position.z << "]\n";
  sdfdbg << "        I: [" << _link->inertial->ixx << ", "
         << _link->inertial->ixy << ", "
         << _link->inertial->ixz << "]\n";
  sdfdbg << "           [" << _link->inertial->ixy << ", "
         << _link->inertial->iyy << ", "
         << _link->inertial->iyz << "]\n";
  sdfdbg << "           [" << _link->inertial->ixz << ", "
         << _link->inertial->iyz << ", "
         << _link->inertial->izz << "]\n";
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump inertial to parent link
void ReduceInertialToParent(urdf::LinkSharedPtr _link)
{
  // now lump all contents of this _link to parent
  if (_link->inertial)
  {
    dMatrix3 R;
    double phi, theta, psi;

    // get parent mass (in parent link cg frame)
    dMass parentMass;

    if (!_link->getParent()->inertial)
    {
      _link->getParent()->inertial.reset(new urdf::Inertial);
    }

    dMassSetParameters(&parentMass, _link->getParent()->inertial->mass,
                       0, 0, 0,
                       _link->getParent()->inertial->ixx,
                       _link->getParent()->inertial->iyy,
                       _link->getParent()->inertial->izz,
                       _link->getParent()->inertial->ixy,
                       _link->getParent()->inertial->ixz,
                       _link->getParent()->inertial->iyz);

    // transform parent inertia to parent link origin
    _link->getParent()->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&parentMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&parentMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&parentMass, R);

    // un-translate link mass from cg(inertial frame) into link frame
    dMassTranslate(&parentMass,
                   _link->getParent()->inertial->origin.position.x,
                   _link->getParent()->inertial->origin.position.y,
                   _link->getParent()->inertial->origin.position.z);

    PrintMass("parent: " + _link->getParent()->name, parentMass);
    // PrintMass(_link->getParent());

    //////////////////////////////////////////////
    //                                          //
    // create a_link mass (in _link's cg frame) //
    //                                          //
    //////////////////////////////////////////////
    dMass linkMass;
    dMassSetParameters(&linkMass, _link->inertial->mass,
                       0, 0, 0,
                       _link->inertial->ixx,
                       _link->inertial->iyy,
                       _link->inertial->izz,
                       _link->inertial->ixy,
                       _link->inertial->ixz,
                       _link->inertial->iyz);

    PrintMass("link : " + _link->name, linkMass);

    ////////////////////////////////////////////
    //                                        //
    // from cg (inertial frame) to link frame //
    //                                        //
    ////////////////////////////////////////////

    // Un-rotate _link mass from cg(inertial frame) into link frame
    _link->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&linkMass, R);

    // un-translate link mass from cg(inertial frame) into link frame
    dMassTranslate(&linkMass,
                   _link->inertial->origin.position.x,
                   _link->inertial->origin.position.y,
                   _link->inertial->origin.position.z);

    ////////////////////////////////////////////
    //                                        //
    // from link frame to parent link frame   //
    //                                        //
    ////////////////////////////////////////////

    // un-rotate _link mass into parent link frame
    _link->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(
        phi, theta, psi);
    dRFromEulerAngles(R, -phi,      0,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0, -theta,    0);
    dMassRotate(&linkMass, R);
    dRFromEulerAngles(R,    0,      0, -psi);
    dMassRotate(&linkMass, R);

    // un-translate _link mass into parent link frame
    dMassTranslate(&linkMass,
        _link->parent_joint->parent_to_joint_origin_transform.position.x,
        _link->parent_joint->parent_to_joint_origin_transform.position.y,
        _link->parent_joint->parent_to_joint_origin_transform.position.z);

    PrintMass("link in parent link: " + _link->name, linkMass);

    //
    // now linkMass is in the parent frame, add linkMass to parentMass
    // new parentMass should be combined inertia,
    // centered at parent link inertial frame.
    //

    dMassAdd(&parentMass, &linkMass);

    PrintMass("combined: " + _link->getParent()->name, parentMass);

    //
    // Set new combined inertia in parent link frame into parent link urdf
    //

    // save combined mass
    _link->getParent()->inertial->mass = parentMass.mass;

    // save CoG location
    _link->getParent()->inertial->origin.position.x  = parentMass.c[0];
    _link->getParent()->inertial->origin.position.y  = parentMass.c[1];
    _link->getParent()->inertial->origin.position.z  = parentMass.c[2];

    // get MOI at new CoG location
    dMassTranslate(&parentMass,
                   -_link->getParent()->inertial->origin.position.x,
                   -_link->getParent()->inertial->origin.position.y,
                   -_link->getParent()->inertial->origin.position.z);

    // rotate MOI at new CoG location
    _link->getParent()->inertial->origin.rotation.getRPY(phi, theta, psi);
    dRFromEulerAngles(R, phi, theta, psi);
    dMassRotate(&parentMass, R);

    // save new combined MOI
    _link->getParent()->inertial->ixx  = parentMass.I[0+4*0];
    _link->getParent()->inertial->iyy  = parentMass.I[1+4*1];
    _link->getParent()->inertial->izz  = parentMass.I[2+4*2];
    _link->getParent()->inertial->ixy  = parentMass.I[0+4*1];
    _link->getParent()->inertial->ixz  = parentMass.I[0+4*2];
    _link->getParent()->inertial->iyz  = parentMass.I[1+4*2];

    // final urdf inertia check
    PrintMass(_link->getParent());
  }
}

/////////////////////////////////////////////////
/// \brief reduce fixed joints:  lump visuals to parent link
/// \param[in] _link take all visuals from _link and lump/move them
///            to the parent link (_link->getParentLink()).
void ReduceVisualsToParent(urdf::LinkSharedPtr _link)
{
  // lump all visuals of _link to _link->getParent().
  // modify visual name (urdf 0.3.x) or
  //        visual group name (urdf 0.2.x)
  // to indicate that it was lumped (fixed joint reduced)
  // from another descendant link connected by a fixed joint.
  //
  // Algorithm for generating new name (or group name) is:
  //   original name + g_lumpPrefix+original link name (urdf 0.3.x)
  //   original group name + g_lumpPrefix+original link name (urdf 0.2.x)
  // The purpose is to track where this visual came from
  // (original parent link name before lumping/reducing).
  for (std::vector<urdf::VisualSharedPtr>::iterator
      visualIt = _link->visual_array.begin();
      visualIt != _link->visual_array.end(); ++visualIt)
  {
    // 20151116: changelog for pull request #235
    std::string newVisualName;
    std::size_t lumpIndex = (*visualIt)->name.find(g_lumpPrefix);
    if (lumpIndex != std::string::npos)
    {
      newVisualName = (*visualIt)->name;
      sdfdbg << "re-lumping visual [" << (*visualIt)->name
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with name [" << newVisualName << "]\n";
    }
    else
    {
      if ((*visualIt)->name.empty())
      {
        newVisualName = _link->name;
      }
      else
      {
        newVisualName = (*visualIt)->name;
      }
      sdfdbg << "lumping visual [" << (*visualIt)->name
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with name [" << newVisualName << "]\n";
    }

    // transform visual origin from _link frame to
    // parent link frame before adding to parent
    (*visualIt)->origin = TransformToParentFrame(
        (*visualIt)->origin,
        _link->parent_joint->parent_to_joint_origin_transform);

    // add the modified visual to parent
    ReduceVisualToParent(_link->getParent(), newVisualName,
                         *visualIt);
  }
}

/////////////////////////////////////////////////
/// \brief reduce fixed joints:  lump collisions to parent link
/// \param[in] _link take all collisions from _link and lump/move them
///            to the parent link (_link->getParentLink()).
void ReduceCollisionsToParent(urdf::LinkSharedPtr _link)
{
  // lump all collisions of _link to _link->getParent().
  // modify collision name (urdf 0.3.x) or
  //        collision group name (urdf 0.2.x)
  // to indicate that it was lumped (fixed joint reduced)
  // from another descendant link connected by a fixed joint.
  //
  // Algorithm for generating new name (or group name) is:
  //   original name + g_lumpPrefix+original link name (urdf 0.3.x)
  //   original group name + g_lumpPrefix+original link name (urdf 0.2.x)
  // The purpose is to track where this collision came from
  // (original parent link name before lumping/reducing).
  for (std::vector<urdf::CollisionSharedPtr>::iterator
      collisionIt = _link->collision_array.begin();
      collisionIt != _link->collision_array.end(); ++collisionIt)
  {
    std::string newCollisionName;
    std::size_t lumpIndex = (*collisionIt)->name.find(g_lumpPrefix);
    if (lumpIndex != std::string::npos)
    {
      newCollisionName = (*collisionIt)->name;
      sdfdbg << "re-lumping collision [" << (*collisionIt)->name
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with name [" << newCollisionName << "]\n";
    }
    else
    {
      if ((*collisionIt)->name.empty())
      {
        newCollisionName = _link->name;
      }
      else
      {
        newCollisionName = (*collisionIt)->name;
      }
      sdfdbg << "lumping collision [" << (*collisionIt)->name
             << "] for link [" << _link->name
             << "] to parent [" << _link->getParent()->name
             << "] with name [" << newCollisionName << "]\n";
    }
    // transform collision origin from _link frame to
    // parent link frame before adding to parent
    (*collisionIt)->origin = TransformToParentFrame(
        (*collisionIt)->origin,
        _link->parent_joint->parent_to_joint_origin_transform);

    // add the modified collision to parent
    ReduceCollisionToParent(_link->getParent(), newCollisionName,
                            *collisionIt);
  }
}

/////////////////////////////////////////////////
/// reduce fixed joints:  lump joints to parent link
void ReduceJointsToParent(urdf::LinkSharedPtr _link)
{
  // set child link's parentJoint's parent link to
  // a parent link up stream that does not have a fixed parentJoint
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
  {
    urdf::JointSharedPtr parentJoint = _link->child_links[i]->parent_joint;
    if (!FixedJointShouldBeReduced(parentJoint))
    {
      // go down the tree until we hit a parent joint that is not fixed
      urdf::LinkSharedPtr newParentLink = _link;
      ignition::math::Pose3d jointAnchorTransform;
      while (newParentLink->parent_joint &&
             newParentLink->getParent()->name != "world" &&
             FixedJointShouldBeReduced(newParentLink->parent_joint) )
      {
        jointAnchorTransform = jointAnchorTransform * jointAnchorTransform;
        parentJoint->parent_to_joint_origin_transform =
          TransformToParentFrame(
              parentJoint->parent_to_joint_origin_transform,
              newParentLink->parent_joint->parent_to_joint_origin_transform);
        newParentLink = newParentLink->getParent();
      }
      // now set the _link->child_links[i]->parent_joint's parent link to
      // the newParentLink
      _link->child_links[i]->setParent(newParentLink);
      parentJoint->parent_link_name = newParentLink->name;
      // and set the _link->child_links[i]->parent_joint's
      // parent_to_joint_origin_transform as the aggregated anchor transform?
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
std::string lowerStr(const std::string &_str)
{
  std::string out = _str;
  std::transform(out.begin(), out.end(), out.begin(),
      [](unsigned char c)
      {
        return static_cast<unsigned char>(std::tolower(c));
      });
  return out;
}

////////////////////////////////////////////////////////////////////////////////
URDF2SDF::URDF2SDF()
{
  // default options
  g_enforceLimits = true;
  g_reduceFixedJoints = true;
  g_extensions.clear();
  g_collisionExt = "_collision";
  g_visualExt = "_visual";
  g_lumpPrefix = "_fixed_joint_lump__";
  g_initialRobotPoseValid = false;
  g_fixedJointsTransformedInRevoluteJoints.clear();
  g_fixedJointsTransformedInFixedJoints.clear();
}

////////////////////////////////////////////////////////////////////////////////
URDF2SDF::~URDF2SDF()
{
}



////////////////////////////////////////////////////////////////////////////////
std::string Values2str(unsigned int _count, const double *_values)
{
  std::stringstream ss;
  ss.precision(g_outputDecimalPrecision);
  for (unsigned int i = 0 ; i < _count ; ++i)
  {
    if (i > 0)
    {
      ss << " ";
    }
    if (std::fpclassify(_values[i]) == FP_ZERO)
      ss << 0;
    else
      ss << _values[i];
  }
  return ss.str();
}

/////////////////////////////////////////////////
std::string Values2str(unsigned int _count, const int *_values)
{
  std::stringstream ss;
  for (unsigned int i = 0 ; i < _count ; ++i)
  {
    if (i > 0)
    {
      ss << " ";
    }
    ss << _values[i];
  }
  return ss.str();
}

////////////////////////////////////////////////////////////////////////////////
void AddKeyValue(tinyxml2::XMLElement *_elem, const std::string &_key,
                 const std::string &_value)
{
  tinyxml2::XMLElement *childElem = _elem->FirstChildElement(_key.c_str());
  if (childElem)
  {
    std::string oldValue = GetKeyValueAsString(childElem);
    if (oldValue != _value)
    {
      sdfwarn << "multiple inconsistent <" << _key
              << "> exists due to fixed joint reduction"
              << " overwriting previous value [" << oldValue
              << "] with [" << _value << "].\n";
    }
    else
    {
       sdfdbg << "multiple consistent <" << _key
              << "> exists with [" << _value
              << "] due to fixed joint reduction.\n";
    }
    _elem->DeleteChild(childElem);  // remove old _elem
  }

  auto *doc = _elem->GetDocument();
  tinyxml2::XMLElement *ekey = doc->NewElement(_key.c_str());
  tinyxml2::XMLText *textEkey = doc->NewText(_value.c_str());
  ekey->LinkEndChild(textEkey);
  _elem->LinkEndChild(ekey);
}

////////////////////////////////////////////////////////////////////////////////
void AddTransform(tinyxml2::XMLElement *_elem,
                  const ignition::math::Pose3d &_transform)
{
  ignition::math::Vector3d e = _transform.Rot().Euler();
  double cpose[6] = { _transform.Pos().X(), _transform.Pos().Y(),
                      _transform.Pos().Z(), e.X(), e.Y(), e.Z() };

  // set geometry transform
  AddKeyValue(_elem, "pose", Values2str(6, cpose));
}

////////////////////////////////////////////////////////////////////////////////
std::string GetKeyValueAsString(tinyxml2::XMLElement* _elem)
{
  std::string valueStr;
  if (_elem->Attribute("value"))
  {
    valueStr = _elem->Attribute("value");
  }
  else if (_elem->FirstChild())
  {
    // Check that this node is a XMLText
    if (_elem->FirstChild()->ToText())
    {
      valueStr = _elem->FirstChild()->Value();
    }
    else
    {
      sdfwarn << "Attribute value string not set\n";
    }
  }
  return trim(valueStr);
}

/////////////////////////////////////////////////
void ParseRobotOrigin(tinyxml2::XMLDocument &_urdfXml)
{
  tinyxml2::XMLElement *robotXml = _urdfXml.FirstChildElement("robot");
  tinyxml2::XMLElement *originXml = robotXml->FirstChildElement("origin");
  if (originXml)
  {
    const char *xyzstr = originXml->Attribute("xyz");
    if (xyzstr == nullptr)
    {
      g_initialRobotPose.position = urdf::Vector3(0, 0, 0);
    }
    else
    {
      g_initialRobotPose.position = ParseVector3(std::string(xyzstr));
    }
    const char *rpystr = originXml->Attribute("rpy");
    urdf::Vector3 rpy;
    if (rpystr == nullptr)
    {
      rpy = urdf::Vector3(0, 0, 0);
    }
    else
    {
      rpy = ParseVector3(std::string(rpystr));
    }
    g_initialRobotPose.rotation.setFromRPY(rpy.x, rpy.y, rpy.z);
    g_initialRobotPoseValid = true;
  }
}

/////////////////////////////////////////////////
void InsertRobotOrigin(tinyxml2::XMLElement *_elem)
{
  if (g_initialRobotPoseValid)
  {
    // set transform
    double pose[6];
    pose[0] = g_initialRobotPose.position.x;
    pose[1] = g_initialRobotPose.position.y;
    pose[2] = g_initialRobotPose.position.z;
    g_initialRobotPose.rotation.getRPY(pose[3], pose[4], pose[5]);
    AddKeyValue(_elem, "pose", Values2str(6, pose));
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ParseSDFExtension(tinyxml2::XMLDocument &_urdfXml)
{
  tinyxml2::XMLElement* robotXml = _urdfXml.FirstChildElement("robot");

  // Get all SDF extension elements, put everything in
  //   g_extensions map, containing a key string
  //   (link/joint name) and values
  for (tinyxml2::XMLElement* sdfXml = robotXml->FirstChildElement("gazebo");
       sdfXml; sdfXml = sdfXml->NextSiblingElement("gazebo"))
  {
    const char* ref = sdfXml->Attribute("reference");
    std::string refStr;
    if (!ref)
    {
      // copy extensions for robot (outside of link/joint)
      refStr.clear();
    }
    else
    {
      // copy extensions for link/joint
      refStr = std::string(ref);
    }

    if (g_extensions.find(refStr) == g_extensions.end())
    {
      // create extension map for reference
      std::vector<SDFExtensionPtr> ge;
      g_extensions.insert(std::make_pair(refStr, ge));
    }

    // create and insert a new SDFExtension into the map
    SDFExtensionPtr sdf(new SDFExtension());

    // begin parsing xml node
    for (tinyxml2::XMLElement *childElem = sdfXml->FirstChildElement();
         childElem; childElem = childElem->NextSiblingElement())
    {
      sdf->oldLinkName = refStr;

      // go through all elements of the extension,
      //   extract what we know, and save the rest in blobs
      // @todo:  somehow use sdf definitions here instead of hard coded
      //         objects

      // material
      if (strcmp(childElem->Name(), "material") == 0)
      {
        sdf->material = GetKeyValueAsString(childElem);
      }
      else if (strcmp(childElem->Name(), "collision") == 0
            || strcmp(childElem->Name(), "visual") == 0)
      {
        // anything inside of collision or visual tags:
        // <gazebo reference="link_name">
        //   <collision>
        //     <collision_extention_stuff_here/>
        //   </collision>
        //   <visual>
        //     <visual_extention_stuff_here/>
        //   </visual>
        // </gazebl>
        // are treated as blobs that gets inserted
        // into all collisions and visuals for the link
        // <collision name="link_name[anything here]">
        //   <stuff_from_urdf_link_collisions/>
        //   <collision_extention_stuff_here/>
        // </collision>
        // <visual name="link_name[anything here]">
        //   <stuff_from_urdf_link_visuals/>
        //   <visual_extention_stuff_here/>
        // </visual>

        // a place to store converted doc
        for (tinyxml2::XMLElement* e = childElem->FirstChildElement(); e;
             e = e->NextSiblingElement())
        {
          tinyxml2::XMLPrinter printer;
          e->Accept(&printer);

          XMLDocumentPtr xmlDocBlob(new tinyxml2::XMLDocument);
          xmlDocBlob->Parse(printer.CStr());

          // save all unknown stuff in a vector of blobs
          if (strcmp(childElem->Name(), "collision") == 0)
          {
            sdf->collision_blobs.push_back(xmlDocBlob);
          }
          else
          {
            sdf->visual_blobs.push_back(xmlDocBlob);
          }
        }
      }
      else if (strcmp(childElem->Name(), "static") == 0)
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of setting static flag is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          sdf->setStaticFlag = true;
        }
        else
        {
          sdf->setStaticFlag = false;
        }
      }
      else if (strcmp(childElem->Name(), "turnGravityOff") == 0)
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of gravity is true
        if (lowerStr(valueStr) == "false" || lowerStr(valueStr) == "no" ||
            valueStr == "0")
        {
          sdf->gravity = true;
        }
        else
        {
          sdf->gravity = false;
        }
      }
      else if (strcmp(childElem->Name(), "dampingFactor") == 0)
      {
        sdf->isDampingFactor = true;
        sdf->dampingFactor = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "maxVel") == 0)
      {
        sdf->isMaxVel = true;
        sdf->maxVel = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "minDepth") == 0)
      {
        sdf->isMinDepth = true;
        sdf->minDepth = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "mu1") == 0)
      {
        sdf->isMu1 = true;
        sdf->mu1 = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "mu2") == 0)
      {
        sdf->isMu2 = true;
        sdf->mu2 = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "fdir1") == 0)
      {
        sdf->fdir1 = GetKeyValueAsString(childElem);
      }
      else if (strcmp(childElem->Name(), "kp") == 0)
      {
        sdf->isKp = true;
        sdf->kp = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "kd") == 0)
      {
        sdf->isKd = true;
        sdf->kd = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "selfCollide") == 0)
      {
        sdf->isSelfCollide = true;
        std::string valueStr = GetKeyValueAsString(childElem);

        // default of selfCollide is false
        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          sdf->selfCollide = true;
        }
        else
        {
          sdf->selfCollide = false;
        }
      }
      else if (strcmp(childElem->Name(), "maxContacts") == 0)
      {
        sdf->isMaxContacts = true;
        sdf->maxContacts = std::stoi(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "laserRetro") == 0)
      {
        sdf->isLaserRetro = true;
        sdf->laserRetro = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "springReference") == 0)
      {
        sdf->isSpringReference = true;
        sdf->springReference = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "springStiffness") == 0)
      {
        sdf->isSpringStiffness = true;
        sdf->springStiffness = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "stopCfm") == 0)
      {
        sdf->isStopCfm = true;
        sdf->stopCfm = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "stopErp") == 0)
      {
        sdf->isStopErp = true;
        sdf->stopErp = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "fudgeFactor") == 0)
      {
        sdf->isFudgeFactor = true;
        sdf->fudgeFactor = std::stod(GetKeyValueAsString(childElem));
      }
      else if (strcmp(childElem->Name(), "provideFeedback") == 0)
      {
        sdf->isProvideFeedback = true;
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          sdf->provideFeedback = true;
        }
        else
        {
          sdf->provideFeedback = false;
        }
      }
      else if (strcmp(childElem->Name(), "canonicalBody") == 0)
      {
        sdfdbg << "do nothing with canonicalBody\n";
      }
      else if (strcmp(childElem->Name(), "cfmDamping") == 0 ||
                 strcmp(childElem->Name(), "implicitSpringDamper") == 0)
      {
        if (strcmp(childElem->Name(), "cfmDamping") == 0)
        {
          sdfwarn << "Note that cfmDamping is being deprecated by "
                  << "implicitSpringDamper, please replace instances "
                  << "of cfmDamping with implicitSpringDamper in your model.\n";
        }

        sdf->isImplicitSpringDamper = true;
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          sdf->implicitSpringDamper = true;
        }
        else
        {
          sdf->implicitSpringDamper = false;
        }
      }
      else if (strcmp(childElem->Name(), "disableFixedJointLumping") == 0)
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          g_fixedJointsTransformedInRevoluteJoints.insert(refStr);
        }
      }
      else if (strcmp(childElem->Name(), "preserveFixedJoint") == 0)
      {
        std::string valueStr = GetKeyValueAsString(childElem);

        if (lowerStr(valueStr) == "true" || lowerStr(valueStr) == "yes" ||
            valueStr == "1")
        {
          g_fixedJointsTransformedInFixedJoints.insert(refStr);
        }
      }
      else
      {
        // a place to store converted doc
        XMLDocumentPtr xmlNewDoc(new tinyxml2::XMLDocument);
        tinyxml2::XMLPrinter printer;
        childElem->Accept(&printer);
        xmlNewDoc->Parse(printer.CStr());

        sdfdbg << "extension [" << printer.CStr() <<
          "] not converted from URDF, probably already in SDF format.\n";

        // save all unknown stuff in a vector of blobs
        sdf->blobs.push_back(xmlNewDoc);
      }
    }

    // insert into my map
    (g_extensions.find(refStr))->second.push_back(sdf);
  }

  // Handle fixed joints for which both disableFixedJointLumping
  // and preserveFixedJoint options are present
  for (auto& fixedJointConvertedToFixed:
             g_fixedJointsTransformedInFixedJoints)
  {
    // If both options are present, the model creator is aware of the
    // existence of the preserveFixedJoint option and the
    // disableFixedJointLumping option is there only for backward compatibility
    // For this reason, if both options are present then the preserveFixedJoint
    // option has the precedence
    g_fixedJointsTransformedInRevoluteJoints.erase(fixedJointConvertedToFixed);
  }
}

void CopyBlob(tinyxml2::XMLElement *_src, tinyxml2::XMLElement *_blob_parent)
{
  if (_blob_parent == nullptr)
  {
    sdferr << "blob parent is null\n";
    return;
  }

  tinyxml2::XMLNode *clone = DeepClone(_blob_parent->GetDocument(), _src);
  if (clone == nullptr)
  {
    sdferr << "Unable to deep copy blob\n";
  }
  else
  {
    _blob_parent->LinkEndChild(clone);
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionCollision(tinyxml2::XMLElement *_elem,
                                 const std::string &_linkName)
{
  // loop through extensions for the whole model
  // and see which ones belong to _linkName
  // This might be complicated since there's:
  //   - urdf collision name -> sdf collision name conversion
  //   - fixed joint reduction / lumping
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _linkName)
    {
      // std::cerr << "============================\n";
      // std::cerr << "working on g_extensions for link ["
      //           << sdfIt->first << "]\n";
      // if _elem already has a surface element, use it
      tinyxml2::XMLNode *surface = _elem->FirstChildElement("surface");
      tinyxml2::XMLNode *friction = nullptr;
      tinyxml2::XMLNode *frictionOde = nullptr;
      tinyxml2::XMLNode *contact = nullptr;
      tinyxml2::XMLNode *contactOde = nullptr;

      // loop through all the gazebo extensions stored in sdfIt->second
      for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
           ge != sdfIt->second.end(); ++ge)
      {
        // Check if this blob belongs to _elem based on
        //   - blob's reference link name (_linkName or sdfIt->first)
        //   - _elem (destination for blob, which is a collision sdf).

        if (!_elem->Attribute("name"))
        {
          sdferr << "ERROR: collision _elem has no name,"
                 << " something is wrong" << "\n";
        }

        std::string sdfCollisionName(_elem->Attribute("name"));

        // std::cerr << "----------------------------\n";
        // std::cerr << "blob belongs to [" << _linkName
        //           << "] with old parent LinkName [" << (*ge)->oldLinkName
        //           << "]\n";
        // std::cerr << "_elem sdf collision name [" << sdfCollisionName
        //           << "]\n";
        // std::cerr << "----------------------------\n";

        std::string lumpCollisionName = g_lumpPrefix +
          (*ge)->oldLinkName + g_collisionExt;

        bool wasReduced = (_linkName == (*ge)->oldLinkName);
        bool collisionNameContainsLinkname =
          sdfCollisionName.find(_linkName) != std::string::npos;
        bool collisionNameContainsLumpedLinkname =
          sdfCollisionName.find(lumpCollisionName) != std::string::npos;
        bool collisionNameContainsLumpedRef =
          sdfCollisionName.find(g_lumpPrefix) != std::string::npos;

        if (!collisionNameContainsLinkname)
        {
          sdferr << "collision name does not contain link name,"
                 << " file an issue.\n";
        }

        // if the collision _elem was not reduced,
        // its name should not have g_lumpPrefix in it.
        // otherwise, its name should have
        // "g_lumpPrefix+[original link name before reduction]".
        if ((wasReduced && !collisionNameContainsLumpedRef) ||
            (!wasReduced && collisionNameContainsLumpedLinkname))
        {
          // insert any blobs (including visual plugins)
          // warning, if you insert a <surface> sdf here, it might
          // duplicate what was constructed above.
          // in the future, we should use blobs (below) in place of
          // explicitly specified fields (above).
          if (!(*ge)->collision_blobs.empty())
          {
            for (auto blob = (*ge)->collision_blobs.begin();
                blob != (*ge)->collision_blobs.end(); ++blob)
            {
              // find elements and assign pointers if they exist
              // for mu1, mu2, minDepth, maxVel, fdir1, kp, kd
              // otherwise, they are allocated by 'new' below.
              // std::cerr << ">>>>> working on extension blob: ["
              //           << (*blob)->Value() << "]\n";

              if (strcmp((*blob)->FirstChildElement()->Name(), "surface") == 0)
              {
                // blob is a <surface>, tread carefully otherwise
                // we end up with multiple copies of <surface>.
                // Also, get pointers (contact[Ode], friction[Ode])
                // below for backwards (non-blob) compatibility.
                if (surface == nullptr)
                {
                  // <surface> do not exist, it simple,
                  // just add it to the current collision
                  // and it's done.
                  CopyBlob((*blob)->FirstChildElement(), _elem);
                  surface = _elem->LastChildElement("surface");
                  // std::cerr << " --- surface created "
                  //           <<  (void*)surface << "\n";
                }
                else
                {
                  // <surface> exist already, remove it and
                  // overwrite with the blob.
                  _elem->DeleteChild(surface);
                  CopyBlob((*blob)->FirstChildElement(), _elem);
                  surface = _elem->FirstChildElement("surface");
                  // std::cerr << " --- surface exists, replace with blob.\n";
                }

                // Extra code for backwards compatibility, to
                // deal with old way of specifying collision attributes
                // using individual elements listed below:
                //   "mu"
                //   "mu2"
                //   "fdir1"
                //   "kp"
                //   "kd"
                //   "max_vel"
                //   "min_depth"
                //   "laser_retro"
                //   "max_contacts"
                // Get contact[Ode] and friction[Ode] node pointers
                // if they exist.
                contact  = surface->FirstChildElement("contact");
                if (contact != nullptr)
                {
                  contactOde  = contact->FirstChildElement("ode");
                }
                friction = surface->FirstChildElement("friction");
                if (friction != nullptr)
                {
                  frictionOde  = friction->FirstChildElement("ode");
                }
              }
              else
              {
                // If the blob is not a <surface>, we don't have
                // to worry about backwards compatibility.
                // Simply add to master element.
                CopyBlob((*blob)->FirstChildElement(), _elem);
              }
            }
          }

          // Extra code for backwards compatibility, to
          // deal with old way of specifying collision attributes
          // using individual elements listed below:
          //   "mu"
          //   "mu2"
          //   "fdir1"
          //   "kp"
          //   "kd"
          //   "max_vel"
          //   "min_depth"
          //   "laser_retro"
          //   "max_contacts"
          // The new way to do this is to specify everything
          // in collision blobs by using the <collision> tag.
          // So there's no need for custom code for each property.

          // construct new elements if not in blobs
          auto* doc = _elem->GetDocument();
          if (surface == nullptr)
          {
            surface  = doc->NewElement("surface");
            if (!surface)
            {
              // Memory allocation error
              sdferr << "Memory allocation error while"
                     << " processing <surface>.\n";
            }
            _elem->LinkEndChild(surface);
          }

          // construct new elements if not in blobs
          if (contact == nullptr)
          {
            if (surface->FirstChildElement("contact") == nullptr)
            {
              contact  = doc->NewElement("contact");
              if (!contact)
              {
                // Memory allocation error
                sdferr << "Memory allocation error while"
                       << " processing <contact>.\n";
              }
              surface->LinkEndChild(contact);
            }
            else
            {
              contact  = surface->FirstChildElement("contact");
            }
          }

          if (contactOde == nullptr)
          {
            if (contact->FirstChildElement("ode") == nullptr)
            {
              contactOde  = doc->NewElement("ode");
              if (!contactOde)
              {
                // Memory allocation error
                sdferr << "Memory allocation error while"
                       << " processing <contact><ode>.\n";
              }
              contact->LinkEndChild(contactOde);
            }
            else
            {
              contactOde  = contact->FirstChildElement("ode");
            }
          }

          if (friction == nullptr)
          {
            if (surface->FirstChildElement("friction") == nullptr)
            {
              friction  = doc->NewElement("friction");
              if (!friction)
              {
                // Memory allocation error
                sdferr << "Memory allocation error while"
                       << " processing <friction>.\n";
              }
              surface->LinkEndChild(friction);
            }
            else
            {
              friction  = surface->FirstChildElement("friction");
            }
          }

          if (frictionOde == nullptr)
          {
            if (friction->FirstChildElement("ode") == nullptr)
            {
              frictionOde  = doc->NewElement("ode");
              if (!frictionOde)
              {
                // Memory allocation error
                sdferr << "Memory allocation error while"
                       << " processing <friction><ode>.\n";
              }
              friction->LinkEndChild(frictionOde);
            }
            else
            {
              frictionOde = friction->FirstChildElement("ode");
            }
          }

          // insert mu1, mu2, kp, kd for collision
          if ((*ge)->isMu1)
          {
            AddKeyValue(frictionOde->ToElement(), "mu",
                        Values2str(1, &(*ge)->mu1));
          }
          if ((*ge)->isMu2)
          {
            AddKeyValue(frictionOde->ToElement(), "mu2",
                        Values2str(1, &(*ge)->mu2));
          }
          if (!(*ge)->fdir1.empty())
          {
            AddKeyValue(frictionOde->ToElement(), "fdir1", (*ge)->fdir1);
          }
          if ((*ge)->isKp)
          {
            AddKeyValue(contactOde->ToElement(), "kp",
                        Values2str(1, &(*ge)->kp));
          }
          if ((*ge)->isKd)
          {
            AddKeyValue(contactOde->ToElement(), "kd",
                        Values2str(1, &(*ge)->kd));
          }
          // max contact interpenetration correction velocity
          if ((*ge)->isMaxVel)
          {
            AddKeyValue(contactOde->ToElement(), "max_vel",
                        Values2str(1, &(*ge)->maxVel));
          }
          // contact interpenetration margin tolerance
          if ((*ge)->isMinDepth)
          {
            AddKeyValue(contactOde->ToElement(), "min_depth",
                        Values2str(1, &(*ge)->minDepth));
          }
          if ((*ge)->isLaserRetro)
          {
            AddKeyValue(_elem, "laser_retro",
                        Values2str(1, &(*ge)->laserRetro));
          }
          if ((*ge)->isMaxContacts)
          {
            AddKeyValue(_elem, "max_contacts",
                        Values2str(1, &(*ge)->maxContacts));
          }
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionVisual(tinyxml2::XMLElement *_elem,
                              const std::string &_linkName)
{
  // loop through extensions for the whole model
  // and see which ones belong to _linkName
  // This might be complicated since there's:
  //   - urdf visual name -> sdf visual name conversion
  //   - fixed joint reduction / lumping
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _linkName)
    {
      // std::cerr << "============================\n";
      // std::cerr << "working on g_extensions for link ["
      //           << sdfIt->first << "]\n";
      // if _elem already has a material element, use it
      tinyxml2::XMLElement *material = _elem->FirstChildElement("material");
      tinyxml2::XMLElement *script = nullptr;

      // loop through all the gazebo extensions stored in sdfIt->second
      for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
           ge != sdfIt->second.end(); ++ge)
      {
        // Check if this blob belongs to _elem based on
        //   - blob's reference link name (_linkName or sdfIt->first)
        //   - _elem (destination for blob, which is a visual sdf).

        if (!_elem->Attribute("name"))
        {
          sdferr << "ERROR: visual _elem has no name,"
                 << " something is wrong" << "\n";
        }

        std::string sdfVisualName(_elem->Attribute("name"));

        // std::cerr << "----------------------------\n";
        // std::cerr << "blob belongs to [" << _linkName
        //           << "] with old parent LinkName [" << (*ge)->oldLinkName
        //           << "]\n";
        // std::cerr << "_elem sdf visual name [" << sdfVisualName
        //           << "]\n";
        // std::cerr << "----------------------------\n";

        std::string lumpVisualName = g_lumpPrefix +
          (*ge)->oldLinkName + g_visualExt;

        bool wasReduced = (_linkName == (*ge)->oldLinkName);
        bool visualNameContainsLinkname =
          sdfVisualName.find(_linkName) != std::string::npos;
        bool visualNameContainsLumpedLinkname =
          sdfVisualName.find(lumpVisualName) != std::string::npos;
        bool visualNameContainsLumpedRef =
          sdfVisualName.find(g_lumpPrefix) != std::string::npos;

        if (!visualNameContainsLinkname)
        {
          sdferr << "visual name does not contain link name,"
                 << " file an issue.\n";
        }

        // if the visual _elem was not reduced,
        // its name should not have g_lumpPrefix in it.
        // otherwise, its name should have
        // "g_lumpPrefix+[original link name before reduction]".
        if ((wasReduced && !visualNameContainsLumpedRef) ||
            (!wasReduced && visualNameContainsLumpedLinkname))
        {
          // insert any blobs (including visual plugins)
          // warning, if you insert a <material> sdf here, it might
          // duplicate what was constructed above.
          // in the future, we should use blobs (below) in place of
          // explicitly specified fields (above).
          if (!(*ge)->visual_blobs.empty())
          {
            for (auto blob = (*ge)->visual_blobs.begin();
                blob != (*ge)->visual_blobs.end(); ++blob)
            {
              // find elements and assign pointers if they exist
              // for mu1, mu2, minDepth, maxVel, fdir1, kp, kd
              // otherwise, they are allocated by 'new' below.
              // std::cerr << ">>>>> working on extension blob: ["
              //           << (*blob)->Value() << "]\n";

              // print for debug
              // std::ostringstream origStream;
              // origStream << *(*blob)->Clone();
              // std::cerr << "visual extension ["
              //           << origStream.str() << "]\n";

              if (strcmp((*blob)->FirstChildElement()->Name(), "material") == 0)
              {
                // blob is a <material>, tread carefully otherwise
                // we end up with multiple copies of <material>.
                // Also, get pointers (script)
                // below for backwards (non-blob) compatibility.
                if (material == nullptr)
                {
                  // <material> do not exist, it simple,
                  // just add it to the current visual
                  // and it's done.
                  CopyBlob((*blob)->FirstChildElement(), _elem);
                  material = _elem->LastChildElement("material");
                  // std::cerr << " --- material created "
                  //           <<  (void*)material << "\n";
                }
                else
                {
                  // <material> exist already, remove it and
                  // overwrite with the blob.
                  _elem->DeleteChild(material);
                  CopyBlob((*blob)->FirstChildElement(), _elem);
                  material = _elem->FirstChildElement("material");
                  // std::cerr << " --- material exists, replace with blob.\n";
                }

                // Extra code for backwards compatibility, to
                // deal with old way of specifying visual attributes
                // using individual element:
                //   "script"
                // Get script node pointers
                // if they exist.
                script = material->FirstChildElement("script");
              }
              else
              {
                // std::cerr << "***** working on extension blob: ["
                //           << (*blob)->Value() << "]\n";
                // If the blob is not a <material>, we don't have
                // to worry about backwards compatibility.
                // Simply add to master element.
                CopyBlob((*blob)->FirstChildElement(), _elem);
              }
            }
          }

          // Extra code for backwards compatibility, to
          // deal with old way of specifying visual attributes
          // using individual element:
          //   "script"
          // The new way to do this is to specify everything
          // in visual blobs by using the <visual> tag.
          // So there's no need for custom code for each property.

          // backward compatibility for old code
          // insert material/script block for visual
          // (*ge)->material block goes under sdf <material><script><name>.
          if (!(*ge)->material.empty())
          {
            // construct new elements if not in blobs
            if (material == nullptr)
            {
              material  = _elem->GetDocument()->NewElement("material");
              if (!material)
              {
                // Memory allocation error
                sdferr << "Memory allocation error while"
                       << " processing <material>.\n";
              }
              _elem->LinkEndChild(material);
            }

            if (script == nullptr)
            {
              if (material->FirstChildElement("script") == nullptr)
              {
                script  = _elem->GetDocument()->NewElement("script");
                if (!script)
                {
                  // Memory allocation error
                  sdferr << "Memory allocation error while"
                         << " processing <script>.\n";
                }
                material->LinkEndChild(script);
              }
              else
              {
                script = material->FirstChildElement("script");
              }
            }

            AddKeyValue(script, "name", (*ge)->material);
            // hard code original default gazebo materials files
            AddKeyValue(script, "uri",
              "file://media/materials/scripts/gazebo.material");
          }
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionLink(tinyxml2::XMLElement *_elem,
                            const std::string &_linkName)
{
  for (StringSDFExtensionPtrMap::iterator
       sdfIt = g_extensions.begin();
       sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _linkName)
    {
      sdfdbg << "inserting extension with reference ["
             << _linkName << "] into link.\n";
      for (std::vector<SDFExtensionPtr>::iterator ge =
          sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        // insert gravity
        if ((*ge)->gravity)
        {
          AddKeyValue(_elem, "gravity", "true");
        }
        else
        {
          AddKeyValue(_elem, "gravity", "false");
        }

        // damping factor

        tinyxml2::XMLElement *velocityDecay =
          _elem->GetDocument()->NewElement("velocity_decay");
        if ((*ge)->isDampingFactor)
        {
          /// @todo separate linear and angular velocity decay
          AddKeyValue(velocityDecay, "linear",
                      Values2str(1, &(*ge)->dampingFactor));
          AddKeyValue(velocityDecay, "angular",
                      Values2str(1, &(*ge)->dampingFactor));
        }
        _elem->LinkEndChild(velocityDecay);
        // selfCollide tag
        if ((*ge)->isSelfCollide)
        {
          AddKeyValue(_elem, "self_collide", (*ge)->selfCollide ? "1" : "0");
        }
        // insert blobs into body
        for (auto blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          // Be sure to always copy only the first element; code in
          // ReduceSDFExtensionSensorTransformReduction depends in this behavior
          CopyBlob((*blobIt)->FirstChildElement(), _elem);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionJoint(tinyxml2::XMLElement *_elem,
                             const std::string &_jointName)
{
  auto* doc = _elem->GetDocument();
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _jointName)
    {
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin();
          ge != sdfIt->second.end(); ++ge)
      {
        tinyxml2::XMLElement *physics = _elem->FirstChildElement("physics");
        bool newPhysics = false;
        if (physics == nullptr)
        {
          physics = doc->NewElement("physics");
          newPhysics = true;
        }

        tinyxml2::XMLElement *physicsOde = physics->FirstChildElement("ode");
        bool newPhysicsOde = false;
        if (physicsOde == nullptr)
        {
          physicsOde = doc->NewElement("ode");
          newPhysicsOde = true;
        }

        tinyxml2::XMLElement *limit = physicsOde->FirstChildElement("limit");
        bool newLimit = false;
        if (limit == nullptr)
        {
          limit = doc->NewElement("limit");
          newLimit = true;
        }

        tinyxml2::XMLElement *axis = _elem->FirstChildElement("axis");
        bool newAxis = false;
        if (axis == nullptr)
        {
          axis = doc->NewElement("axis");
          newAxis = true;
        }

        tinyxml2::XMLElement *dynamics = axis->FirstChildElement("dynamics");
        bool newDynamics = false;
        if (dynamics == nullptr)
        {
          dynamics = doc->NewElement("dynamics");
          newDynamics = true;
        }

        // insert stopCfm, stopErp, fudgeFactor
        if ((*ge)->isStopCfm)
        {
          AddKeyValue(limit, "cfm", Values2str(1, &(*ge)->stopCfm));
        }
        if ((*ge)->isStopErp)
        {
          AddKeyValue(limit, "erp", Values2str(1, &(*ge)->stopErp));
        }
        if ((*ge)->isSpringReference)
        {
          AddKeyValue(dynamics, "spring_reference",
                      Values2str(1, &(*ge)->springReference));
        }
        if ((*ge)->isSpringStiffness)
        {
          AddKeyValue(dynamics, "spring_stiffness",
                      Values2str(1, &(*ge)->springStiffness));
        }

        // insert provideFeedback
        if ((*ge)->isProvideFeedback)
        {
          if ((*ge)->provideFeedback)
          {
            AddKeyValue(physics, "provide_feedback", "true");
            AddKeyValue(physicsOde, "provide_feedback", "true");
          }
          else
          {
            AddKeyValue(physics, "provide_feedback", "false");
            AddKeyValue(physicsOde, "provide_feedback", "false");
          }
        }

        // insert implicitSpringDamper
        if ((*ge)->isImplicitSpringDamper)
        {
          if ((*ge)->implicitSpringDamper)
          {
            AddKeyValue(physicsOde, "implicit_spring_damper", "true");
            /// \TODO: deprecating cfm_damping, transitional tag below
            AddKeyValue(physicsOde, "cfm_damping", "true");
          }
          else
          {
            AddKeyValue(physicsOde, "implicit_spring_damper", "false");
            /// \TODO: deprecating cfm_damping, transitional tag below
            AddKeyValue(physicsOde, "cfm_damping", "false");
          }
        }

        // insert fudgeFactor
        if ((*ge)->isFudgeFactor)
        {
          AddKeyValue(physicsOde, "fudge_factor",
                      Values2str(1, &(*ge)->fudgeFactor));
        }

        if (newDynamics)
        {
          axis->LinkEndChild(dynamics);
        }
        if (newAxis)
        {
          _elem->LinkEndChild(axis);
        }

        if (newLimit)
        {
          physicsOde->LinkEndChild(limit);
        }
        if (newPhysicsOde)
        {
          physics->LinkEndChild(physicsOde);
        }
        if (newPhysics)
        {
          _elem->LinkEndChild(physics);
        }

        // insert all additional blobs into joint
        for (auto blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          CopyBlob((*blobIt)->FirstChildElement(), _elem);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void InsertSDFExtensionRobot(tinyxml2::XMLElement *_elem)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first.empty())
    {
      // no reference specified
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        // insert static flag
        if ((*ge)->setStaticFlag)
        {
          AddKeyValue(_elem, "static", "true");
        }
        else
        {
          AddKeyValue(_elem, "static", "false");
        }

        // copy extension containing blobs and without reference
        for (auto blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          CopyBlob((*blobIt)->FirstChildElement(), _elem);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateGeometry(tinyxml2::XMLElement* _elem,
                    urdf::GeometrySharedPtr _geometry)
{
  auto* doc = _elem->GetDocument();
  tinyxml2::XMLElement *sdfGeometry = doc->NewElement("geometry");

  std::string type;
  tinyxml2::XMLElement *geometryType = nullptr;

  switch (_geometry->type)
  {
    case urdf::Geometry::BOX:
      type = "box";
      {
        urdf::BoxConstSharedPtr box =
          urdf::dynamic_pointer_cast<urdf::Box>(_geometry);
        int sizeCount = 3;
        double sizeVals[3];
        sizeVals[0] = box->dim.x;
        sizeVals[1] = box->dim.y;
        sizeVals[2] = box->dim.z;
        geometryType = doc->NewElement(type.c_str());
        AddKeyValue(geometryType, "size", Values2str(sizeCount, sizeVals));
      }
      break;
    case urdf::Geometry::CYLINDER:
      type = "cylinder";
      {
        urdf::CylinderConstSharedPtr cylinder =
          urdf::dynamic_pointer_cast<urdf::Cylinder>(_geometry);
        geometryType = doc->NewElement(type.c_str());
        AddKeyValue(geometryType, "length", Values2str(1, &cylinder->length));
        AddKeyValue(geometryType, "radius", Values2str(1, &cylinder->radius));
      }
      break;
    case urdf::Geometry::SPHERE:
      type = "sphere";
      {
        urdf::SphereConstSharedPtr sphere =
          urdf::dynamic_pointer_cast<urdf::Sphere>(_geometry);
        geometryType = doc->NewElement(type.c_str());
        AddKeyValue(geometryType, "radius", Values2str(1, &sphere->radius));
      }
      break;
    case urdf::Geometry::MESH:
      type = "mesh";
      {
        urdf::MeshConstSharedPtr mesh =
          urdf::dynamic_pointer_cast<urdf::Mesh>(_geometry);
        geometryType = doc->NewElement(type.c_str());
        AddKeyValue(geometryType, "scale", Vector32Str(mesh->scale));
        // do something more to meshes
        {
          // set mesh file
          if (mesh->filename.empty())
          {
            sdferr << "urdf2sdf: mesh geometry with no filename given.\n";
          }

          // give some warning if file does not exist.
          // disabled while switching to uri
          // @todo: re-enable check
          // std::ifstream fin;
          // fin.open(mesh->filename.c_str(), std::ios::in);
          // fin.close();
          // if (fin.fail())
          //   sdfwarn << "filename referred by mesh ["
          //          << mesh->filename << "] does not appear to exist.\n";

          // Convert package:// to model://,
          // in ROS, this will work if
          // the model package is in ROS_PACKAGE_PATH and has a manifest.xml
          // as a typical ros package does.
          std::string modelFilename = mesh->filename;
          std::string packagePrefix("package://");
          std::string modelPrefix("model://");
          size_t pos1 = modelFilename.find(packagePrefix, 0);
          if (pos1 != std::string::npos)
          {
            size_t repLen = packagePrefix.size();
            modelFilename.replace(pos1, repLen, modelPrefix);
            // sdfwarn << "ros style uri [package://] is"
            //   << "automatically converted: [" << modelFilename
            //   << "], make sure your ros package is in GAZEBO_MODEL_PATH"
            //   << " and switch your manifest to conform to sdf's"
            //   << " model database format.  See ["
            //   << "http://sdfsim.org/wiki/Model_database#Model_Manifest_XML"
            //   << "] for more info.\n";
          }

          // add mesh filename
          AddKeyValue(geometryType, "uri", modelFilename);
        }
      }
      break;
    default:
      sdfwarn << "Unknown body type: [" << static_cast<int>(_geometry->type)
              << "] skipped in geometry\n";
      break;
  }

  if (geometryType)
  {
    sdfGeometry->LinkEndChild(geometryType);
    _elem->LinkEndChild(sdfGeometry);
  }
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose TransformToParentFrame(urdf::Pose _transformInLinkFrame,
                                  urdf::Pose _parentToLinkTransform)
{
  // transform to ignition::math::Pose3d then call TransformToParentFrame
  ignition::math::Pose3d p1 = CopyPose(_transformInLinkFrame);
  ignition::math::Pose3d p2 = CopyPose(_parentToLinkTransform);
  return CopyPose(TransformToParentFrame(p1, p2));
}

////////////////////////////////////////////////////////////////////////////////
ignition::math::Pose3d TransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform)
{
  // transform to ignition::math::Pose3d then call TransformToParentFrame
  ignition::math::Pose3d p2 = CopyPose(_parentToLinkTransform);
  return TransformToParentFrame(_transformInLinkFrame, p2);
}

////////////////////////////////////////////////////////////////////////////////
ignition::math::Pose3d TransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    ignition::math::Pose3d _parentToLinkTransform)
{
  ignition::math::Pose3d transformInParentLinkFrame;
  // rotate link pose to parentLink frame
  transformInParentLinkFrame.Pos() =
    _parentToLinkTransform.Rot() * _transformInLinkFrame.Pos();
  transformInParentLinkFrame.Rot() =
    _parentToLinkTransform.Rot() * _transformInLinkFrame.Rot();
  // translate link to parentLink frame
  transformInParentLinkFrame.Pos() =
    _parentToLinkTransform.Pos() + transformInParentLinkFrame.Pos();

  return transformInParentLinkFrame;
}

/////////////////////////////////////////////////
/// reduced fixed joints: transform to parent frame
ignition::math::Pose3d inverseTransformToParentFrame(
    ignition::math::Pose3d _transformInLinkFrame,
    urdf::Pose _parentToLinkTransform)
{
  ignition::math::Pose3d transformInParentLinkFrame;
  //   rotate link pose to parentLink frame
  urdf::Rotation ri = _parentToLinkTransform.rotation.GetInverse();
  ignition::math::Quaterniond q1(ri.w, ri.x, ri.y, ri.z);
  transformInParentLinkFrame.Pos() = q1 * _transformInLinkFrame.Pos();
  urdf::Rotation r2 = _parentToLinkTransform.rotation.GetInverse();
  ignition::math::Quaterniond q3(r2.w, r2.x, r2.y, r2.z);
  transformInParentLinkFrame.Rot() = q3 * _transformInLinkFrame.Rot();
  //   translate link to parentLink frame
  transformInParentLinkFrame.Pos().X() = transformInParentLinkFrame.Pos().X()
    - _parentToLinkTransform.position.x;
  transformInParentLinkFrame.Pos().Y() = transformInParentLinkFrame.Pos().Y()
    - _parentToLinkTransform.position.y;
  transformInParentLinkFrame.Pos().Z() = transformInParentLinkFrame.Pos().Z()
    - _parentToLinkTransform.position.z;

  return transformInParentLinkFrame;
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionToParent(urdf::LinkSharedPtr _link)
{
  /// \todo: move to header
  /// Take the link's existing list of gazebo extensions, transfer them
  /// into parent link.  Along the way, update local transforms by adding
  /// the additional transform to parent.  Also, look through all
  /// referenced link names with plugins and update references to current
  /// link to the parent link. (reduceGazeboExtensionFrameReplace())

  /// @todo: this is a very complicated module that updates the plugins
  /// based on fixed joint reduction really wish this could be a lot cleaner

  std::string linkName = _link->name;

  // update extension map with references to linkName
  // this->ListSDFExtensions();
  StringSDFExtensionPtrMap::iterator ext = g_extensions.find(linkName);
  if (ext != g_extensions.end())
  {
    sdfdbg << "  REDUCE EXTENSION: moving reference from ["
           << linkName << "] to [" << _link->getParent()->name << "]\n";

    // update reduction transform (for rays, cameras for now).
    //   FIXME: contact frames too?
    for (std::vector<SDFExtensionPtr>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      (*ge)->reductionTransform = TransformToParentFrame(
          (*ge)->reductionTransform,
          _link->parent_joint->parent_to_joint_origin_transform);
      // for sensor and projector blocks only
      ReduceSDFExtensionsTransform((*ge));
    }

    // find pointer to the existing extension with the new _link reference
    std::string parentLinkName = _link->getParent()->name;
    StringSDFExtensionPtrMap::iterator parentExt =
      g_extensions.find(parentLinkName);

    // if none exist, create new extension with parentLinkName
    if (parentExt == g_extensions.end())
    {
      std::vector<SDFExtensionPtr> ge;
      g_extensions.insert(std::make_pair(parentLinkName, ge));
      parentExt = g_extensions.find(parentLinkName);
    }

    // move sdf extensions from _link into the parent _link's extensions
    for (std::vector<SDFExtensionPtr>::iterator ge = ext->second.begin();
         ge != ext->second.end(); ++ge)
    {
      parentExt->second.push_back(*ge);
    }
    ext->second.clear();
  }

  // for extensions with empty reference, search and replace
  // _link name patterns within the plugin with new _link name
  // and assign the proper reduction transform for the _link name pattern
  for (StringSDFExtensionPtrMap::iterator sdfIt = g_extensions.begin();
       sdfIt != g_extensions.end(); ++sdfIt)
  {
    // update reduction transform (for contacts, rays, cameras for now).
    for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
         ge != sdfIt->second.end(); ++ge)
    {
      ReduceSDFExtensionFrameReplace(*ge, _link);
    }
  }

  // this->ListSDFExtensions();
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionFrameReplace(SDFExtensionPtr _ge,
                                    urdf::LinkSharedPtr _link)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;

  // HACK: need to do this more generally, but we also need to replace
  //       all instances of _link name with new link name
  //       e.g. contact sensor refers to
  //         <collision>base_link_collision</collision>
  //         and it needs to be reparented to
  //         <collision>base_footprint_collision</collision>
  sdfdbg << "  STRING REPLACE: instances of _link name ["
        << linkName << "] with [" << parentLinkName << "]\n";
  for (auto blobIt = _ge->blobs.begin();
         blobIt != _ge->blobs.end(); ++blobIt)
  {
    tinyxml2::XMLPrinter debugStreamIn;
    (*blobIt)->Print(&debugStreamIn);
    sdfdbg << "        INITIAL STRING link ["
           << linkName << "]-->[" << parentLinkName << "]: ["
           << debugStreamIn.CStr() << "]\n";

    ReduceSDFExtensionContactSensorFrameReplace(blobIt, _link);
    ReduceSDFExtensionPluginFrameReplace(blobIt, _link,
                                         "plugin", "bodyName",
                                         _ge->reductionTransform);
    ReduceSDFExtensionPluginFrameReplace(blobIt, _link,
                                         "plugin", "frameName",
                                         _ge->reductionTransform);
    ReduceSDFExtensionProjectorFrameReplace(blobIt, _link);
    ReduceSDFExtensionGripperFrameReplace(blobIt, _link);
    ReduceSDFExtensionJointFrameReplace(blobIt, _link);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionsTransform(SDFExtensionPtr _ge)
{
  for (auto blobIt = _ge->blobs.begin();
         blobIt != _ge->blobs.end(); ++blobIt)
  {
    /// @todo make sure we are not missing any additional transform reductions
    ReduceSDFExtensionSensorTransformReduction(blobIt,
                                               _ge->reductionTransform);
    ReduceSDFExtensionProjectorTransformReduction(blobIt,
                                                  _ge->reductionTransform);
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ListSDFExtensions()
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    int extCount = 0;
    for (std::vector<SDFExtensionPtr>::iterator ge = sdfIt->second.begin();
         ge != sdfIt->second.end(); ++ge)
    {
      if (!(*ge)->blobs.empty())
      {
        sdfdbg <<  "  PRINTING [" << static_cast<int>((*ge)->blobs.size())
               << "] BLOBS for extension [" << ++extCount
               << "] referencing [" << sdfIt->first << "]\n";
        for (auto blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          tinyxml2::XMLPrinter streamIn;
          (*blobIt)->Print(&streamIn);
          sdfdbg << "    BLOB: [" << streamIn.CStr() << "]\n";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::ListSDFExtensions(const std::string &_reference)
{
  for (StringSDFExtensionPtrMap::iterator
      sdfIt = g_extensions.begin();
      sdfIt != g_extensions.end(); ++sdfIt)
  {
    if (sdfIt->first == _reference)
    {
      sdfdbg <<  "  PRINTING [" << static_cast<int>(sdfIt->second.size())
             << "] extensions referencing [" << _reference << "]\n";
      for (std::vector<SDFExtensionPtr>::iterator
          ge = sdfIt->second.begin(); ge != sdfIt->second.end(); ++ge)
      {
        for (auto blobIt = (*ge)->blobs.begin();
            blobIt != (*ge)->blobs.end(); ++blobIt)
        {
          tinyxml2::XMLPrinter streamIn;
          (*blobIt)->Print(&streamIn);
          sdfdbg << "    BLOB: [" << streamIn.CStr() << "]\n";
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateSDF(tinyxml2::XMLElement *_root,
               urdf::LinkConstSharedPtr _link,
               const ignition::math::Pose3d &_transform)
{
  ignition::math::Pose3d _currentTransform = _transform;

  // must have an <inertial> block and cannot have zero mass.
  //  allow det(I) == zero, in the case of point mass geoms.
  // @todo:  keyword "world" should be a constant defined somewhere else
  if (_link->name != "world" &&
      ((!_link->inertial) || ignition::math::equal(_link->inertial->mass, 0.0)))
  {
    if (!_link->child_links.empty())
    {
      sdfdbg << "urdf2sdf: link[" << _link->name
             << "] has no inertia, ["
             << static_cast<int>(_link->child_links.size())
             << "] children links ignored.\n";
    }

    if (!_link->child_joints.empty())
    {
      sdfdbg << "urdf2sdf: link[" << _link->name
             << "] has no inertia, ["
             << static_cast<int>(_link->child_links.size())
             << "] children joints ignored.\n";
    }

    if (_link->parent_joint)
    {
      sdfdbg << "urdf2sdf: link[" << _link->name
             << "] has no inertia, "
             << "parent joint [" << _link->parent_joint->name
             << "] ignored.\n";
    }

    sdfdbg << "urdf2sdf: link[" << _link->name
           << "] has no inertia, not modeled in sdf\n";
    return;
  }

  // create <body:...> block for non fixed joint attached bodies
  if ((_link->getParent() && _link->getParent()->name == "world") ||
      !g_reduceFixedJoints ||
      (!_link->parent_joint ||
       !FixedJointShouldBeReduced(_link->parent_joint)))
  {
    CreateLink(_root, _link, _currentTransform);
  }

  // recurse into children
  for (unsigned int i = 0 ; i < _link->child_links.size() ; ++i)
  {
    CreateSDF(_root, _link->child_links[i], _currentTransform);
  }
}

////////////////////////////////////////////////////////////////////////////////
ignition::math::Pose3d CopyPose(urdf::Pose _pose)
{
  ignition::math::Pose3d p;
  p.Pos().X() = _pose.position.x;
  p.Pos().Y() = _pose.position.y;
  p.Pos().Z() = _pose.position.z;
  p.Rot().X() = _pose.rotation.x;
  p.Rot().Y() = _pose.rotation.y;
  p.Rot().Z() = _pose.rotation.z;
  p.Rot().W() = _pose.rotation.w;
  return p;
}

////////////////////////////////////////////////////////////////////////////////
urdf::Pose CopyPose(ignition::math::Pose3d _pose)
{
  urdf::Pose p;
  p.position.x = _pose.Pos().X();
  p.position.y = _pose.Pos().Y();
  p.position.z = _pose.Pos().Z();
  p.rotation.x = _pose.Rot().X();
  p.rotation.y = _pose.Rot().Y();
  p.rotation.z = _pose.Rot().Z();
  p.rotation.w = _pose.Rot().W();
  return p;
}

////////////////////////////////////////////////////////////////////////////////
void CreateLink(tinyxml2::XMLElement *_root,
                urdf::LinkConstSharedPtr _link,
                ignition::math::Pose3d &_currentTransform)
{
  // create new body
  tinyxml2::XMLElement *elem = _root->GetDocument()->NewElement("link");

  // set body name
  elem->SetAttribute("name", _link->name.c_str());

  // compute global transform
  ignition::math::Pose3d localTransform;
  // this is the transform from parent link to current _link
  // this transform does not exist for the root link
  if (_link->parent_joint)
  {
    tinyxml2::XMLElement *pose = _root->GetDocument()->NewElement("pose");
    pose->SetAttribute("relative_to", _link->parent_joint->name.c_str());
    elem->LinkEndChild(pose);
  }
  else
  {
    sdfdbg << "[" << _link->name << "] has no parent joint\n";

    if (_currentTransform != ignition::math::Pose3d::Zero)
    {
      // create origin tag for this element
      AddTransform(elem, _currentTransform);
    }
  }

  // create new inerial block
  CreateInertial(elem, _link);

  // create new collision block
  CreateCollisions(elem, _link);

  // create new visual block
  CreateVisuals(elem, _link);

  // copy sdf extensions data
  InsertSDFExtensionLink(elem, _link->name);

  // make a <joint:...> block
  CreateJoint(_root, _link, _currentTransform);

  // add body to document
  _root->LinkEndChild(elem);
}

////////////////////////////////////////////////////////////////////////////////
void CreateCollisions(tinyxml2::XMLElement* _elem,
                      urdf::LinkConstSharedPtr _link)
{
  // loop through all collisions in
  //   collision_array (urdf 0.3.x)
  //   collision_groups (urdf 0.2.x)
  // and create collision sdf blocks.
  // Note, well as additional collision from
  //   lumped meshes (fixed joint reduction)
  unsigned int collisionCount = 0;
  for (std::vector<urdf::CollisionSharedPtr>::const_iterator
      collision = _link->collision_array.begin();
      collision != _link->collision_array.end();
      ++collision)
  {
    sdfdbg << "creating collision for link [" << _link->name
           << "] collision [" << (*collision)->name << "]\n";

    // collision sdf has a name if it was lumped/reduced
    // otherwise, use the link name
    std::string collisionName = (*collision)->name;
    if (collisionName.empty())
    {
      collisionName = _link->name;
    }

    // add _collision extension
    collisionName = collisionName + g_collisionExt;

    if (collisionCount > 0)
    {
      std::ostringstream collisionNameStream;
      collisionNameStream << collisionName
                          << "_" << collisionCount;
      collisionName = collisionNameStream.str();
    }

    // make a <collision> block
    CreateCollision(_elem, _link, *collision, collisionName);

    ++collisionCount;
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateVisuals(tinyxml2::XMLElement* _elem,
                   urdf::LinkConstSharedPtr _link)
{
  // loop through all visuals in
  //   visual_array (urdf 0.3.x)
  //   visual_groups (urdf 0.2.x)
  // and create visual sdf blocks.
  // Note, well as additional visual from
  //   lumped meshes (fixed joint reduction)
  unsigned int visualCount = 0;
  for (std::vector<urdf::VisualSharedPtr>::const_iterator
      visual = _link->visual_array.begin();
      visual != _link->visual_array.end();
      ++visual)
  {
    sdfdbg << "creating visual for link [" << _link->name
           << "] visual [" << (*visual)->name << "]\n";

    // visual sdf has a name if it was lumped/reduced
    // otherwise, use the link name
    std::string visualName = (*visual)->name;
    if (visualName.empty())
    {
      visualName = _link->name;
    }

    // add _visual extension
    visualName = visualName + g_visualExt;

    if (visualCount > 0)
    {
      std::ostringstream visualNameStream;
      visualNameStream << visualName
                       << "_" << visualCount;
      visualName = visualNameStream.str();
    }

    // make a <visual> block
    CreateVisual(_elem, _link, *visual, visualName);

    ++visualCount;
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateInertial(tinyxml2::XMLElement *_elem,
                    urdf::LinkConstSharedPtr _link)
{
  auto* doc = _elem->GetDocument();
  tinyxml2::XMLElement *inertial = doc->NewElement("inertial");

  // set mass properties
  // check and print a warning message
  double roll, pitch, yaw;
  _link->inertial->origin.rotation.getRPY(roll, pitch, yaw);

  /// add pose
  ignition::math::Pose3d pose = CopyPose(_link->inertial->origin);
  AddTransform(inertial, pose);

  // add mass
  AddKeyValue(inertial, "mass",
              Values2str(1, &_link->inertial->mass));

  // add inertia (ixx, ixy, ixz, iyy, iyz, izz)
  tinyxml2::XMLElement *inertia = doc->NewElement("inertia");
  AddKeyValue(inertia, "ixx",
              Values2str(1, &_link->inertial->ixx));
  AddKeyValue(inertia, "ixy",
              Values2str(1, &_link->inertial->ixy));
  AddKeyValue(inertia, "ixz",
              Values2str(1, &_link->inertial->ixz));
  AddKeyValue(inertia, "iyy",
              Values2str(1, &_link->inertial->iyy));
  AddKeyValue(inertia, "iyz",
              Values2str(1, &_link->inertial->iyz));
  AddKeyValue(inertia, "izz",
              Values2str(1, &_link->inertial->izz));
  inertial->LinkEndChild(inertia);

  _elem->LinkEndChild(inertial);
}

////////////////////////////////////////////////////////////////////////////////
void CreateJoint(tinyxml2::XMLElement *_root,
                 urdf::LinkConstSharedPtr _link,
                 ignition::math::Pose3d &/*_currentTransform*/)
{
  // compute the joint tag
  std::string jtype;
  jtype.clear();
  if (_link->parent_joint != nullptr)
  {
    switch (_link->parent_joint->type)
    {
      case urdf::Joint::CONTINUOUS:
      case urdf::Joint::REVOLUTE:
        jtype = "revolute";
        break;
      case urdf::Joint::PRISMATIC:
        jtype = "prismatic";
        break;
      case urdf::Joint::FLOATING:
      case urdf::Joint::PLANAR:
        break;
      case urdf::Joint::FIXED:
        jtype = "fixed";
        break;
      default:
        sdfwarn << "Unknown joint type: ["
                << static_cast<int>(_link->parent_joint->type)
                << "] in link [" << _link->name << "]\n";
        break;
    }
  }

  // If this is a fixed joint and the legacy option disableFixedJointLumping
  // is present and the new option preserveFixedJoint is not, then the fixed
  // joint should be converted to a revolute joint with max and mim position
  // limits set to (0, 0) for backward compatibility
  bool fixedJointConvertedToRevoluteJoint = false;
  if (jtype == "fixed")
  {
    fixedJointConvertedToRevoluteJoint =
      (g_fixedJointsTransformedInRevoluteJoints.find(_link->parent_joint->name)
       != g_fixedJointsTransformedInRevoluteJoints.end());
  }

  // skip if joint type is fixed and it is lumped
  //   skip/return with the exception of root link being world,
  //   because there's no lumping there
  if (_link->getParent() && _link->getParent()->name != "world"
      && FixedJointShouldBeReduced(_link->parent_joint)
      && g_reduceFixedJoints)
  {
    return;
  }

  if (!jtype.empty())
  {
    auto* doc = _root->GetDocument();
    tinyxml2::XMLElement *joint = doc->NewElement("joint");
    if (jtype == "fixed" && fixedJointConvertedToRevoluteJoint)
    {
      joint->SetAttribute("type", "revolute");
    }
    else
    {
      joint->SetAttribute("type", jtype.c_str());
    }
    joint->SetAttribute("name", _link->parent_joint->name.c_str());
    // Add joint pose relative to parent link
    AddTransform(
        joint, CopyPose(_link->parent_joint->parent_to_joint_origin_transform));
    auto pose = joint->FirstChildElement("pose");
    std::string relativeToAttr = _link->getParent()->name;
    if ("world" == relativeToAttr )
    {
      relativeToAttr = "__model__";
    }
    pose->SetAttribute("relative_to", relativeToAttr.c_str());

    AddKeyValue(joint, "parent", _link->getParent()->name);
    AddKeyValue(joint, "child", _link->name);

    tinyxml2::XMLElement *jointAxis = doc->NewElement("axis");
    tinyxml2::XMLElement *jointAxisLimit = doc->NewElement("limit");
    tinyxml2::XMLElement *jointAxisDynamics = doc->NewElement("dynamics");
    if (jtype == "fixed" && fixedJointConvertedToRevoluteJoint)
    {
      AddKeyValue(jointAxisLimit, "lower", "0");
      AddKeyValue(jointAxisLimit, "upper", "0");
      AddKeyValue(jointAxisDynamics, "damping", "0");
      AddKeyValue(jointAxisDynamics, "friction", "0");
    }
    else if (jtype != "fixed")
    {
      double jointAxisXyzArray[3] =
      { _link->parent_joint->axis.x,
        _link->parent_joint->axis.y,
        _link->parent_joint->axis.z};
      AddKeyValue(jointAxis, "xyz",
                  Values2str(3, jointAxisXyzArray));
      if (_link->parent_joint->dynamics)
      {
        AddKeyValue(jointAxisDynamics, "damping",
                    Values2str(1, &_link->parent_joint->dynamics->damping));
        AddKeyValue(jointAxisDynamics, "friction",
                    Values2str(1, &_link->parent_joint->dynamics->friction));
      }

      if (g_enforceLimits && _link->parent_joint->limits)
      {
        if (jtype == "slider")
        {
          AddKeyValue(jointAxisLimit, "lower",
                      Values2str(1, &_link->parent_joint->limits->lower));
          AddKeyValue(jointAxisLimit, "upper",
                      Values2str(1, &_link->parent_joint->limits->upper));
          AddKeyValue(jointAxisLimit, "effort",
                      Values2str(1, &_link->parent_joint->limits->effort));
          AddKeyValue(jointAxisLimit, "velocity",
                      Values2str(1, &_link->parent_joint->limits->velocity));
        }
        else if (_link->parent_joint->type != urdf::Joint::CONTINUOUS)
        {
          double *lowstop  = &_link->parent_joint->limits->lower;
          double *highstop = &_link->parent_joint->limits->upper;
          // enforce ode bounds, this will need to be fixed
          if (*lowstop > *highstop)
          {
            sdfwarn << "urdf2sdf: revolute joint ["
                    << _link->parent_joint->name
                    << "] with limits: lowStop[" << *lowstop
                    << "] > highStop[" << *highstop
                    << "], switching the two.\n";
            double tmp = *lowstop;
            *lowstop = *highstop;
            *highstop = tmp;
          }
          AddKeyValue(jointAxisLimit, "lower",
                      Values2str(1, &_link->parent_joint->limits->lower));
          AddKeyValue(jointAxisLimit, "upper",
                      Values2str(1, &_link->parent_joint->limits->upper));
          AddKeyValue(jointAxisLimit, "effort",
                      Values2str(1, &_link->parent_joint->limits->effort));
          AddKeyValue(jointAxisLimit, "velocity",
                      Values2str(1, &_link->parent_joint->limits->velocity));
        }
      }
    }

    if (jtype == "fixed" && !fixedJointConvertedToRevoluteJoint)
    {
      doc->DeleteNode(jointAxisLimit);
      jointAxisLimit = 0;
      doc->DeleteNode(jointAxisDynamics);
      jointAxisDynamics = 0;
      doc->DeleteNode(jointAxis);
      jointAxis = 0;
    }
    else
    {
      jointAxis->LinkEndChild(jointAxisLimit);
      jointAxis->LinkEndChild(jointAxisDynamics);
      joint->LinkEndChild(jointAxis);
    }

    // copy sdf extensions data
    InsertSDFExtensionJoint(joint, _link->parent_joint->name);

    // add joint to document
    _root->LinkEndChild(joint);
  }
}

////////////////////////////////////////////////////////////////////////////////
void CreateCollision(tinyxml2::XMLElement* _elem,
                     urdf::LinkConstSharedPtr _link,
                     urdf::CollisionSharedPtr _collision,
                     const std::string &_oldLinkName)
{
  auto* doc = _elem->GetDocument();
  // begin create geometry node, skip if no collision specified
  tinyxml2::XMLElement *sdfCollision = doc->NewElement("collision");

  // std::cerr << "CreateCollision link [" << _link->name
  //           << "] old [" << _oldLinkName
  //           << "]\n";
  // set its name, if lumped, add original link name
  // for meshes in an original mesh, it's likely
  // _link->name + mesh count
  if (_oldLinkName.compare(0, _link->name.size(), _link->name) == 0 ||
      _oldLinkName.empty())
  {
    sdfCollision->SetAttribute("name", _oldLinkName.c_str());
  }
  else
  {
    sdfCollision->SetAttribute("name", (_link->name
        + g_lumpPrefix + _oldLinkName).c_str());
  }

  // std::cerr << "collision [" << sdfCollision->Attribute("name") << "]\n";

  // set transform
  double pose[6];
  pose[0] = _collision->origin.position.x;
  pose[1] = _collision->origin.position.y;
  pose[2] = _collision->origin.position.z;
  _collision->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
  AddKeyValue(sdfCollision, "pose", Values2str(6, pose));

  // add geometry block
  if (!_collision || !_collision->geometry)
  {
    sdfdbg << "urdf2sdf: collision of link [" << _link->name
           << "] has no <geometry>.\n";
  }
  else
  {
    CreateGeometry(sdfCollision, _collision->geometry);
  }

  // set additional data from extensions
  InsertSDFExtensionCollision(sdfCollision, _link->name);

  // add geometry to body
  _elem->LinkEndChild(sdfCollision);
}

////////////////////////////////////////////////////////////////////////////////
void CreateVisual(tinyxml2::XMLElement *_elem, urdf::LinkConstSharedPtr _link,
    urdf::VisualSharedPtr _visual, const std::string &_oldLinkName)
{
  auto* doc = _elem->GetDocument();
  // begin create sdf visual node
  tinyxml2::XMLElement *sdfVisual = doc->NewElement("visual");

  // set its name
  if (_oldLinkName.compare(0, _link->name.size(), _link->name) == 0 ||
      _oldLinkName.empty())
  {
    sdfVisual->SetAttribute("name", _oldLinkName.c_str());
  }
  else
  {
    sdfVisual->SetAttribute("name",
        (_link->name + g_lumpPrefix + _oldLinkName).c_str());
  }

  // add the visualisation transfrom
  double pose[6];
  pose[0] = _visual->origin.position.x;
  pose[1] = _visual->origin.position.y;
  pose[2] = _visual->origin.position.z;
  _visual->origin.rotation.getRPY(pose[3], pose[4], pose[5]);
  AddKeyValue(sdfVisual, "pose", Values2str(6, pose));

  // insert geometry
  if (!_visual || !_visual->geometry)
  {
    sdfdbg << "urdf2sdf: visual of link [" << _link->name
           << "] has no <geometry>.\n";
  }
  else
  {
    CreateGeometry(sdfVisual, _visual->geometry);
  }

  // set additional data from extensions
  InsertSDFExtensionVisual(sdfVisual, _link->name);

  // end create _visual node
  _elem->LinkEndChild(sdfVisual);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::InitModelString(const std::string &_urdfStr,
                               tinyxml2::XMLDocument* _sdfXmlOut,
                               bool _enforceLimits)
{
  g_enforceLimits = _enforceLimits;

  // Create a RobotModel from string
  urdf::ModelInterfaceSharedPtr robotModel = urdf::parseURDF(_urdfStr);

  if (!robotModel)
  {
    sdferr << "Unable to call parseURDF on robot model\n";
    return;
  }

  // create root element and define needed namespaces
  tinyxml2::XMLElement *robot = _sdfXmlOut->NewElement("model");

  // set model name to urdf robot name if not specified
  robot->SetAttribute("name", robotModel->getName().c_str());

  // initialize transform for the model, urdf is recursive,
  // while sdf defines all links relative to model frame
  ignition::math::Pose3d transform;

  // parse sdf extension
  tinyxml2::XMLDocument urdfXml;
  if (urdfXml.Parse(_urdfStr.c_str()))
  {
    sdferr << "Unable to parse URDF string: " << urdfXml.ErrorStr() << "\n";
    return;
  }
  g_extensions.clear();
  g_fixedJointsTransformedInFixedJoints.clear();
  g_fixedJointsTransformedInRevoluteJoints.clear();
  this->ParseSDFExtension(urdfXml);

  // Parse robot pose
  ParseRobotOrigin(urdfXml);

  urdf::LinkConstSharedPtr rootLink = robotModel->getRoot();
  tinyxml2::XMLElement *sdf;

  try
  {
    // set model name to urdf robot name if not specified
    robot->SetAttribute("name", robotModel->getName().c_str());

    // Fixed Joint Reduction
    // if link connects to parent via fixed joint, lump down and remove link
    // set reduceFixedJoints to false will replace fixed joints with
    // zero limit revolute joints, otherwise, we reduce it down to its
    // parent link recursively
    // using the disabledFixedJointLumping or preserveFixedJoint options
    // is possible to disable fixed joint lumping only for selected joints
    if (g_reduceFixedJoints)
    {
      ReduceFixedJoints(robot, urdf::const_pointer_cast<urdf::Link>(rootLink));
    }

    if (rootLink->name == "world")
    {
      // convert all children link
      for (std::vector<urdf::LinkSharedPtr>::const_iterator
          child = rootLink->child_links.begin();
          child != rootLink->child_links.end(); ++child)
      {
        CreateSDF(robot, (*child), transform);
      }
    }
    else
    {
      // convert, starting from root link
      CreateSDF(robot, rootLink, transform);
    }

    // insert the extensions without reference into <robot> root level
    InsertSDFExtensionRobot(robot);

    InsertRobotOrigin(robot);

    // Create new sdf
    sdf = _sdfXmlOut->NewElement("sdf");

    try
    {
      // URDF is compatible with version 1.7. The automatic conversion script
      // will up-convert URDF to SDF.
      sdf->SetAttribute("version", "1.7");
      // add robot to sdf
      sdf->LinkEndChild(robot);
    }
    catch(...)
    {
      throw;
    }
  }
  catch(...)
  {
    throw;
  }

  _sdfXmlOut->LinkEndChild(sdf);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::InitModelDoc(const tinyxml2::XMLDocument *_xmlDoc,
                            tinyxml2::XMLDocument *_sdfXmlDoc)
{
  tinyxml2::XMLPrinter printer;
  _xmlDoc->Print(&printer);
  std::string urdfStr = printer.CStr();
  InitModelString(urdfStr, _sdfXmlDoc);
}

////////////////////////////////////////////////////////////////////////////////
void URDF2SDF::InitModelFile(const std::string &_filename,
                             tinyxml2::XMLDocument *_sdfXmlDoc)
{
  tinyxml2::XMLDocument xmlDoc;
  if (!xmlDoc.LoadFile(_filename.c_str()))
  {
    this->InitModelDoc(&xmlDoc, _sdfXmlDoc);
  }
  else
  {
    sdferr << "Unable to load file["
      << _filename << "]:" << xmlDoc.ErrorStr() << "\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
bool FixedJointShouldBeReduced(urdf::JointSharedPtr _jnt)
{
    // A joint should be lumped only if its type is fixed and
    // the disabledFixedJointLumping or preserveFixedJoint
    // joint options are not set
    return (_jnt->type == urdf::Joint::FIXED &&
              (g_fixedJointsTransformedInRevoluteJoints.find(_jnt->name) ==
                 g_fixedJointsTransformedInRevoluteJoints.end()) &&
              (g_fixedJointsTransformedInFixedJoints.find(_jnt->name) ==
                 g_fixedJointsTransformedInFixedJoints.end()));
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionSensorTransformReduction(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    ignition::math::Pose3d _reductionTransform)
{
  auto sensorElement = (*_blobIt)->FirstChildElement();
  if ( strcmp(sensorElement->Name(), "sensor") == 0)
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform

    // debug print
    // for (tinyxml2::XMLNode* elIt = (*_blobIt)->FirstChild();
    //      elIt; elIt = elIt->NextSibling())
    // {
    //   tinyxml2::XMLPrinter streamIn;
    //   elIt->Accept(&streamIn);
    //   sdfdbg << "    " << streamIn.CStr() << "\n";
    // }

    auto sensorPose {ignition::math::Pose3d::Zero};
    {
      auto sensorPosText = "0 0 0 0 0 0";
      const auto& oldPoseKey = sensorElement->FirstChildElement("pose");
      if (oldPoseKey)
      {
        const auto& poseElemXml = oldPoseKey->ToElement();
        if (poseElemXml->Attribute("relative_to"))
          return;

        // see below for explanation; if a sibling element exists, it stores the
        // original <sensor><pose> tag content
        const auto& sibling = sensorElement->NextSibling();
        if (poseElemXml->GetText() && !sibling)
          sensorPosText = poseElemXml->GetText();
        else if (sibling && sibling->ToElement()->GetText())
          sensorPosText = sibling->ToElement()->GetText();
        else
        {
          sdferr << "Unexpected case in sensor pose computation\n";
          return;
        }

        // delete the <pose> tag, we'll add a new one at the end
        sensorElement->DeleteChild(oldPoseKey);
      }

      // parse the 6-tuple text into math::Pose3d
      std::stringstream  ss;
      ss.imbue(std::locale::classic());
      ss << sensorPosText;
      ss >> sensorPose;
      if (ss.fail())
      {
        sdferr << "Could not parse <sensor><pose>: [" << sensorPosText << "]\n";
        return;
      }

      // critical part: we store the original <pose> tag from <sensor> actually
      // as a sibling of <sensor>... only first element of the blob is processed
      // further, so its siblings can be used as temporary storage; we store the
      // original <pose> tag there so that we can use the <sensor><pose> tag
      // for storing the reduced position
      auto doc = (*_blobIt)->GetDocument();
      const auto& poseTxt = doc->NewText(sensorPosText);
      auto poseKey = doc->NewElement("pose");
      poseKey->LinkEndChild(poseTxt);
      (*_blobIt)->LinkEndChild(poseKey);
    }

    _reductionTransform = _reductionTransform * sensorPose;

    // convert reductionTransform to values
    urdf::Vector3 reductionXyz(_reductionTransform.Pos().X(),
                               _reductionTransform.Pos().Y(),
                               _reductionTransform.Pos().Z());
    urdf::Rotation reductionQ(_reductionTransform.Rot().X(),
                              _reductionTransform.Rot().Y(),
                              _reductionTransform.Rot().Z(),
                              _reductionTransform.Rot().W());

    urdf::Vector3 reductionRpy;
    reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

    // output updated pose to text
    std::ostringstream poseStream;
    poseStream << reductionXyz.x << " " << reductionXyz.y
               << " " << reductionXyz.z << " " << reductionRpy.x
               << " " << reductionRpy.y << " " << reductionRpy.z;

    auto* doc = (*_blobIt)->GetDocument();
    tinyxml2::XMLText *poseTxt = doc->NewText(poseStream.str().c_str());
    tinyxml2::XMLElement *poseKey = doc->NewElement("pose");

    poseKey->LinkEndChild(poseTxt);

    sensorElement->LinkEndChild(poseKey);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionProjectorTransformReduction(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    ignition::math::Pose3d _reductionTransform)
{
  auto projectorElement = (*_blobIt)->FirstChildElement();
  if ( strcmp(projectorElement->Name(), "projector") == 0)
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    //
    // for (tinyxml2::XMLNode* elIt = (*_blobIt)->FirstChildElement();
    // elIt; elIt = elIt->NextSibling())
    // {
    //   std::ostringstream streamIn;
    //   streamIn << *elIt;
    //   sdfdbg << "    " << streamIn << "\n";
    // }

    auto projectorPose {ignition::math::Pose3d::Zero};
    {
      auto projectorPosText = "0 0 0 0 0 0";
      const auto& oldPoseKey = projectorElement->FirstChildElement("pose");
      if (oldPoseKey)
      {
        const auto& poseElemXml = oldPoseKey->ToElement();
        if (poseElemXml->Attribute("relative_to"))
          return;

        // see below for explanation; if a sibling element exists, it stores the
        // original <projector><pose> tag content
        const auto& sibling = projectorElement->NextSibling();
        if (poseElemXml->GetText() && !sibling)
          projectorPosText = poseElemXml->GetText();
        else if (sibling && sibling->ToElement()->GetText())
          projectorPosText = sibling->ToElement()->GetText();
        else
        {
          sdferr << "Unexpected case in projector pose computation\n";
          return;
        }

        // delete the <pose> tag, we'll add a new one at the end
        projectorElement->DeleteChild(oldPoseKey);
      }

      // parse the 6-tuple text into math::Pose3d
      std::stringstream  ss;
      ss.imbue(std::locale::classic());
      ss << projectorPosText;
      ss >> projectorPose;
      if (ss.fail())
      {
        sdferr << "Could not parse <projector><pose>: ["
               << projectorPosText << "]\n";
        return;
      }

      // critical part: we store the original <pose> tag from <projector>
      // actually as a sibling of <projector>... only first element of the blob
      // is processed further, so its siblings can be used as temporary storage;
      // we store the original <pose> tag there so that we can use the
      // <projector><pose> tag for storing the reduced position
      auto doc = (*_blobIt)->GetDocument();
      const auto& poseTxt = doc->NewText(projectorPosText);
      auto poseKey = doc->NewElement("pose");
      poseKey->LinkEndChild(poseTxt);
      (*_blobIt)->LinkEndChild(poseKey);
    }

    _reductionTransform = _reductionTransform * projectorPose;

    // convert reductionTransform to values
    urdf::Vector3 reductionXyz(_reductionTransform.Pos().X(),
                               _reductionTransform.Pos().Y(),
                               _reductionTransform.Pos().Z());
    urdf::Rotation reductionQ(_reductionTransform.Rot().X(),
                              _reductionTransform.Rot().Y(),
                              _reductionTransform.Rot().Z(),
                              _reductionTransform.Rot().W());

    urdf::Vector3 reductionRpy;
    reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);

    // output updated pose to text
    std::ostringstream poseStream;
    poseStream << reductionXyz.x << " " << reductionXyz.y
               << " " << reductionXyz.z << " " << reductionRpy.x
               << " " << reductionRpy.y << " " << reductionRpy.z;

    auto* doc = (*_blobIt)->GetDocument();
    tinyxml2::XMLText *poseTxt = doc->NewText(poseStream.str().c_str());
    auto poseKey = doc->NewElement("pose");
    poseKey->LinkEndChild(poseTxt);

    projectorElement->LinkEndChild(poseKey);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionContactSensorFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;
  if ( strcmp((*_blobIt)->FirstChildElement()->Name(), "sensor") == 0)
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    tinyxml2::XMLNode *contact = (*_blobIt)->FirstChildElement("contact");
    if (contact)
    {
      tinyxml2::XMLNode *collision = contact->FirstChildElement("collision");
      if (collision)
      {
        if (GetKeyValueAsString(collision->ToElement()) ==
            linkName + g_collisionExt)
        {
          contact->DeleteChild(collision);

          auto* doc = contact->GetDocument();
          tinyxml2::XMLElement *collisionNameKey = doc->NewElement("collision");
          std::ostringstream collisionNameStream;
          collisionNameStream << parentLinkName << g_collisionExt
                              << "_" << linkName;
          tinyxml2::XMLText *collisionNameTxt = doc->NewText(
              collisionNameStream.str().c_str());
          collisionNameKey->LinkEndChild(collisionNameTxt);
          contact->LinkEndChild(collisionNameKey);
        }
        // @todo: FIXME: chagning contact sensor's contact collision
        //   should trigger a update in sensor offset as well.
        //   But first we need to implement offsets in contact sensors
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionPluginFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link,
    const std::string &_pluginName, const std::string &_elementName,
    ignition::math::Pose3d _reductionTransform)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;
  if ((*_blobIt)->FirstChildElement()->Name() == _pluginName)
  {
    // replace element containing _link names to parent link names
    // find first instance of xyz and rpy, replace with reduction transform
    tinyxml2::XMLNode *elementNode =
      (*_blobIt)->FirstChildElement(_elementName.c_str());
    if (elementNode)
    {
      if (GetKeyValueAsString(elementNode->ToElement()) == linkName)
      {
        (*_blobIt)->DeleteChild(elementNode);
        auto* doc = elementNode->GetDocument();
        tinyxml2::XMLElement *bodyNameKey =
          doc->NewElement(_elementName.c_str());
        std::ostringstream bodyNameStream;
        bodyNameStream << parentLinkName;
        tinyxml2::XMLText *bodyNameTxt =
          doc->NewText(bodyNameStream.str().c_str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
        /// @todo update transforms for this sdf plugin too

        // look for offset transforms, add reduction transform
        tinyxml2::XMLNode *xyzKey = (*_blobIt)->FirstChildElement("xyzOffset");
        if (xyzKey)
        {
          urdf::Vector3 v1 = ParseVector3(xyzKey);
          _reductionTransform.Pos() =
            ignition::math::Vector3d(v1.x, v1.y, v1.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->DeleteChild(xyzKey);
        }
        tinyxml2::XMLNode *rpyKey = (*_blobIt)->FirstChildElement("rpyOffset");
        if (rpyKey)
        {
          urdf::Vector3 rpy = ParseVector3(rpyKey, M_PI/180.0);
          _reductionTransform.Rot() =
            ignition::math::Quaterniond::EulerToQuaternion(rpy.x, rpy.y, rpy.z);
          // remove xyzOffset and rpyOffset
          (*_blobIt)->DeleteChild(rpyKey);
        }

        // pass through the parent transform from fixed joint reduction
        _reductionTransform = inverseTransformToParentFrame(_reductionTransform,
            _link->parent_joint->parent_to_joint_origin_transform);

        // create new offset xml blocks
        xyzKey = doc->NewElement("xyzOffset");
        rpyKey = doc->NewElement("rpyOffset");

        // create new offset xml blocks
        urdf::Vector3 reductionXyz(_reductionTransform.Pos().X(),
                                   _reductionTransform.Pos().Y(),
                                   _reductionTransform.Pos().Z());
        urdf::Rotation reductionQ(_reductionTransform.Rot().X(),
                                  _reductionTransform.Rot().Y(),
                                  _reductionTransform.Rot().Z(),
                                  _reductionTransform.Rot().W());

        std::ostringstream xyzStream, rpyStream;
        xyzStream << reductionXyz.x << " " << reductionXyz.y << " "
                  << reductionXyz.z;
        urdf::Vector3 reductionRpy;
        reductionQ.getRPY(reductionRpy.x, reductionRpy.y, reductionRpy.z);
        rpyStream << reductionRpy.x << " " << reductionRpy.y << " "
                  << reductionRpy.z;

        tinyxml2::XMLText *xyzTxt = doc->NewText(xyzStream.str().c_str());
        tinyxml2::XMLText *rpyTxt = doc->NewText(rpyStream.str().c_str());

        xyzKey->LinkEndChild(xyzTxt);
        rpyKey->LinkEndChild(rpyTxt);

        (*_blobIt)->LinkEndChild(xyzKey);
        (*_blobIt)->LinkEndChild(rpyKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionProjectorFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;

  // updates _link reference for <projector> inside of
  // projector plugins
  // update from <projector>MyLinkName/MyProjectorName</projector>
  // to <projector>NewLinkName/MyProjectorName</projector>
  tinyxml2::XMLNode *projectorElem = (*_blobIt)->FirstChildElement("projector");
  {
    if (projectorElem)
    {
      std::string projectorName =  GetKeyValueAsString(
          projectorElem->ToElement());
      // extract projector _link name and projector name
      size_t pos = projectorName.find("/");
      if (pos == std::string::npos)
      {
        sdferr << "no slash in projector reference tag [" << projectorName
               << "], expecting linkName/projector_name.\n";
      }
      else
      {
        std::string projectorLinkName = projectorName.substr(0, pos);

        if (projectorLinkName == linkName)
        {
          // do the replacement
          projectorName = parentLinkName + "/" +
            projectorName.substr(pos+1, projectorName.size());

          (*_blobIt)->DeleteChild(projectorElem);
          auto* doc = projectorElem->GetDocument();
          tinyxml2::XMLElement *bodyNameKey = doc->NewElement("projector");
          std::ostringstream bodyNameStream;
          bodyNameStream << projectorName;
          tinyxml2::XMLText *bodyNameTxt =
            doc->NewText(bodyNameStream.str().c_str());
          bodyNameKey->LinkEndChild(bodyNameTxt);
          (*_blobIt)->LinkEndChild(bodyNameKey);
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionGripperFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;

  if (strcmp((*_blobIt)->FirstChildElement()->Name(), "gripper") == 0)
  {
    tinyxml2::XMLNode *gripperLink =
      (*_blobIt)->FirstChildElement("gripper_link");
    if (gripperLink)
    {
      if (GetKeyValueAsString(gripperLink->ToElement()) == linkName)
      {
        (*_blobIt)->DeleteChild(gripperLink);
        auto* doc = (*_blobIt)->GetDocument();
        tinyxml2::XMLElement *bodyNameKey = doc->NewElement("gripper_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << parentLinkName;
        tinyxml2::XMLText *bodyNameTxt =
          doc->NewText(bodyNameStream.str().c_str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
    tinyxml2::XMLNode *palmLink = (*_blobIt)->FirstChildElement("palm_link");
    if (palmLink)
    {
      if (GetKeyValueAsString(palmLink->ToElement()) == linkName)
      {
        (*_blobIt)->DeleteChild(palmLink);
        auto* doc = (*_blobIt)->GetDocument();
        tinyxml2::XMLElement *bodyNameKey =
            doc->NewElement("palm_link");
        std::ostringstream bodyNameStream;
        bodyNameStream << parentLinkName;
        tinyxml2::XMLText *bodyNameTxt =
          doc->NewText(bodyNameStream.str().c_str());
        bodyNameKey->LinkEndChild(bodyNameTxt);
        (*_blobIt)->LinkEndChild(bodyNameKey);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
void ReduceSDFExtensionJointFrameReplace(
    std::vector<XMLDocumentPtr>::iterator _blobIt,
    urdf::LinkSharedPtr _link)
{
  std::string linkName = _link->name;
  std::string parentLinkName = _link->getParent()->name;
  auto* doc = (*_blobIt)->GetDocument();

  if (strcmp((*_blobIt)->FirstChildElement()->Name(), "joint") == 0)
  {
    // parse it and add/replace the reduction transform
    // find first instance of xyz and rpy, replace with reduction transform
    tinyxml2::XMLNode *parent = (*_blobIt)->FirstChildElement("parent");
    if (parent)
    {
      if (GetKeyValueAsString(parent->ToElement()) == linkName)
      {
        (*_blobIt)->DeleteChild(parent);
        tinyxml2::XMLElement *parentNameKey = doc->NewElement("parent");
        std::ostringstream parentNameStream;
        parentNameStream << parentLinkName;
        tinyxml2::XMLText *parentNameTxt =
          doc->NewText(parentNameStream.str().c_str());
        parentNameKey->LinkEndChild(parentNameTxt);
        (*_blobIt)->LinkEndChild(parentNameKey);
      }
    }
    tinyxml2::XMLNode *child = (*_blobIt)->FirstChildElement("child");
    if (child)
    {
      if (GetKeyValueAsString(child->ToElement()) == linkName)
      {
        (*_blobIt)->DeleteChild(child);
        tinyxml2::XMLElement *childNameKey = doc->NewElement("child");
        std::ostringstream childNameStream;
        childNameStream << parentLinkName;
        tinyxml2::XMLText *childNameTxt =
          doc->NewText(childNameStream.str().c_str());
        childNameKey->LinkEndChild(childNameTxt);
        (*_blobIt)->LinkEndChild(childNameKey);
      }
    }
    /// @todo add anchor offsets if parent link changes location!
  }
}
}
}
