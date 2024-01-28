#include <fmt/core.h>
#include <cmath>

#include "helix_calib/calib_result_writer.h"
#include "helix_calib/odom.h"

std::ostream& helix::CalibWriter::writeCalibResults(std::ostream& os, helix::RigidTransform& T)
{
  fmt::print(os,
             "{from_frame}:\n"
             "  extrinsics:\n"
             "#   rotX: {rotX}deg, rotY: {rotY}deg, rotZ: {rotZ}deg\n"
             "    quaternion: [{qw}, {qx}, {qy}, {qz}] # [w, x, y, z]\n"
             "    translation: [{x}, {y}, {z}]\n"
             "    parent: {to_frame}\n",
             fmt::arg("from_frame", from_frame_), fmt::arg("to_frame", to_frame_),
             fmt::arg("rotX", T.angleX() * 180 / M_PI), fmt::arg("rotY", T.angleY() * 180 / M_PI),
             fmt::arg("rotZ", T.angleZ() * 180 / M_PI), fmt::arg("qw", T.unit_quaternion().w()),
             fmt::arg("qx", T.unit_quaternion().x()), fmt::arg("qy", T.unit_quaternion().y()),
             fmt::arg("qz", T.unit_quaternion().z()), fmt::arg("x", T.translation().x()),
             fmt::arg("y", T.translation().y()), fmt::arg("z", T.translation().z()));
  return os;
}
