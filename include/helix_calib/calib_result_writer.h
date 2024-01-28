#pragma once

#include <fstream>
#include <ios>
#include <ostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "helix_calib/odom.h"
#include "helix_calib/utils.h"

namespace helix
{
class [[deprecated]] CalibWriter
{
public:
  HELIX_MAKE_EXCEPTION

private:
  std::string from_frame_;
  std::string to_frame_;

public:
  CalibWriter(std::string& from_frame, std::string& to_frame) : to_frame_(to_frame), from_frame_(from_frame)
  {
  }

  std::ostream& writeCalibResults(std::ostream& os, helix::RigidTransform& T);
  void writeCalibYAML(std::string& filepath, helix::RigidTransform& T)
  {
    std::fstream fs(filepath, std::ios_base::out | std::ios_base::trunc);
    if (fs.is_open())
    {
      writeCalibResults(fs, T);
    }
    else
    {
      HELIX_THROW("Cannot open file: " + filepath);
    }
  }
};
}  // namespace helix
