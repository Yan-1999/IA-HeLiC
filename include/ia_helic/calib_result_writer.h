#pragma once

#include <fstream>
#include <ios>
#include <ostream>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ia_helic/odom.h"
#include "ia_helic/utils.h"

namespace ia_helic
{
class [[deprecated]] CalibWriter
{
public:
  IA_HELIC_MAKE_EXCEPTION

private:
  std::string from_frame_;
  std::string to_frame_;

public:
  CalibWriter(std::string& from_frame, std::string& to_frame) : to_frame_(to_frame), from_frame_(from_frame)
  {
  }

  std::ostream& writeCalibResults(std::ostream& os, ia_helic::RigidTransform& T);
  void writeCalibYAML(std::string& filepath, ia_helic::RigidTransform& T)
  {
    std::fstream fs(filepath, std::ios_base::out | std::ios_base::trunc);
    if (fs.is_open())
    {
      writeCalibResults(fs, T);
    }
    else
    {
      IA_HELIC_THROW("Cannot open file: " + filepath);
    }
  }
};
}  // namespace ia_helic
