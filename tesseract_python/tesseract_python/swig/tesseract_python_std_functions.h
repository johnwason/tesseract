#include <tesseract/tesseract.h>

#pragma once

class FindTCPCallbackFnBase
{
public:
    virtual Eigen::Isometry3d call(const tesseract_planning::ManipulatorInfo& a) = 0;
}; 