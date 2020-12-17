#include <tesseract_scene_graph/resource_locator.h>

#pragma once

class SimpleResourceLocatorFnBase
{
public:
  virtual std::string call(const std::string& a) = 0;
};