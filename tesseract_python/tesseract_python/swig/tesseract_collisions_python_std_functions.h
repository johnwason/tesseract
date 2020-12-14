#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>

#pragma once

class IsContactAllowedFnBase
{
public:
    virtual bool call(const std::string& a, const std::string& b) = 0;
};

class IsContactValidFnBase
{
public:
    virtual bool call(const tesseract_collision::ContactResult& a) = 0;
};

class ContinuousContactManagerFactory_CreateMethodBase
{
public:
    virtual tesseract_collision::ContinuousContactManager::Ptr call() = 0;
};

class DiscreteContactManagerFactory_CreateMethodBase
{
public:
    virtual tesseract_collision::DiscreteContactManager::Ptr call() = 0;
};