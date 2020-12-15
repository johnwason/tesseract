#include <tesseract_command_language/utils/filter_functions.h>

class flattenFilterFnBase
{
public:
    virtual bool call(const tesseract_planning::Instruction&, const tesseract_planning::CompositeInstruction&, bool parent_is_first_composite) = 0;
    virtual ~flattenFilterFnBase() {}
};

class locateFilterFnBase
{
public:
    virtual bool call(const tesseract_planning::Instruction&, const tesseract_planning::CompositeInstruction&, bool parent_is_first_composite) = 0;
    virtual ~locateFilterFnBase() {}
};