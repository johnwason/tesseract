%{
#include <tesseract_planning/basic_planner.h>	
%}

namespace tesseract
{
namespace tesseract_planning
{
%nodefaultctor BasicPlanner;
class BasicPlanner
{
public:
		
	virtual ~BasicPlanner();
	
	const std::string& getName() const;
	
	const PlannerRequest& getRequest() const;
	
	void setRequest(const PlannerRequest& request);
	
	const StatusCodeMap& getAvailableStatusCodes();
			
	virtual bool terminate();
	
	virtual void clear();
	
	%extend
	{
		PlannerResponse solve()
		{
			tesseract::tesseract_planning::PlannerResponse res;
			if(!$self->solve(res))
			{
				throw std::runtime_error("Planner solve failed");
			}
			return res;			
		}
		
	}
	
};

}
}