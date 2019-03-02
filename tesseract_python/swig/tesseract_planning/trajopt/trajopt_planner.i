
%{
#include <tesseract_planning/trajopt/trajopt_planner.h>
%}

namespace tesseract
{
namespace tesseract_planning
{
struct TrajOptPlannerConfig
{
	TrajOptPlannerConfig(trajopt::TrajOptProbPtr prob);
	virtual ~TrajOptPlannerConfig();
	
	trajopt::TrajOptProbPtr prob;
	
	sco::BasicTrustRegionSQPParameters params;
};

class TrajOptPlanner : public BasicPlanner
{
public:
	%extend
	{
		PlannerResponse solve(const TrajOptPlannerConfig& config)
		{
			tesseract::tesseract_planning::PlannerResponse res;
			if(!$self->solve(res, config))
			{
				throw std::runtime_error("Planner solve failed");
			}
			return res;			
		}
		
	}	
};

}
}
