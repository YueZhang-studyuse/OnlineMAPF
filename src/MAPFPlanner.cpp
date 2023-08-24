#include <MAPFPlanner.h>
#include <random>
#include <LNS.h>
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    remain_commit = commit;
    Instance instance(env->map, env->rows, env->cols, env->num_of_agents);

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = 5;
    pipp_option.winPIBTSoft = true;

    LNS lns(instance, preprocess_time_limit,
                "LACAM",
                "PP",
                "Adaptive",
                8,
                MAX_TIMESTEP,
                true,
                "Adaptive",
                true,
                true,
                0, pipp_option);
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    actions = std::vector<Action>(env->curr_states.size(), Action::WA);
    
    return;
}

