#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Instance.h"
#include <LNS.h>


class MAPFPlanner
{
public:
    SharedEnvironment* env;

	MAPFPlanner(SharedEnvironment* env): env(env){};
    MAPFPlanner(){env = new SharedEnvironment();};
	virtual ~MAPFPlanner(){delete env; delete lns;};


    virtual void initialize(int preprocess_time_limit);

    // return next states for all agents
    virtual void plan(int time_limit, std::vector<Action> & plan);

    int commit = 1;
    int remain_commit = 1;

    Instance instance;
    LNS* lns = nullptr;

    vector<list<int>> commited_paths;
    vector<list<int>> future_paths;
    bool initial_run = true;
};
