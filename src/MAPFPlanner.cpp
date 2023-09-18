#include <MAPFPlanner.h>
#include <random>
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "lacam2/lacam2.hpp"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    remain_commit = commit;
    instance.initMap(env);
    lns = new LNS(instance, preprocess_time_limit,
                "LACAM",
                "PP",
                "Adaptive",
                8,
                MAX_TIMESTEP,
                true,
                "Adaptive",
                true,
                true,3);
    lns->setIterations(0);
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    bool new_task = instance.updateStartGoals();
    lns->clearAll("Adaptive");
    if (new_task)
        lns->setHasInitialSolution(false);

    //the 1st ver. lns complete restart
    commited_paths.resize(env->num_of_agents);
    future_paths.resize(env->num_of_agents);
    lns->setRuntimeLimit(time_limit);
    //Instance instance(env);
    
    // LNS lns(instance, time_limit - 0.1,
    //             "LACAM",
    //             "PP",
    //             "Adaptive",
    //             8,
    //             MAX_TIMESTEP,
    //             true,
    //             "Adaptive",
    //             true,
    //             true,3);
    // lns.setIterations(0);
    
    if (!initial_run)
    {
        lns->loadPaths(future_paths);
        cout<<"loading"<<endl;
    }
    for (int i = 0; i < env->num_of_agents; i++)
    {
        future_paths[i].clear();
        commited_paths[i].clear();
    }
    bool succ = lns->run();

    actions = std::vector<Action>(env->curr_states.size(), Action::WA);
    if (!succ)
        return;
    lns->commitPath(1,commited_paths,future_paths,true,env->curr_timestep);
    initial_run = false;

    
    // auto start1 = std::chrono::steady_clock::now()

    // LACAMInstance ins = LACAMInstance(env);
    // string verbose = "";
    // auto MT = std::mt19937(0);
    // const auto deadline = Deadline((time_limit-0.1) * 1000);

    // const Objective objective = static_cast<Objective>(0);
    // const float restart_rate = 0.01;
    // const auto solution = solve(ins, verbose, 0, &deadline, &MT, objective, restart_rate);
    // auto end1 = std::chrono::steady_clock::now();
    // auto diff1 = end1-start1;
    // cout<<"lacam solve ends at.."<<std::chrono::duration<double>(diff1).count()<<endl;

    // if (solution.empty())
    // {
    //     cout<<"no solution"<<endl;
    //     return;
    // }

    for (int agent = 0; agent < env->num_of_agents; agent++)
    {
        int next_loc = commited_paths[agent].front();
        int diff = next_loc - env->curr_states[agent].location;
        if (diff == 1)
            actions[agent] = Action::E;
        else if (diff == -1)
            actions[agent] = Action::WE;
        else if (diff > 0)
            actions[agent] = Action::S;
        else if (diff == 0)
            actions[agent] = Action::WA;
        else
            actions[agent] = Action::N;

        if (future_paths[agent].size()==1)
        {
            cout<<"finished"<<endl;
            initial_run = true;
        }
    }

    return;
}

