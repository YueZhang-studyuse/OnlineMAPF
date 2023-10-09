#include <MAPFPlanner.h>
#include <random>
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "lacam2/lacam2.hpp"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    remain_commit = commit;
    instance.initMap(env);
    instance.computeAllPair();
    lns = new LNS(instance, preprocess_time_limit,
                "LACAM",
                "PP",
                "Adaptive",
                8,
                MAX_TIMESTEP,
                true,
                "Adaptive",
                true,
                true,0);
    lns->setIterations(0);

    commited_paths.resize(env->num_of_agents);
    future_paths.resize(env->num_of_agents);
}

void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    //bool new_task = instance.updateStartGoals();
    bool replan_need = false;

    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (future_paths[i].empty())
        {
            replan_need = true;
            break;
        }
        if (future_paths[i].front() != env->curr_states[i].location)
        {
            replan_need = true;
            break;
        }
        bool arrived = false;
        for (auto loc: future_paths[i])
        {
            if (loc == env->goal_locations[i][0].first)
            {
                arrived = true;
                break;
            }
        }
        if (!arrived)
        {
            replan_need = true;
            break;
        }
    }

    lns->clearAll("Adaptive");

    if (replan_need)
    {
        //if (future_paths.empty() || future_paths[0].empty()){
        lns->setHasInitialSolution(false);
        initial_run = true;
    }


    lns->setRuntimeLimit(time_limit);
    
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
    {
        cout<<"not success"<<endl;
        return;
    }
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
            //initial_run = true;
        }
    }

    return;
}

