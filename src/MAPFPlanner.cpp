#include <MAPFPlanner.h>
#include <random>
#include <LNS.h>
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "lacam2/lacam2.hpp"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    remain_commit = commit;
    // Instance instance(env->map, env->rows, env->cols, env->num_of_agents);

    // LNS lns(instance, preprocess_time_limit,
    //             "LACAM",
    //             "PP",
    //             "Adaptive",
    //             8,
    //             MAX_TIMESTEP,
    //             true,
    //             "Adaptive",
    //             true,
    //             true,0);
    cout<<"init"<<endl;
}


// plan using simple A* that ignores the time dimension
void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    auto start1 = std::chrono::steady_clock::now();
    actions = std::vector<Action>(env->curr_states.size(), Action::WA);

    LACAMInstance ins = LACAMInstance(env);
    string verbose = "";
    auto MT = std::mt19937(0);
    const auto deadline = Deadline((time_limit-1) * 1000);

    const Objective objective = static_cast<Objective>(0);
    const float restart_rate = 0.01;
    const auto solution = solve(ins, verbose, 0, &deadline, &MT, objective, restart_rate);
    auto end1 = std::chrono::steady_clock::now();
    auto diff1 = end1-start1;
    cout<<"lacam solve ends at.."<<std::chrono::duration<double>(diff1).count()<<endl;

    if (solution.empty())
    {
        cout<<"no solution"<<endl;
        return;
    }

    for (int agent = 0; agent < env->num_of_agents; agent++)
    {
        // size_t max_time = solution.size()-1;
        // for (; max_time > 0; max_time--)
        // {
        //     if (solution[max_time][agent]->index != solution[max_time-1][agent]->index)
        //         break;
        // }
        // agents[agent].path.resize(max_time+1);
        // for (size_t t = 0; t <= max_time; t++)
        // {
        //     agents[agent].path[t].location = solution[t][agent]->index;
        // }
        // path_table.insertPath(agents[agent].id, agents[agent].path);
        // soc+=agents[agent].path.size()-1;
        int next_loc = solution[1][agent]->index;
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
    }

    return;
}

