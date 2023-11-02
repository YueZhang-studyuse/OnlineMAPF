#include <MAPFPlanner.h>
#include <random>
#include "lacam2/lacam2.hpp"


void MAPFPlanner::initialize(int preprocess_time_limit)
{
    remain_commit = 0;
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

    lns->commit = commit;
}

void MAPFPlanner::plan(int time_limit,vector<Action> & actions) 
{
    if (!commited_paths[0].empty())
    {
        //trans to actions
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
            commited_paths[agent].pop_front();
        }
        cout<<"commit directly"<<endl;
        return;
    }

    if (algo == mapf_algo::LACAM)
    {
        lns->clearAll("Adaptive");
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(0);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->setRuntimeLimit(1); //1s for initial run
            initial_success = lns->run();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            initial_success = lns->run();
        }
        else 
        {
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
            if (replan_need)
            {
                lns->setRuntimeLimit(time_limit);
                lns->setIterations(0); //lacam only
                initial_success = lns->run();
            }
            else
            {
                lns->loadPaths(future_paths);
            }
        }

        for (int i = 0; i < env->num_of_agents; i++)
        {
            future_paths[i].clear();
            commited_paths[i].clear();
        }

        actions = std::vector<Action>(env->curr_states.size(), Action::WA);

        lns->commitPath(commit,commited_paths,future_paths,true,env->curr_timestep);
        if (!lns->validateCommitSolution(commited_paths)) //current window has collisions
        {
            for (int i = 0; i <commited_paths.size(); i++)
            {
                future_paths[i].push_front(env->curr_states[i].location);
            }
            commited_paths.clear();
            return;
        }
    }


    //trans to actions
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
        commited_paths[agent].pop_front();
    }

    return;
}

