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
    //lns->setIterations(0);

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
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(0);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->setRuntimeLimit(1); //1s for initial run
            initial_success = lns->getInitialSolution();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            lns->clearAll("Adaptive");
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            initial_success = lns->getInitialSolution();
        }
        else 
        {
            lns->clearAll("Adaptive");
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(0); //lacam only
            lns->loadPaths(future_paths);
            lns->fixInitialSolutionWithLaCAM();
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
            cout<<"errors"<<endl;
            for (int i = 0; i <commited_paths.size(); i++)
            {
                future_paths[i].push_front(env->curr_states[i].location);
            }
            for (int i = 0; i < env->num_of_agents; i++)
            {
                commited_paths[i].clear();
            }
            return;
        }
    }

    if (algo == mapf_algo::LACAMLNS)
    {
        lns->setRuntimeLimit(time_limit);
        lns->setIterations(MAX_TIMESTEP);

        if (initial_run)
        {
            instance.prepareDummy();
            lns->clearAll("Adaptive");
            lns->setRuntimeLimit(1); //1s for initial run
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
            initial_run = false;
        }
        else if (!initial_success) //restart if current no initional solution
        {
            lns->clearAll("Adaptive");
            cout<<"initial not success"<<endl;
            lns->setRuntimeLimit(time_limit);
            lns->setIterations(MAX_TIMESTEP);
            initial_success = lns->run();
        }
        else 
        {
            lns->clearAll("Adaptive");
            lns->loadPaths(future_paths);
            lns->setRuntimeLimit(time_limit);
            lns->fixInitialSolutionWithLaCAM();
            lns->has_initial_solution = true;
            lns->setIterations(MAX_TIMESTEP); 
            lns->run();
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
            cout<<"errors"<<endl;
            for (int i = 0; i <commited_paths.size(); i++)
            {
                future_paths[i].push_front(env->curr_states[i].location);
            }
            for (int i = 0; i < env->num_of_agents; i++)
            {
                commited_paths[i].clear();
            }
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

