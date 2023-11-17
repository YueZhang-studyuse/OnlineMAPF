#include "mcp.h"


void MCP::simulate(vector<Path*>& paths)
{
    vector<Path> path_copy; 
    path_copy.resize(paths.size());
    copy_agent_time = agent_time;
    copy_mcp = mcp;
    unfinished_agents.clear();
    for (int i = 0; i < paths.size(); i++)
    {
        if (paths[i]->size() == 0)
            continue;
        unfinished_agents.push_back(i);
        path_copy[i].reserve(paths[i]->size() * 2);
        assert(copy_agent_time[i] > 0);
        
        path_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]));
        
    }
    
    for (int t = 0; !unfinished_agents.empty(); t++) {
        cout<<"Similate t = "<<t<<endl;
        cout<<"Start "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;
        auto old_size = unfinished_agents.size();


        std::__1::vector<int> before = copy_agent_time;
        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();) {
            int i = *p;
            moveAgent(path_copy, paths, p, t);
        }

        cout<<"unfinished: "<< unfinished_agents.size() <<endl;

        bool no_move = true;

        for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();p++) {
            if (copy_agent_time[*p] != before[*p]){
                no_move = false;
                break;
            }
        } 
        if (no_move && !unfinished_agents.empty()){
            cout<<"No move at "<<t<<endl;

            for (auto p = unfinished_agents.begin(); p != unfinished_agents.end();p++) {
                int i = *p;

                int loc = paths[i]->at(no_wait_time[i][copy_agent_time[i]]).location;
                cout<<"Agent "<<i<< "at" << path_copy[i][t].location<<"last "<< paths[i]->back().location<<" for "<<loc;
                for (auto l : copy_mcp[loc]){
                    cout <<" [";
                    for (auto a : l){
                        cout<<a<<",";
                    }
                    cout<<"]";
                }
                cout<<endl;
            }
            _exit(1);
        }       

        
    }

    // for (int i = 0; i < paths.size(); i++)
    // {
    //     if (copy_agent_time[i] != (int) no_wait_time[i].size()){
    //         path_copy[i].insert(path_copy[i].end()
    //         , std::next(paths[i]->begin(),no_wait_time[i][copy_agent_time[i]]), paths[i]->end());
    //     }
    //     *(paths[i]) = path_copy[i];
    // };
    cout<<"Simulation done at "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;


    return;
}

bool MCP::moveAgent(vector<Path>& paths_copy, vector<Path*>& paths, list<int>::iterator& p, int t)
{

    int i = *p;

    // cout <<"work on agent "<<i<<endl;
    if (paths_copy[i].size() == t + 2)  // we have already made the movement decision for the agent
    {
        ++p;
        return false;
    }
    assert(paths_copy[i].size() == t + 1);
    assert(copy_agent_time[i] <= (int) no_wait_time[i].size());
    if (copy_agent_time[i] == (int) no_wait_time[i].size()) // the agent has reached the last location on its path
    {
        cout<<"Agent "<<i<<" reach last " <<endl;
        int loc = paths[i]->back().location;
        if (paths_copy[i][t].location == loc)// the agent has reached its goal location
        {
            assert(copy_mcp[loc].front().count(i)>0);

            if (t <= window_size)
            {
                paths_copy[i].push_back(paths_copy[i].back());
                ++p;
                return false;
            }
            copy_mcp[loc].front().erase(i);
            if (copy_mcp[loc].front().empty())
                copy_mcp[loc].pop_front();
            cout<<"Agent "<<i<<" finished at "<<t << "at" << loc <<endl;
            p = unfinished_agents.erase(p);
        
            return true;
        }
        else 
        {
            assert(false);
        }
    }


    // check mcp to determine whether the agent should move or wait
    int loc = paths[i]->at(no_wait_time[i][copy_agent_time[i]]).location;
    assert(!copy_mcp[loc].empty());

    // if (t <= delay_for[i] && delay_for[i] > 0){
    //     paths_copy[i].push_back(paths_copy[i].back());
    //     ++p;
    //     // cout<< i <<" stop at" << t << "; delay for "<< delay_for[i] <<endl; 
    //     return false;
    // }

    // if t is in the window, 
    if (t <= window_size){
        // cout<<"check if collision at "<< t <<" for "<< i <<endl;
        // cout<<"time "<<(float)clock()/(float)CLOCKS_PER_SEC<<endl;


        //and next location record more than one agent in top of mcp (vertex conflict may occur), then wait
        if (copy_mcp[loc] .front().size() > 1){
            paths_copy[i].push_back(paths_copy[i].back());
            ++p;
            // cout<< i <<" stop at" << t << "to avoid vertex conflict on  "<< loc <<endl; 
            return false;
        }
        //if edge conflict may occur, then wait
        else {
            // cout << "check edge conflict" << endl;
            int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;

            // cout << "get previous" << endl;

            if (
                //if the second agent of next loc include i
                previous != loc &&
                (std::next(copy_mcp[previous].begin()) != copy_mcp[previous].end()) &&
                (std::next(copy_mcp[loc].begin())!= copy_mcp[loc].end() )&&
                ((*std::next(copy_mcp[loc].begin())).count(i) > 0)
            ){
                // cout << "check edge conflict 1" << endl;

                //check if the first agents a of next loc is at loc, and want to move to previous loc (the second agents of previous include a)
                for (auto a : copy_mcp[loc].front()){
                    // cout << "check edge conflict for "<< a << endl;

                    int target = paths[a]->at(no_wait_time[a][copy_agent_time[a]]).location;
                    if ( (target== previous) && (paths_copy[a][t].location == loc) && (*std::next(copy_mcp[previous].begin())).count(a) > 0){
                        paths_copy[i].push_back(paths_copy[i].back());
                        ++p;
                        // cout<< i <<" stop at" << t << "to avoid edge conflict on "<< previous << " - "<< loc <<endl; 
                        return false;
                    }

                }
                //  cout << "check edge conflict done " << endl;


            }
                //  cout << "check edge conflict done2 " << endl;



        }
    }

    // cout<<"No collision, continue"  <<endl;


    if (copy_mcp[loc].front().count(i)>0)
    {
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move
        assert(copy_agent_time[i] > 0);
        
        int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;
        
        if (copy_mcp[previous].front().count(i)==0)
        {
            cout<<"error: previouse location's front has no "<<i<<" :";
            for (auto item : copy_mcp[previous])
            {
                cout<<"[";
                for(auto a : item)
                    cout << a << ", ";
                cout<<"],";
            }
            cout << endl;
        }

        copy_mcp[previous].front().erase(i);
        if (copy_mcp[previous].front().empty())
            copy_mcp[previous].pop_front();
        
        copy_agent_time[i]++;
        ++p;
        return true;
    }
    assert(copy_mcp[loc].size() > 1);

    if ((*std::next(copy_mcp[loc].begin())).count(i) > 0){// the second agent is i
        if (t <= window_size && (*std::next(copy_mcp[loc].begin())).size() > 1){
            paths_copy[i].push_back(paths_copy[i].back()); // stay still
            ++p;
            return false;
        }
        // pretend this agent can move: see whether the first agent can move successfully
        paths_copy[i].push_back(paths[i]->at(no_wait_time[i][copy_agent_time[i]])); // move

        int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;
        assert(copy_mcp[previous].front().count(i) != 0);
        int mcp_pop = false;
        copy_mcp[previous].front().erase(i);
        if (copy_mcp[previous].front().empty()){
            copy_mcp[previous].pop_front();
            mcp_pop = true;
        }
        
        copy_agent_time[i]++;
        

        for (int first_agent:std::set<int>(copy_mcp[loc].front())){

            bool succ = false;
            if ( 
            paths_copy[first_agent].size() == t + 1 &&  // we have not made the movement decision for the first agent
            paths_copy[first_agent][t].location == loc )  // the fist agent is already at loc
            {
                auto p2 = std::find(unfinished_agents.begin(), unfinished_agents.end(), first_agent);
                assert(p2 != unfinished_agents.end());
                
                succ = moveAgent(paths_copy, paths, p2, t);
                
                
            }

            
            if (!succ)
            // this agent cannot move
            {
                paths_copy[i][t + 1] = paths_copy[i].at(t);
                copy_agent_time[i]--;
                int previous = paths[i]->at(no_wait_time[i][copy_agent_time[i] - 1]).location;

                if(mcp_pop){
                    copy_mcp[previous].push_front(set<int>());
                }

                
                copy_mcp[previous].front().insert(i);
                ++p;
                return false;
            }
        }
        ++p;
        return true;

    }
    
    paths_copy[i].push_back(paths_copy[i].back()); // stay still
    ++p; // next agent
    return false;
}


void MCP::build(vector<Path*>& paths)  
{
    cout<<"MCP window size:"<< window_size <<endl;
    //if (options1.debug)
    //    cout << "Start MCP ..." << endl;
    size_t map_size = instance.env->map.size();
    //if (options1.debug)
    //    cout << "map_size: " << map_size << endl;
    mcp.resize(map_size);
    agent_time.resize(paths.size(), 0);
    size_t max_timestep = 0;
    for (int i = 0; i<paths.size();i++)
    {
        if (paths[i]->size() ==0)
            cout<< "error; agent " << i << " has no path" <<endl;
            // _exit(1);
        max_timestep = max(max_timestep, paths[i]->size());
    }

    //if (options1.debug)
    //    cout << "max_timestep = " << max_timestep << endl;

    // Push nodes to MCP
    no_wait_time.resize(paths.size());
    delay_for.resize(paths.size(), 0);
    // std::unordered_map<int,int> goal_arrived;
    // for (int i = 0; i<paths.size();i++)
    // {
    //     if (paths[i]->size()-1 <window_size){
    //         goal_arrived[paths[i]->back().location] = paths[i]->size()-1;
    //     }
    // }

    // cout << "21: ";
    // for (auto p : *(paths[21])){
    //     cout<<p.location<<",";
    // }
    // cout<<endl;

    // cout << "84: ";
    // for (auto p : *(paths[84])){
    //     cout<<p.location<<",";
    // }
    // cout<<endl;


    for (size_t t = 0; t < max_timestep; t++)
    {
        unordered_map<int, set<int>> t_occupy;
        unordered_map<int, set<int>> t_occupy_mcp;
        unordered_map<set<int>, set<int>> t_occupy_edge;

        for (int i = 0; i<paths.size();i++)
        {
            // if (t < paths[i]->size())
            //     t_occupy[paths[i]->at(t).location].insert(i);

            if (t < paths[i]->size())
                // (t==0 || paths[i]->at(t).location != paths[i]->at(t-1).location))
            {
                t_occupy_mcp[paths[i]->at(t).location].insert(i);

                // if (t>0 && paths[i]->at(t).location != paths[i]->at(t-1).location){
                //     set<int> ed = {paths[i]->at(t-1).location, paths[i]->at(t).location};
                //     t_occupy_edge[ed].insert(i);
                // }
                no_wait_time[i].push_back(t);

            //     // if (goal_arrived.count(paths[i]->at(t).location) && t >= goal_arrived.at(paths[i]->at(t).location) ){
            //     //     if (window_size+1 - t > delay_for[i]){
            //     //         delay_for[i] = window_size+1 - t;
            //     //     }
            //     // }
            }
            
        }

        for(auto& o : t_occupy_mcp){
            mcp[o.first].push_back(o.second);
        }

        // for(auto& o : t_occupy){
        //     if (o.second.size() > 1 && t <= window_size)
        //         for (auto a : o.second){
        //             if (window_size+1 - t > delay_for[a]){
        //                 delay_for[a] = window_size+1 - t;
        //                 std::cout<<"delay for "<<a<<" is "<<delay_for[a] <<" with conflict "<< o.first << " at "<< t <<std::endl;
        //             }
        //         }
        // }

        // for(auto& o : t_occupy_edge){
        //     if (o.second.size() > 1 && t <= window_size)
        //         for (auto a : o.second){
        //             if (window_size+1 - t > delay_for[a]){
        //                 delay_for[a] = window_size+1 - t;
        //                std::cout<<"delay for "<<a<<" is "<<delay_for[a] <<" with conflict "<< *(o.first.begin()) <<" - "<< *(o.first.begin()++) << " at "<< t <<std::endl;
        //             }
        //         }
        // }
    }

    //debug print

    // for  (int l = 0; l < map_size; l++){
    //     if (mcp[l].size()== 0)
    //         continue;
    //     cout<< l <<": ";
    //     for (auto o : mcp[l]){
    //         cout<<"[";
    //         for (auto a : o){
    //             cout<<a<<",";
    //         }
    //         cout<<"],";

    //     }
    //     cout<<endl;

    // }

    for (int i = 0; i<paths.size();i++)
    {
        assert(!no_wait_time[i].empty());
        if (!no_wait_time[i].empty() && no_wait_time[i][0] == 0)
        {
            agent_time[i] = 1;
        }
        else{
            assert(false);
        }
    }
}



void MCP::printAll()
{
    cout << "==================== MCP ====================" << endl;
    for (int i = 0; i < mcp.size(); i++)
    {
        if (!mcp[i].empty())
        {
            cout << "[" << i << "]: ";
            auto &last = *(--mcp[i].end());
            for (const auto& p: mcp[i])
            {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
            }
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::print(int loc)
{
    cout << "==================== MCP ====================" << endl;
    if (loc < mcp.size() && !mcp[loc].empty())
    {
        cout << "[" << loc << "]: ";
        auto &last = *(--mcp[loc].end());
        for (const auto& p: mcp[loc])
        {
                cout<<"[";
                for(auto& o: p){
                    cout<<o<<",";
                }
                cout<<"]";
                if (&p != &last)
                    cout << "->";
                else
                    cout << endl;
        }
    }
    cout << "\n================== MCP END ==================" << endl;
}


void MCP::printAgentTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": " << agent_time[i] << endl;
    }
    cout << "================== End Time ==================" << endl;
}


void MCP::printAgentNoWaitTime(int num_agents)
{
    cout << "==================== Time ====================" << endl;
    for (int i = 0; i < num_agents; i++)
    {
        cout << "Agent " << i << ": ";
        for (int t = 0; t < no_wait_time[i].size(); t++)
            cout << no_wait_time[i][t] << ", ";
        cout << endl;
    }
    cout << "================== End Time ==================" << endl;
}