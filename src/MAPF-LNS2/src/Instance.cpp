#include<boost/tokenizer.hpp>
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include"Instance.h"


Instance::Instance(SharedEnvironment* env)
{
    my_env = env;
	num_of_rows = env->rows;
	num_of_cols = env->cols;
	num_of_agents = env->num_of_agents;
	map_size = env->map.size();
    my_map.resize(map_size);
	//read map to my_map
	for (int i = 0; i < map_size; i++)
	{
		my_map[i] = (env->map[i] == 1);
 	}

    start_locations.resize(num_of_agents);
	goal_locations.resize(num_of_agents);

    for (int i = 0; i < num_of_agents; i++)
    {
        start_locations[i] = env->curr_states[i].location;
        goal_locations[i] = env->goal_locations[i][0].first;
        cout<<"start  "<<my_env->curr_states[i].location<<"  end   "<<my_env->goal_locations[i][0].first<<endl;
    }
}


void Instance::printMap() const
{
	for (int i = 0; i< num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (this->my_map[linearizeCoordinate(i, j)])
				cout << '@';
			else
				cout << '.';
		}
		cout << endl;
	}
}


void Instance::saveMap() const
{
	ofstream myfile;
	myfile.open(map_fname);
	if (!myfile.is_open())
	{
		cout << "Fail to save the map to " << map_fname << endl;
		return;
	}
	myfile << num_of_rows << "," << num_of_cols << endl;
	for (int i = 0; i < num_of_rows; i++)
	{
		for (int j = 0; j < num_of_cols; j++)
		{
			if (my_map[linearizeCoordinate(i, j)])
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}



void Instance::printAgents() const
{
  for (int i = 0; i < num_of_agents; i++) 
  {
    cout << "Agent" << i << " : S=(" << getRowCoordinate(start_locations[i]) << "," << getColCoordinate(start_locations[i]) 
				<< ") ; G=(" << getRowCoordinate(goal_locations[i]) << "," << getColCoordinate(goal_locations[i]) << ")" << endl;
  }
}


list<int> Instance::getNeighbors(int curr) const
{
	list<int> neighbors;
	int candidates[4] = {curr + 1, curr - 1, curr + num_of_cols, curr - num_of_cols};
	for (int next : candidates)
	{
		if (validMove(curr, next))
			neighbors.emplace_back(next);
	}
	return neighbors;
}

void Instance::savePaths(const string & file_name, const vector<Path*>& paths) const
{
    std::ofstream output;
    output.open(file_name);

    for (auto i = 0; i < paths.size(); i++)
    {
        output << "Agent " << i << ":";
        for (const auto &state : (*paths[i]))
        {
            if (nathan_benchmark)
                output << "(" << getColCoordinate(state.location) << "," << getRowCoordinate(state.location) << ")->";
            else
                output << "(" << getRowCoordinate(state.location) << "," << getColCoordinate(state.location) << ")->";
        }
        output << endl;
    }
    output.close();
}

bool Instance::validateSolution(const vector<Path*>& paths, int sum_of_costs, int num_of_colliding_pairs) const
{
    cout << "Validate solution ..." << endl;
    if (paths.size() != start_locations.size())
    {
        cerr << "We have " << paths.size() << " for " << start_locations.size() << " agents." << endl;
        exit(-1);
    }
    int sum = 0;
    for (auto i = 0; i < start_locations.size(); i++)
    {
        if (paths[i] == nullptr or paths[i]->empty())
        {
            cerr << "No path for agent " << i << endl;
            exit(-1);
        }
        else if (start_locations[i] != paths[i]->front().location)
        {
            cerr << "The path of agent " << i << " starts from location " << paths[i]->front().location
                 << ", which is different from its start location " << start_locations[i] << endl;
            exit(-1);
        }
        else if (goal_locations[i] != paths[i]->back().location)
        {
            cerr << "The path of agent " << i << " ends at location " << paths[i]->back().location
                 << ", which is different from its goal location " << goal_locations[i] << endl;
            exit(-1);
        }
        for (int t = 1; t < (int) paths[i]->size(); t++ )
        {
            if (!validMove(paths[i]->at(t - 1).location, paths[i]->at(t).location))
            {
                cerr << "The path of agent " << i << " jumps from "
                     << paths[i]->at(t - 1).location << " to " << paths[i]->at(t).location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                exit(-1);
            }
        }
        sum += (int) paths[i]->size() - 1;
    }
    if (sum_of_costs != sum)
    {
        cerr << "The computed sum of costs " << sum_of_costs <<
             " is different from that of the solution " << sum << endl;
        exit(-1);
    }
    // check for colliions
    int collisions = 0;
    for (auto i = 0; i < start_locations.size(); i++)
    {
        for (auto j = i + 1; j < start_locations.size(); j++)
        {
            bool found_collision = false;
            const auto a1 = paths[i]->size() <= paths[j]->size()? i : j;
            const auto a2 = paths[i]->size() <= paths[j]->size()? j : i;
            int t = 1;
            for (; t < (int) paths[a1]->size(); t++)
            {
                if (paths[a1]->at(t).location == paths[a2]->at(t).location) // vertex conflict
                {
                    if (num_of_colliding_pairs == 0)
                    {
                        cerr << "Find a vertex conflict between agents " << a1 << " and " << a2 <<
                             " at location " << paths[a1]->at(t).location << " at timestep " << t << endl;
                        exit(-1);
                    }
                    collisions++;
                    found_collision = true;
                    break;
                }
                else if (paths[a1]->at(t).location == paths[a2]->at(t-1).location &&
                        paths[a1]->at(t-1).location == paths[a2]->at(t).location) // edge conflict
                {
                    if (num_of_colliding_pairs == 0)
                    {
                        cerr << "Find an edge conflict between agents " << a1 << " and " << a2 <<
                             " at edge (" << paths[a1]->at(t-1).location << "," << paths[a1]->at(t).location <<
                             ") at timestep " << t << endl;
                        exit(-1);
                    }
                    collisions++;
                    found_collision = true;
                    break;
                }
            }
            if (!found_collision)
            {
                auto target = paths[a1]->back().location;
                for (; t < (int) paths[a2]->size(); t++)
                {
                    if (paths[a2]->at(t).location == target)  // target conflict
                    {
                        if (num_of_colliding_pairs == 0)
                        {
                            cerr << "Find a target conflict where agent " << a2 << " (of length " << paths[a2]->size() - 1 <<
                                 ") traverses agent " << a1 << " (of length " << paths[a1]->size() - 1<<
                                 ")'s target location " << target << " at timestep " << t << endl;
                            exit(-1);
                        }
                        collisions++;
                        break;
                    }
                }
            }
        }
    }
    if (collisions != num_of_colliding_pairs)
    {
        cerr << "The computed number of colliding pairs " << num_of_colliding_pairs <<
             " is different from that of the solution " << collisions << endl;
        exit(-1);
    }
    cout << "Done!" << endl;
    return true;
}

bool Instance::hasCollision(const Path& p1, const Path& p2) const
{
    int t = 1;
    for (; t < (int) min(p1.size(), p2.size()); t++)
    {
        if (p1[t].location == p2[t].location) // vertex conflict
        {
            return true;
        }
        else if (p1[t].location == p2[t-1].location and p1[t-1].location == p2[t].location) // edge conflict
        {
            return true;
        }
    }
    if (p1.size() == p2.size()) return false;

    auto p = p1.size() > p2.size()? p1 : p2;
    auto target = p1.size() < p2.size()? p1.back().location : p2.back().location;
    for (; t < (int) p.size(); t++)
    {
        if (p[t].location == target)  // target conflict
        {
            return true;
        }
    }
    return false;
}