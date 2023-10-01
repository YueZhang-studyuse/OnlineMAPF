#pragma once
#include"lns_common.h"
#include "SharedEnv.h"


// Currently only works for undirected unweighted 4-nighbor grids
class Instance 
{
public:
	int num_of_cols;
	int num_of_rows;
	int map_size;
	SharedEnvironment* env;
	vector<vector<int>> heuristic;

	void computeAllPair();

	vector<vector<int>> heuristic;

	// enum valid_moves_t { NORTH, EAST, SOUTH, WEST, WAIT_MOVE, MOVE_COUNT };  // MOVE_COUNT is the enum's size

	Instance()=default;
	//Instance(SharedEnvironment* env);

	void initMap(SharedEnvironment* simulate_env);
	bool updateStartGoals();


	//void printAgents() const;
    vector<int> getStarts() const {return start_locations;};
    vector<list<int>> getGoals() const {return goal_location_seqs;};


    inline bool isObstacle(int loc) const { return my_map[loc]; }
    inline bool validMove(int curr, int next) const
    {
        if (next < 0 || next >= map_size)
            return false;
        if (my_map[next])
            return false;
        return getManhattanDistance(curr, next) < 2;
    }
    list<int> getNeighbors(int curr) const;


    inline int linearizeCoordinate(int row, int col) const { return ( this->num_of_cols * row + col); }
    inline int getRowCoordinate(int id) const { return id / this->num_of_cols; }
    inline int getColCoordinate(int id) const { return id % this->num_of_cols; }
    inline pair<int, int> getCoordinate(int id) const { return make_pair(id / this->num_of_cols, id % this->num_of_cols); }
    inline int getCols() const { return num_of_cols; }

    inline int getManhattanDistance(int loc1, int loc2) const
    {
        int loc1_x = getRowCoordinate(loc1);
        int loc1_y = getColCoordinate(loc1);
        int loc2_x = getRowCoordinate(loc2);
        int loc2_y = getColCoordinate(loc2);
        return abs(loc1_x - loc2_x) + abs(loc1_y - loc2_y);
    }

    static inline int getManhattanDistance(const pair<int, int>& loc1, const pair<int, int>& loc2)
    {
        return abs(loc1.first - loc2.first) + abs(loc1.second - loc2.second);
    }

	int getAllpairDistance(int loc1, int loc2) const
	{
		return heuristic[loc1][loc2];
	}

	int getDegree(int loc) const
	{
		assert(loc >= 0 && loc < map_size && !my_map[loc]);
		int degree = 0;
		if (0 < loc - num_of_cols && !my_map[loc - num_of_cols])
			degree++;
		if (loc + num_of_cols < map_size && !my_map[loc + num_of_cols])
			degree++;
		if (loc % num_of_cols > 0 && !my_map[loc - 1])
			degree++;
		if (loc % num_of_cols < num_of_cols - 1 && !my_map[loc + 1])
			degree++;
		return degree;
	}

	int getDefaultNumberOfAgents() const { return num_of_agents; }
    void savePaths(const string & file_name, const vector<Path*>& paths) const;
    //bool validateSolution(const vector<Path*>& paths, int sum_of_costs, int num_of_colliding_pairs) const;
    bool hasCollision(const Path& p1, const Path& p2) const;

	void setStart(int agent, int location){start_locations[agent] = location;}

	void createDummyGoals();

	void computeAllPair();
	
private:
	  // int moves_offset[MOVE_COUNT];
	  vector<bool> my_map;
	  string map_fname;
	  string agent_fname;

	  int num_of_agents;
	  vector<int> start_locations;
	  //vector<int> goal_locations;
	  vector<list<int>> goal_location_seqs;
	  vector<int> dummy_goals;

	  bool nathan_benchmark = true;
	  void printMap() const;
	  void saveMap() const;

	  int dummy_goal_accpetance = 4;

	  // Class  SingleAgentSolver can access private members of Node 
	  friend class SingleAgentSolver;
};

