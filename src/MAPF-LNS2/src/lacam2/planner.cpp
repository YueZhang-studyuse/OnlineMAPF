#include "../../inc/lacam2/planner.hpp"

LNode::LNode(LNode* parent, uint i, Vertex* v)
: who(), where(), depth(parent == nullptr ? 0 : parent->depth + 1)
{
    if (parent != nullptr) 
    {
        who = parent->who;
        who.push_back(i);
        where = parent->where;
        where.push_back(v);
    }
}

uint HNode::HNODE_CNT = 0;

// for high-level
//HNode::HNode(const Config& _C, DistTable& D, HNode* _parent, const uint _g, const uint _h)
HNode::HNode(const Config& _C, const Instance& I, HNode* _parent, const uint _g, const uint _h)
    : C(_C),
      parent(_parent),
      neighbor(),
      g(_g),
      h(_h),
      f(g + h),
      priorities(C.size()),
      order(C.size(), 0),
      search_tree(std::queue<LNode*>())
{
    ++HNODE_CNT;
    search_tree.push(new LNode());
    const auto N = C.size();

    //init reach goal size
    reach_goal.resize(N);

    // update neighbor
    if (parent != nullptr) 
        parent->neighbor.insert(this);

    // set priorities
    if (parent == nullptr) 
    {
        // initialize
        for (uint i = 0; i < N; ++i)
        {
          priorities[i] = (float)i/N;
          reach_goal[i] = false; //do not reach goal at the start stage
        }
        depth = 0;
    } 
    else 
    {
        depth = parent->depth+1;
        curr_time = parent->curr_time + 1;
        // dynamic priorities, akin to PIBT
        for (size_t i = 0; i < N; ++i) 
        { 
            if (parent->reach_goal[i]) //reached goal before
            {
                reach_goal[i] = true;
                num_agent_reached++;
                //priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
                //parent reached goal before, so the current goal is dummy goal
                auto dummy_goal = I.getDummyGoals()[i];
                if (I.getAllpairDistance(dummy_goal, C[i]->index) == 0) //also reached dummy goal
                {
                    priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
                }
            }
            else
            {
                auto goal_index = I.env->goal_locations[i][0].first; //current we only consider plan for the first goal
                if (I.getAllpairDistance(goal_index, C[i]->index) == 0) //current timestep arrive the real goal
                {
                    reach_goal[i] = true;
                    num_agent_reached++;
                    priorities[i] = parent->priorities[i] - (int)parent->priorities[i];
                }
                else //still not arrived
                {
                    priorities[i] = parent->priorities[i] + 1;
                }
            }
        }
    }

  // set order
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(),
            [&](uint i, uint j) { return priorities[i] > priorities[j]; });
}

HNode::~HNode()
{
  while (!search_tree.empty()) {
    delete search_tree.front();
    search_tree.pop();
  }
}

Planner::Planner(const Instance& _instance, const LACAMInstance* _ins, const Deadline* _deadline,
                 std::mt19937* _MT, const int _verbose,
                 const Objective _objective, const float _restart_rate)
    : 
      instance(_instance),
      ins(_ins),
      deadline(_deadline),
      MT(_MT),
      verbose(_verbose),
      objective(_objective),
      RESTART_RATE(_restart_rate),
      N(ins->N),
      V_size(ins->G.size()),
      //D(DistTable(ins)),
      loop_cnt(0),
      C_next(N),
      tie_breakers(V_size, 0),
      A(N, nullptr),
      occupied_now(V_size, nullptr),
      occupied_next(V_size, nullptr)
{
}

Planner::~Planner() {}

Solution Planner::solve(std::string& additional_info)
{
    // setup agents
    for (auto i = 0; i < N; ++i) A[i] = new LACAMAgent(i);

    // setup search
    auto OPEN = std::stack<HNode*>();
    //auto EXPLORED = std::unordered_map<Config, HNode*, ConfigHasher>();
    auto EXPLORED = std::unordered_map<pair<Config,vector<bool>>, HNode*, RConfigHasher, RCEqual>();
    // insert initial node, 'H': high-level node
    auto H_init = new HNode(ins->starts, instance, nullptr, 0, get_h_value(ins->starts));
    OPEN.push(H_init);
    //EXPLORED[H_init->C] = H_init;
    EXPLORED[make_pair(H_init->C, H_init->reach_goal)] = H_init;

    std::vector<Config> solution;
    auto C_new = Config(N, nullptr);  // for new configuration
    HNode* H_goal = nullptr;          // to store goal node
    HNode* curr_best = nullptr;

    // DFS
    while (!OPEN.empty() && !is_expired(deadline)) 
    {
        // do not pop here!
        auto H = OPEN.top();  // high-level node



        // low-level search end search tree refers to constraint tree
        if (H->search_tree.empty()) 
        {
            OPEN.pop();
            continue;
        }


        //record the current best, in case no solution found
        if (curr_best == nullptr)
            curr_best = H;
        else
        {
            if (curr_best->num_agent_reached < H->num_agent_reached)
                curr_best = H;
        }

        // check goal condition -- reach goal once
        //should all reached the current goal and after that reach the dummy goal
        if (H_goal == nullptr) 
        {
            if (H->num_agent_reached == ins->N)
            {
                H_goal = H;
                break;
            }
        }

        // create successors at the low-level search
        auto L = H->search_tree.front();
        H->search_tree.pop();
        expand_lowlevel_tree(H, L); //generate constraint 

        if (is_expired(deadline)) break;


        // create successors at the high-level search
        const auto res = get_new_config(H, L);

        if (is_expired(deadline)) break;

        delete L;  // free
        if (!res) 
        {
            continue;
        }


        vector<bool> reached;
        reached.resize(A.size());
        // create new configuration
        for (auto a : A) 
        {
          C_new[a->id] = a->v_next;
          reached[a->id] = a->reached_goal;
        }

        // check explored list
        //const auto iter = EXPLORED.find(C_new);
        const auto iter = EXPLORED.find(make_pair(C_new,reached));
        if (iter != EXPLORED.end()) 
        {
            // case found
            rewrite(H, iter->second, H_goal, OPEN);
            // re-insert or random-restart
            auto H_insert = (MT != nullptr && get_random_float(MT) >= RESTART_RATE)
                                ? iter->second
                                : H_init;
            if (H_goal == nullptr || H_insert->f < H_goal->f) 
            {
                OPEN.push(H_insert);
            }
        } 
        else 
        {
            // insert new search node
            const auto H_new = new HNode(C_new, instance, H, H->g + get_edge_cost(H->C, C_new), get_h_value(C_new));
            //EXPLORED[H_new->C] = H_new;
            EXPLORED[make_pair(H->C,H->reach_goal)] = H_new;
            if (H_goal == nullptr || H_new->f < H_goal->f) 
            {
                OPEN.push(H_new);
            }
        }
    }

    if (H_goal == nullptr)
    {
      H_goal = curr_best;
    }

    cout<<"num of goal reached "<<H_goal->num_agent_reached<<endl;

    // backtrack
    if (H_goal != nullptr) 
    {
        auto H = H_goal;
        while (H != nullptr) 
        {
            solution.push_back(H->C);
            H = H->parent;
        }
        std::reverse(solution.begin(), solution.end());
    }

    // memory management
    for (auto a : A) delete a;
    for (auto itr : EXPLORED) delete itr.second;

    return solution;
}

void Planner::rewrite(HNode* H_from, HNode* H_to, HNode* H_goal,
                      std::stack<HNode*>& OPEN)
{
    // update neighbors
    H_from->neighbor.insert(H_to);

    // Dijkstra update
    std::queue<HNode*> Q({H_from});  // queue is sufficient
    while (!Q.empty()) 
    {
        auto n_from = Q.front();
        Q.pop();
        for (auto n_to : n_from->neighbor) 
        {
            auto g_val = n_from->g + get_edge_cost(n_from->C, n_to->C);
            if (g_val < n_to->g) 
            {
                if (n_to == H_goal)
                    solver_info(1, "cost update: ", n_to->g, " -> ", g_val);
                n_to->g = g_val;
                n_to->f = n_to->g + n_to->h;
                n_to->parent = n_from;
                Q.push(n_to);
                if (H_goal != nullptr && n_to->f < H_goal->f) OPEN.push(n_to);
            }
        }
    }
}

uint Planner::get_edge_cost(const Config& C1, const Config& C2) //comment because we only use lacam for one shot
{
    return 1; 
}

uint Planner::get_edge_cost(HNode* H_from, HNode* H_to)
{
    return get_edge_cost(H_from->C, H_to->C);
}

uint Planner::get_h_value(const Config& C) //commment due to only need one shot at this moment
{
    uint cost = 0;
    return cost;
}

void Planner::expand_lowlevel_tree(HNode* H, LNode* L)
{
    if (L->depth >= N) return; //prohibt each agent at one depth, so max is the number of agent
    const auto i = H->order[L->depth];
    auto C = H->C[i]->neighbor;
    C.push_back(H->C[i]);
    // randomize
    if (MT != nullptr) std::shuffle(C.begin(), C.end(), *MT); //random the order (neighbor)
    // insert
    for (auto v : C) H->search_tree.push(new LNode(L, i, v));
}

bool Planner::get_new_config(HNode* H, LNode* L)
{
    // setup cache
    for (auto a : A) 
    {
        // clear previous cache
        if (a->v_now != nullptr && occupied_now[a->v_now->id] == a) 
        {
            occupied_now[a->v_now->id] = nullptr;
        }
        if (a->v_next != nullptr) 
        {
            occupied_next[a->v_next->id] = nullptr;
            a->v_next = nullptr;
        }
        //clear previous reached goal flag
        a->reached_goal = false;
        a->curr_timestep = 0;

        // set occupied now
        a->v_now = H->C[a->id];
        a->curr_timestep = H->curr_time;

        if (H->reach_goal[a->id]) a->reached_goal = true;

        if (H->depth <= commit_window || !a->reached_goal) //cannot reach goal before window
        {
            occupied_now[a->v_now->id] = a;
        }
        if (a->reached_goal && H->depth > commit_window)
        {
            a->v_next = a->v_now;
        }
    }

    // add constraints
    for (uint k = 0; k < L->depth; ++k) 
    {
        const auto i = L->who[k];        // agent
        const auto l = L->where[k]->id;  // loc

        // check vertex collision
        if (occupied_next[l] != nullptr) return false;
        // check swap collision
        auto l_pre = H->C[i]->id;
        if (occupied_next[l_pre] != nullptr && occupied_now[l] != nullptr &&
            occupied_next[l_pre]->id == occupied_now[l]->id)
            return false;
        // set occupied_next
        A[i]->v_next = L->where[k];
        
        occupied_next[l] = A[i];

    }

    // perform PIBT
    for (auto k : H->order) 
    {
        if (is_expired(deadline)) return false; //timeout

        auto a = A[k];
        if (a->v_next == nullptr && !funcPIBT(a))
        {
          return false;  // planning failure
        }
    }

    return true;
}

bool Planner::funcPIBT(LACAMAgent* ai)
{
    const auto i = ai->id;
    const auto K = ai->v_now->neighbor.size();

    // get candidates for next locations
    for (auto k = 0; k < K; ++k) 
    {
        auto u = ai->v_now->neighbor[k];
        C_next[i][k] = u;
        if (MT != nullptr)
            tie_breakers[u->id] = get_random_float(MT);  // set tie-breaker
    }
    C_next[i][K] = ai->v_now;

    LACAMAgent* swap_agent = nullptr;


    int goal_loc = ai->reached_goal ? instance.getDummyGoals()[i] : instance.env->goal_locations[i][0].first;
    //sort
    std::sort(C_next[i].begin(), C_next[i].begin() + K + 1,
              [&](Vertex* const v, Vertex* const u) 
              {
                  // return D.get(i, v) + tie_breakers[v->id] <
                  //       D.get(i, u) + tie_breakers[u->id];
                  return instance.getAllpairDistance(goal_loc,v->index) + tie_breakers[v->id] <
                          instance.getAllpairDistance(goal_loc,u->index) + tie_breakers[u->id];
              });

    swap_agent = swap_possible_and_required(ai);

    if (swap_agent != nullptr)
    {
      std::reverse(C_next[i].begin(), C_next[i].begin() + K + 1);
    }
      
    // main operation
    for (auto k = 0; k < K + 1; ++k) 
    {
        auto u = C_next[i][k];


        // avoid vertex conflicts
        if (occupied_next[u->id] != nullptr) 
        {
            continue;
        }


        auto& ak = occupied_now[u->id];

        // avoid swap conflicts
        if (ak != nullptr && ak->v_next == ai->v_now) continue;

        // reserve next location
        occupied_next[u->id] = ai;
        ai->v_next = u;

        // priority inheritance
        if (ak != nullptr && ak != ai && ak->v_next == nullptr && !funcPIBT(ak))
          continue;

        // success to plan next one step
        // pull swap_agent when applicable
        if (k == 0 && swap_agent != nullptr && swap_agent->v_next == nullptr &&
            occupied_next[ai->v_now->id] == nullptr) 
        {
            swap_agent->v_next = ai->v_now;
            occupied_next[swap_agent->v_next->id] = swap_agent;
        }
        return true;
    }

    // failed to secure node
    occupied_next[ai->v_now->id] = ai;
    ai->v_next = ai->v_now;
    return false;
}

LACAMAgent* Planner::swap_possible_and_required(LACAMAgent* ai)
{
    const auto i = ai->id;
    // ai wanna stay at v_now -> no need to swap
    if (C_next[i][0] == ai->v_now) return nullptr;

    // usual swap situation, c.f., case-a, b
    auto aj = occupied_now[C_next[i][0]->id];
    if (aj != nullptr && aj->v_next == nullptr &&
        is_swap_required(ai->id, aj->id, ai->v_now, aj->v_now) &&
        is_swap_possible(aj->v_now, ai->v_now)) 
    {
        return aj;
    }

    // for clear operation, c.f., case-c
    for (auto u : ai->v_now->neighbor) 
    {
        auto ak = occupied_now[u->id];
        if (ak == nullptr || C_next[i][0] == ak->v_now) continue;
        if (is_swap_required(ak->id, ai->id, ai->v_now, C_next[i][0]) &&
            is_swap_possible(C_next[i][0], ai->v_now)) 
        {
            return ak;
        }
    }

    return nullptr;
}

// simulate whether the swap is required
bool Planner::is_swap_required(const uint pusher, const uint puller,
                               Vertex* v_pusher_origin, Vertex* v_puller_origin)
{

    auto pusher_goal = A[pusher]->reached_goal ? instance.getDummyGoals()[pusher] : instance.env->goal_locations[pusher][0].first;
    auto puller_goal = A[puller]->reached_goal ? instance.getDummyGoals()[puller] : instance.env->goal_locations[puller][0].first;
    if (pusher_goal == puller_goal)
    {
        return false; //no need to swap with same goal
    }
    
    auto v_pusher = v_pusher_origin;
    auto v_puller = v_puller_origin;
    int pusher_vpuller = instance.getAllpairDistance(pusher_goal,v_puller->index);
    //A[pusher]->reached_goal ? instance.getAllpairDistance(ins->dummy_goals[pusher]->index, v_puller->index) : D.get(pusher, v_puller);
    int pusher_vpusher = instance.getAllpairDistance(pusher_goal,v_pusher->index);
    //A[pusher]->reached_goal ? instance.getAllpairDistance(ins->dummy_goals[pusher]->index, v_pusher->index) : D.get(pusher, v_pusher);
    int puller_vpuller = instance.getAllpairDistance(puller_goal,v_puller->index);
    //A[puller]->reached_goal ? instance.getAllpairDistance(ins->dummy_goals[puller]->index, v_puller->index) : D.get(puller, v_puller);
    int puller_vpusher = instance.getAllpairDistance(puller_goal,v_pusher->index);
    //A[puller]->reached_goal ? instance.getAllpairDistance(ins->dummy_goals[puller]->index, v_pusher->index) : D.get(puller, v_pusher);


    Vertex* tmp = nullptr;
    //while (D.get(pusher, v_puller) < D.get(pusher, v_pusher)) {
      //while ((!A[pusher]->reached_goal && A[puller]->reached_goal) || (!A[pusher]->reached_goal && !A[puller]->reached_goal && pusher_vpuller < pusher_vpusher)) {
    while ((pusher_vpuller < pusher_vpusher)) 
    {
        auto n = v_puller->neighbor.size();
        // remove agents who need not to move
        for (auto u : v_puller->neighbor) 
        {
            auto a = occupied_now[u->id];
            if (u == v_pusher ||
                (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) 
            {
                --n;
            } 
            else 
            {
                tmp = u;
            }
        }

        if (n >= 2) return false;  // able to swap
        if (n <= 0) break;

        v_pusher = v_puller;
        v_puller = tmp;

        // int pusher_vpuller = instance.getAllpairDistance(pusher_goal,v_puller->index);
        // int pusher_vpusher = instance.getAllpairDistance(pusher_goal,v_pusher->index);
        // int puller_vpuller = instance.getAllpairDistance(puller_goal,v_puller->index);
        // int puller_vpusher = instance.getAllpairDistance(puller_goal,v_pusher->index);
        int pusher_vpuller = instance.getAllpairDistance(pusher_goal,v_puller->index);
        int pusher_vpusher = instance.getAllpairDistance(pusher_goal,v_pusher->index);
        int puller_vpuller = instance.getAllpairDistance(puller_goal,v_puller->index);
        int puller_vpusher = instance.getAllpairDistance(puller_goal,v_pusher->index);
    }

    return (puller_vpusher < puller_vpuller) &&
            (pusher_vpusher == 0 || pusher_vpuller < pusher_vpusher);
}

// simulate whether the swap is possible
bool Planner::is_swap_possible(Vertex* v_pusher_origin, Vertex* v_puller_origin)
{
    auto v_pusher = v_pusher_origin;
    auto v_puller = v_puller_origin;
    Vertex* tmp = nullptr;
    while (v_puller != v_pusher_origin) 
    {   // avoid loop
        auto n = v_puller->neighbor.size();  // count #(possible locations) to pull
        for (auto u : v_puller->neighbor) 
        {
            auto a = occupied_now[u->id];
            if (u == v_pusher ||
                (u->neighbor.size() == 1 && a != nullptr && ins->goals[a->id] == u)) 
            {
                --n;      // pull-impossible with u
            } 
            else 
            {
                tmp = u;  // pull-possible with u
            }
        }
        if (n >= 2) return true;  // able to swap
        if (n <= 0) return false;
        v_pusher = v_puller;
        v_puller = tmp;
    }
    return false;
}

std::ostream& operator<<(std::ostream& os, const Objective obj)
{
    if (obj == OBJ_NONE) 
    {
        os << "none";
    } 
    else if (obj == OBJ_MAKESPAN) 
    {
        os << "makespan";
    } 
    else if (obj == OBJ_SUM_OF_LOSS) 
    {
        os << "sum_of_loss";
    }
    return os;
}