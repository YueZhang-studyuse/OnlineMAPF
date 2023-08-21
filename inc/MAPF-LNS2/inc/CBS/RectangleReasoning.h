#pragma once
#include "MDD.h"

//enum rectangle_strategy { NR, R, RM, DISJOINTR };

class RectangleReasoning
{
public:
	//rectangle_strategy strategy;
	double accumulated_runtime = 0;

	RectangleReasoning(const Instance& instance) : instance(instance) {}

	shared_ptr<CBSConflict> run(const vector<Path*>& paths, int timestep, 
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);


private:
	const Instance& instance;
	shared_ptr<CBSConflict> findRectangleConflictByRM(const vector<Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);
	shared_ptr<CBSConflict> findRectangleConflictByGR(const vector<Path*>& paths, int timestep,
		int a1, int a2, const MDD* mdd1, const MDD* mdd2);

	bool ExtractBarriers(const MDD& mdd, int loc, int timestep, int dir, int dir2, int start, int goal, int start_time, list<CBSConstraint>& B);
	bool isEntryBarrier(const CBSConstraint& b1, const CBSConstraint& b2, int dir1);
	bool isExitBarrier(const CBSConstraint& b1, const CBSConstraint& b2, int dir1);
	pair<int, int> getIntersection(const CBSConstraint& b1, const CBSConstraint& b2);
	bool blockedNodes(const vector<PathEntry>& path,
		const pair<int, int>& Rs, const pair<int, int>& Rg, int Rg_t, int dir);
	bool isCut(const CBSConstraint b, const pair<int, int>& Rs, const pair<int, int>& Rg);

	void generalizedRectangle(const vector<PathEntry>& path1, const vector<PathEntry>& path2, const MDD& mdd1, const MDD& mdd2,
		const list<CBSConstraint>& B1, const list<CBSConstraint>& B2, int timestep,
		int& best_type, pair<int, int>& best_Rs, pair<int, int>& best_Rg);

	//Identify rectangle conflicts
	bool isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
		const pair<int, int>& g1, const pair<int, int>& g2, int g1_t, int g2_t);// for CR and R
	bool isRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2) const;// for RM

	//Classify rectangle conflicts
	int classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2,
		const pair<int, int>& g1, const pair<int, int>& g2);// for CR and R
	int classifyRectangleConflict(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1, const pair<int, int>& g2, const pair<int, int>& Rg);// for RM

	 //Compute rectangle corners
	pair<int, int> getRg(const pair<int, int>& s1, const pair<int, int>& g1, const pair<int, int>& g2);
	pair<int, int> getRs(const pair<int, int>& s1, const pair<int, int>& s2, const pair<int, int>& g1);

	//Compute start and goal candidates for RM
	list<int> getStartCandidates(const Path& path, const MDD& mdd, int timestep);
	list<int> getGoalCandidates(const Path& path, const MDD& mdd, int timestep);
	//Compute start and goal candidates for GR
	int getStartCandidate(const Path& path, int dir1, int dir2, int timestep);
	int getGoalCandidate(const Path& path, int dir1, int dir2, int timestep);


	// int getRectangleTime(const CBSConflict& conflict, const std::vector<std::vector<PathEntry>*>& paths, int num_col);
	bool hasNodeOnBarrier(const MDD* mdd, int y_start, int y_end, int x, int t_min, bool horizontal) const;

	bool addModifiedBarrierConstraints(int a1, int a2, const pair<int, int>& Rs, const pair<int, int>& Rg,
		const pair<int, int>& s1, const pair<int, int>& s2, int Rg_t,
		const MDD* mdd1, const MDD* mdd2,
		list<CBSConstraint>& constraint1, list<CBSConstraint>& constraint2); // for RM

	// add a horizontal modified barrier constraint
	bool addModifiedHorizontalBarrierConstraint(int agent, const MDD* mdd, int x,
		int Ri_y, int Rg_y, int Rg_t, list<CBSConstraint>& constraints);

	// add a vertival modified barrier constraint
	bool addModifiedVerticalBarrierConstraint(int agent, const MDD* mdd, int y,
		int Ri_x, int Rg_x, int Rg_t, list<CBSConstraint>& constraints);

	bool blocked(const Path& path, const list<CBSConstraint>& constraints);
	bool traverse(const Path& path, int loc, int t);

};

