#pragma once
#include "map_loader.h"
#include "agents_loader.h"

#include <boost/heap/fibonacci_heap.hpp>
//#include <google/dense_hash_map>

#include <sparsehash/dense_hash_map> //

#include <list>

#include <cmath> // added to fix "round not declared in this scope" error (ubuntu 20)

using boost::heap::fibonacci_heap;
using boost::heap::compare;
//using namespace std; //

class EPEANode
{
private:
	int num_of_agents;
	int num_of_cols;
	int num_of_rows;
	const int * move_offset;
	vector<double*> my_heuristics;  // this is the precomputed heuristic for this agent

	// constraint table
	list<pair<int, int>> vertex_cons; // locations that are occupied by other agents (the top-left and bottom right locations of the rectangle)
	list<pair<int, int>> edge_cons; // edges that are occupied by other agents
	

public:
	double h;
	double g;

	int index;

	int makespan;
	vector<int> locs;
	vector<int16_t> arrival_time;

	//int time_expanded = -1;
	//int time_generated = -1;
	EPEANode* parent = NULL;

	bool alreadyExpanded = false;
	int targetDeltaF; // Starts at zero, incremented after a node is expanded once. Set on Expand.
	int remainingDeltaF; // Remaining delta F towards targetDeltaF. Reset on Expand.
	int maxDeltaF = 0; // Only computed on demand

	vector<list<pair<int16_t, int16_t>>>* singleAgentDeltaFs; // For each agent and each direction it can go, the effect of that move on F
													// INT16_MAX means this is an illegal move. Only computed on demand.
	vector<vector<int16_t>> fLookup; // Per each agent and delta F, has 1 if that delta F is achievable by moving the agents starting from this one on,
												// 2 if it isn't, and 0 if we don't know yet.
	
	EPEANode(const MapLoader &ml, const AgentsLoader &al, const vector<double*> my_heuristics); // for root node
	EPEANode();

	void deep_copy(const EPEANode &cpy); // copy
	void Update(const EPEANode &cpy); // Update stats, used in duplicate detection.
	void Clear();
	void ClearConstraintTable();
	bool IsValid(int agent_id, int curr_loc, int next_loc);
	//bool IsValid(int agent_id, int curr_loc, int next_loc, const vector<bool>& vertex_cons, const vector<int>& edge_cons);
	void calcSingleAgentDeltaFs();
	bool existsChildForF(int agentNum, int remainingTargetDeltaF);

	void MoveTo(int agent_id, int next_loc);
	vector<vector<int>> GetPlan();
	~EPEANode();

	// the following is used to comapre nodes in the OPEN list
	struct compare_node {
		bool operator()(const EPEANode* n1, const EPEANode* n2) const 
		{
			if(n1->g + n1->h + n1->targetDeltaF == n2->g + n2->h + n2->targetDeltaF)
				return n1->h + n1->targetDeltaF >= n2->h + n2->targetDeltaF;
			return n1->g + n1->h + n1->targetDeltaF > n2->g + n2->h + n2->targetDeltaF;
		}
	}; 

		// The following is used by googledensehash for checking whether two nodes are equal
		// we say that two nodes, s1 and s2, are equal if
		// both are non-NULL and have the same time_expanded (unique)
	struct epea_eqnode {
		bool operator()(const EPEANode* s1, const EPEANode* s2) const {
			if (s1 == s2)
				return true;
			else if (!s1 || !s2 || s1->makespan != s2->makespan)
				return false;
			else
			{
				for (int i = 0; i < (int)s1->locs.size(); i++)
					if (s1->locs[i] != s2->locs[i])
						return false;
				return true;
			}
		}
	};

	// The following is used by googledensehash for generating the hash value of a nodes
	// this is needed because otherwise we'll have to define the specilized template inside std namespace
	struct EPEANodeHasher {
		size_t operator()(const EPEANode* n) const {
			size_t timestep_hash = std::hash<int>()(n->makespan); //     fix bug.. copy "std::" from node.h (size_t loc_hash = std::hash<int>()(n->loc);)
            size_t sum_of_locs = 0;
			for (int i = 0; i < (int)n->locs.size(); i++)
				sum_of_locs += n->locs[i];
			size_t loc_hash = std::hash<int>()(sum_of_locs); // fix bug... copy "std::" from node.h  (The bug: "epea_node.h:106:22: error: reference to ‘hash’ is ambiguous")
            return (loc_hash ^ (timestep_hash << 1));
		}
	};

	typedef fibonacci_heap< EPEANode*, compare<compare_node> >::handle_type open_handle_t;
	open_handle_t open_handle;
	bool in_openlist;
};

