#pragma once

#include "ICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"

class ICBSSearch
{
public:
	constraint_strategy cons_strategy;
	bool lazy_gen;
	double runtime = 0;
	double pre_runtime = 0;
	double runtime_lowlevel;
	double runtime_conflictdetection;
	double runtime_computeh;
	double runtime_listoperation;
	double runtime_updatepaths;
	double runtime_updatecons;
	list<tuple<int, int, int, int, int, int, int>> node_stat;
	//double upper_bound;
	typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::compare_node> > heap_open_t;
	//typedef boost::heap::fibonacci_heap< ICBSNode*, boost::heap::compare<ICBSNode::secondary_compare_node> > heap_focal_t;
	//typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
	//typedef dense_hash_map<ICBSNode*, ICBSNode*, ICBSNode::ICBSNodeHasher, ICBSNode::ecbs_eqnode> hashtable_t;

	heap_open_t open_list;
	//heap_focal_t focal_list;
	//hashtable_t allNodes_table;
	list<ICBSNode*> allNodes_table;

	bool solution_found;
	int solution_cost;

	double focal_w = 1.0;
	double min_f_val;
	//double focal_list_threshold;

	const bool* my_map;
	int map_size;
	int num_of_agents;
	const int* actions_offset;
	const int* moves_offset;
	int num_col;
	AgentsLoader al;

	

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	ICBSNode* dummy_start;

	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found

	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd
	bool runICBSSearch();
	bool findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound = 0);
	bool generateChild(ICBSNode* child, ICBSNode* curr);

	inline void updatePaths(ICBSNode* curr);

	void findConflicts(ICBSNode& curr);
	int computeCollidingTeams();
	//inline bool updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node);
	//inline void updatePaths(ICBSNode* curr, ICBSNode* root_node);
	//void generateChildwithCurrentCost(ICBSNode* n1, const ICBSNode* curr);
	inline int compute_g_val();

	inline int getAgentLocation(int agent_id, size_t timestep);

	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	//void updateReservationTable(bool* res_table, bool* res_table_low_prio, int exclude_agent, const ICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();
	void printPaths() const;
	void printConflicts(const ICBSNode &n) const;
	void printConstraints(const ICBSNode* n) const;
	ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, bool lazy_gen = false);
	~ICBSSearch();
};

