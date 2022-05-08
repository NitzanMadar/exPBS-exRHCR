#pragma once

#include "GICBSNode.h"
#include "SingleAgentICBS.h"
#include "compute_heuristic.h"
#include "agents_loader.h"

//  - include map for depth appearance dictionary
#include <map>

class GICBSSearch
{
public:
	constraint_strategy cons_strategy;
	bool fixed_prior;
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
	typedef boost::heap::fibonacci_heap< GICBSNode*, boost::heap::compare<GICBSNode::compare_node> > heap_open_t;
	// typedef boost::heap::fibonacci_heap< GICBSNode*, boost::heap::compare<GICBSNode::helper_list_compare_node> > heap_open_helper_t;
	//typedef boost::heap::fibonacci_heap< GICBSNode*, boost::heap::compare<GICBSNode::secondary_compare_node> > heap_focal_t;
	//typedef boost::heap::fibonacci_heap< MDDNode*, boost::heap::compare<MDDNode::compare_node> > mdd_open_t;
	//typedef dense_hash_map<GICBSNode*, GICBSNode*, GICBSNode::GICBSNodeHasher, GICBSNode::ecbs_eqnode> hashtable_t;

	heap_open_t open_list;
    // heap_open_helper_t helper_open_list;

	//heap_focal_t focal_list;
	//hashtable_t allNodes_table;
	list<GICBSNode*> allNodes_table;

	bool solution_found;
	int solution_cost;

	// bool NoSolWithExperience = false;

	double focal_w = 1.0;
	double min_f_val;
	double focal_list_threshold;

	const bool* my_map;
	int map_size;
	int num_of_agents;
	const int* actions_offset;
	const int* moves_offset;
	int num_col;
	AgentsLoader al;

	// Priorities:
   // vector<vector<bool>> initial_priorities;
    vector<vector<bool>> trans_priorities;
    vector<vector<bool>> priorities;

    bool is_fallback_used = false;

	uint64_t HL_num_expanded = 0;
	uint64_t HL_num_generated = 0;
	uint64_t LL_num_expanded = 0;
	uint64_t LL_num_generated = 0;

	int SolutionDepth = 0;
	int MaxDepth = 0;
    map <int, int> depth_appearance_map;  // {0: 1, ..., i: how_many_time_we_were_at_depth_i, ...}

	GICBSNode* dummy_start;
	GICBSNode* fallback_node;
	GICBSNode* empty_priority_node;
    bool fallbacked_to_original_pbs = false;  // this helps us also to detect PBS+experience when fallback to original pbs
    bool search_with_experience;
    double experience_strategy;
    bool clean_experience;
    bool first_experience_failed = false;
    int number_of_fallback_upward = 0;
    int HL_DFS_width_limit;
    int window_size = -1; // for windowed-MAPF queries, solve conflicts for window_size steps (-1 means regular MAPF)
    int depth_to_jump_back = -1;

    double fallback_option = 0;  // will be 0 = no fallback, (0,1] = %tail upward fallback, 2 = P_exp1->no experience, 3 = P_exp1->P_exp2->no experience
	vector<vector<PathEntry>*> paths;
	vector<vector<PathEntry>> paths_found_initially;  // contain initial paths found
    int max_breadth = 0;
	vector < SingleAgentICBS* > search_engines;  // used to find (single) agents' paths and mdd
	bool runGICBSSearch();
    void create_trans_priorities(vector<vector<bool>> priorities_matrix, vector<vector<bool>> *trans_priorities);
    bool is_adding_a_higher_than_b_legal(vector<vector<bool>> priorities_matrix, int a_id, int b_id);
	bool findPathForSingleAgent(GICBSNode*  node, int ag, double lowerbound = 0);
	bool generateChild(GICBSNode* child, GICBSNode* curr);

	inline void updatePaths(GICBSNode* curr);

	void findConflicts(GICBSNode& curr);
	int computeCollidingTeams();
	//inline bool updateGICBSNode(GICBSNode* leaf_node, GICBSNode* root_node);
	//inline void updatePaths(GICBSNode* curr, GICBSNode* root_node);
	//void generateChildwithCurrentCost(GICBSNode* n1, const GICBSNode* curr);
	inline int compute_g_val();

	inline int getAgentLocation(int agent_id, size_t timestep);

	void updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight);
	//void updateReservationTable(bool* res_table, bool* res_table_low_prio, int exclude_agent, const GICBSNode &node);
	inline void releaseClosedListNodes();
	inline void releaseOpenListNodes();
	void printPaths() const;
	string printLocationsForNextWindowedMapf(int h) const;
	void printConflicts(const GICBSNode &n) const;
	void printConstraints(const GICBSNode* n) const;
	// GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, const vector < vector< bool > > initial_priorities, const vector < vector< bool > > fallback_priorities, const bool use_experience, const bool use_clean, bool fixed_prior = false);
	GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, const vector < vector< bool > > initial_priorities, const vector < vector< bool > > fallback_priorities, const int experience, const double fallback, const int width_limit, const int window_size_,  bool fixed_prior = false);
	~GICBSSearch();
};

