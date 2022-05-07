#include "ecbs_search.h"
#include <exception>
#include <iostream>
#include <utility>
#include <list>
#include <vector>
#include <tuple>
#include <ctime>
#include <climits>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>



namespace pt = boost::property_tree;


double theta(MapLoader::orientation_t orientation) {
  double theta = 0;
  switch (orientation) {
  case MapLoader::FACE_EAST:
    theta = 0;
    break;
  case MapLoader::FACE_WEST:
    theta = M_PI;
    break;
  case MapLoader::FACE_NORTH:
    theta = M_PI/2.0f;
    break;
  case MapLoader::FACE_SOUTH:
    theta = 1.5f * M_PI;
    break;
  default:
    break;
  }
  return theta;
}

void ECBSSearch::printPaths(const MapLoader& ml, const ECBSNode & node) {
  for (size_t i = 0; i < paths.size(); i++) {
    cout << "AGENT " << i << " Path: ";
    for (auto& entry : *paths[i]) {
      std::cout << "(" << entry.location / ml.cols << "," << entry.location  % ml.cols << ")->";
    }
    cout << endl;
  }



    {
      using namespace pt;

      ptree pt;
      ptree agents;
      for (size_t ag = 0; ag < paths.size(); ag++) {
        ptree agent;
        std::stringstream sstr;
        sstr << "create" << ag + 1;
        agent.put("name", sstr.str());
        ptree path;
        size_t t = 0;
        for (auto& entry : *paths[ag]) {
          ptree pathentry;
          pathentry.put("x", ml.col_coordinate(entry.location));
          pathentry.put("y", ml.row_coordinate(entry.location));
          pathentry.put("theta", theta(entry.orientation));
          pathentry.put("arrival", t);

          path.push_back(std::make_pair("", pathentry));
          ++t;
        }
        agent.add_child("path", path);
        agents.push_back(std::make_pair("", agent));
      }
      pt.add_child("agents", agents);
      write_json("schedule_discrete.json", pt);

    }


}

void ECBSSearch::printResTable(bool* res_table, size_t max_plan_len) {
  cout << "MAP_SIZE=" << map_size << " ; MAX_PLAN_LEN=" << max_plan_len << endl;
  for (size_t t = 0; t < max_plan_len; t++) {
    for (size_t id = 0; id < (size_t)map_size; id++) {
      if ( res_table[id + t*map_size] == false )
        cout << '_';
      else
        cout << '*';
    }
    cout << endl;
  }
}


inline void ECBSSearch::releaseClosedListNodes() {
  //hashtable_t::iterator it;
  //for (it=allNodes_table.begin(); it != allNodes_table.end(); it++) {
  //  delete ( (*it).first );  // should it be .second?
  //}
	for (list<ECBSNode*>::iterator  it = allNodes_table.begin(); it != allNodes_table.end(); it++) 
		delete *it;
}


// computes g_val based on current paths (WRONG!)
inline double ECBSSearch::compute_g_val() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += paths[i]->size();
  return retVal;
}


// computes High-Level lower-bound based
inline double ECBSSearch::compute_hl_lower_bound() {
  double retVal = 0;
  for (int i = 0; i < num_of_agents; i++)
    retVal += ll_min_f_vals[i];
  return retVal;
}


// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ECBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) {
  for (ECBSNode* n : open_list) {
    if ( n->sum_min_f_vals > old_lower_bound &&
         n->sum_min_f_vals <= new_lower_bound )
      n->focal_handle = focal_list.push(n);
  }
}


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ECBSSearch::updatePaths(ECBSNode* curr, ECBSNode* root_node) {
  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;
  vector<bool> updated(num_of_agents, false);  // initialized for false
  /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
   * because younger nodes take into account ancesstors' nodes constraints. */
  while ( curr != root_node ) 
  {
	  for (list<tuple<int, vector<pathEntry>, double, double>>::iterator it = curr->paths_updated.begin();
			it != curr->paths_updated.end(); it++)
	  {
		  if (!updated[get<0>(*it)])
		  {
			  paths[get<0>(*it)] = &(get<1>(*it));
			  paths_costs[get<0>(*it)] = get<2>(*it);
			  ll_min_f_vals[get<0>(*it)] = get<3>(*it);
			  updated[get<0>(*it)] = true;
		  }
	}
    curr = curr->parent;
  }
}


// Used in the GUI
void ECBSSearch::updatePathsForExpTime(int t_exp) {
  if (t_exp > (int)HL_num_expanded || t_exp < 0)
    return;  // do nothing if there's no high-level node for the specified time_expanded

  ECBSNode* t_exp_node = NULL;
 /* for (hashtable_t::iterator it=allNodes_table.begin(); it != allNodes_table.end(); it++)
    if ( ((*it).second)->time_expanded == t_exp )
      t_exp_node = (*it).second;*/
  for (list<ECBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
	  if ((*it)->time_expanded == t_exp)
		  t_exp_node = (*it);

  updatePaths(t_exp_node, dummy_start);
  //  printPaths();
}

vector < list< tuple<int, int, bool> > >* ECBSSearch::collectConstraints(ECBSNode* curr, int agent_id)
{
	// extract all constraints on leaf_node->agent_id
	list < tuple<int, int, int, bool> > constraints_positive;
	list < tuple<int, int, int, bool> > constraints_negative;
	//  cout << "  Find all constraints on him:" << endl;
	int max_timestep = -1;
	while (curr != dummy_start)
	{
		if (get<3>(curr->constraint)) // positive constraint is valid for everyone
		{
			if (curr->agent_id == agent_id) // for the constrained agent, it is a landmark
				constraints_positive.push_back(curr->constraint);
			else // for the other agents, it is equalvelent to a negative constraint
				constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		else if (curr->agent_id == agent_id)
		{
			constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		curr = curr->parent;
	}
	// cout << "  OVERALL #CONS:" << constraints.size() << endl;   
	// cout << "  Latest constraint's timestep:" << max_timestep << endl;

	// initialize a constraint vector of length max_timestep+1. Each entry is an empty list< pair<int,int> > (loc1,loc2)
	//  cout << "  Creating a list of constraints (per timestep):" << endl;
	vector < list< tuple<int, int, bool> > >* cons_vec = new vector < list< tuple<int, int, bool> > >(max_timestep + 1, list< tuple<int, int, bool> >());
	for (list< tuple<int, int, int, bool> >::iterator it = constraints_positive.begin(); it != constraints_positive.end(); it++) {
		//    cout << "   PUSHING a positive constraint for time:" << get<2>(*it) << " ; (constraint is [" << get<0>(*it) << "," << get<1>(*it) << "])" << endl;
		if (get<1>(*it) < 0) // vertex constraint
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), -1, true));
		else // edge constraint
		{
			cons_vec->at(get<2>(*it) - 1).push_back(make_tuple(get<0>(*it), -1, true));
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), -1, true));
		}
	}
	for (list< tuple<int, int, int, bool> >::iterator it = constraints_negative.begin(); it != constraints_negative.end(); it++) {
		//    cout << "   PUSHING a negative constraint for time:" << get<2>(*it) << " ; (constraint is [" << get<0>(*it) << "," << get<1>(*it) << "])" << endl;
		cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), get<1>(*it), false));
	}
	return cons_vec;
}




/*
  return agent_id's location for the given timestep
  Note -- if timestep is longer than its plan length,
          then the location remains the same as its last cell)
 */
inline int ECBSSearch::getAgentLocation(int agent_id, size_t timestep) {
  // if last timestep > plan length, agent remains in its last location
  if (timestep >= paths[agent_id]->size())
    return paths[agent_id]->at(paths[agent_id]->size()-1).location;
  // otherwise, return its location for that timestep
  return paths[agent_id]->at(timestep).location;
}

/*
  return true iff agent1 and agent2 switched locations at timestep [t,t+1]
 */
inline bool ECBSSearch::switchedLocations(int agent1_id, int agent2_id, size_t timestep) {
  // if both agents at their goal, they are done moving (cannot switch places)
  if ( timestep >= paths[agent1_id]->size() && timestep >= paths[agent2_id]->size() )
    return false;
  if ( getAgentLocation(agent1_id, timestep) == getAgentLocation(agent2_id, timestep+1) &&
       getAgentLocation(agent1_id, timestep+1) == getAgentLocation(agent2_id, timestep) )
    return true;
  return false;
}


vector< tuple<int, int, int, int, int> >* ECBSSearch::extractCollisions() {
  vector< tuple<int, int, int, int, int> >* cons_found = new vector< tuple<int, int, int, int, int> >();
  earliest_conflict = make_tuple(-1, -1, -1, -1, INT_MAX);
  int a2 = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
		  int loc1 = getAgentLocation(a1, timestep);
		  int loc2 = getAgentLocation(a2, timestep);
        if ( loc1 == loc2 ) {
          cons_found->push_back(make_tuple(a1, a2, loc1, loc2, timestep));  // vertex collision
//#ifndef NDEBUG
//		  cout << "returning: " << loc1 << ", " << loc2 << endl;
//#endif
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, loc1, loc2, timestep);
        }
        if ( switchedLocations(a1, a2, timestep) ) {
          cons_found->push_back(make_tuple(a1, a2, loc1,  -loc2 - 1,  timestep) );
          if ((int)timestep < std::get<4>(earliest_conflict))
            earliest_conflict = make_tuple(a1, a2, loc1, -loc2 - 1, timestep);
        }
      }
    }
  }
  return cons_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Returns the maximal path length (among all agent)
size_t ECBSSearch::getPathsMaxLength() {
  size_t retVal = 0;
  for (int ag = 0; ag < num_of_agents; ag++)
    if ( paths[ag] != NULL && paths[ag]->size() > retVal )
      retVal = paths[ag]->size();
  return retVal;
}

// Generates a boolean reservation table for paths (cube of map_size*max_timestep).
// This is used by the low-level ECBS to count possible collisions efficiently
// Note -- we do not include the agent for which we are about to plan for
void ECBSSearch::updateReservationTable(bool* res_table, size_t max_path_len, int exclude_agent) {
  for (int ag = 0; ag < num_of_agents; ag++) {
    if (ag != exclude_agent && paths[ag] != NULL) {
      for (size_t timestep = 0; timestep < max_path_len; timestep++) {
        int id = getAgentLocation(ag, timestep);
		res_table[timestep * map_size + id] = true;
      }
    }
  }
}

// Compute the number of pairs of agents colliding (h_3 in ECBS's paper)
int ECBSSearch::computeNumOfCollidingAgents() {
  //  cout << "   *-*-* Computed number of colliding agents: " << endl;
  int retVal = 0;
  for (int a1 = 0; a1 < num_of_agents; a1++) {
    for (int a2 = a1+1; a2 < num_of_agents; a2++) {
      size_t max_path_length = paths[a1]->size() > paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
      for (size_t timestep = 0; timestep < max_path_length; timestep++) {
        //        cout << "   A1:" << getAgentLocation(a1, timestep) << ", A2:" <<  getAgentLocation(a2, timestep) << ", T:" << timestep;
        if ( getAgentLocation(a1, timestep) == getAgentLocation(a2, timestep) ||
             switchedLocations(a1, a2, timestep) ) {
          retVal++;
          // break to the outer (a1) loop
          timestep = max_path_length;
          a2 = num_of_agents;
          //          cout << " !BOOM! ";
        }
      }
    }
  }
  //  cout << "   *-*-* Computed number of colliding agents returns: " << retVal << endl;
  return retVal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
ECBSSearch::ECBSSearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader& egr, double e_w, double f_w, bool tweak_g_val, constraint_strategy c) {
	cons_strategy = c;
  focal_w = f_w;
  HL_num_expanded = 0;
  HL_num_generated = 0;
  LL_num_expanded = 0;
  LL_num_generated = 0;
  //num_useful_constraints = 0;
  this->num_col = ml.cols;
  this->al = al;
  num_of_agents = al.num_of_agents;
  map_size = ml.rows*ml.cols;
  solution_found = false;
  solution_cost = -1;
  ll_min_f_vals = vector <double> (num_of_agents);
  paths_costs = vector <double> (num_of_agents);
  ll_min_f_vals_found_initially = vector <double> (num_of_agents);
  paths_costs_found_initially = vector <double> (num_of_agents);
  search_engines = vector < SingleAgentECBS* > (num_of_agents);
  for (int i = 0; i < num_of_agents; i++) {
    int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
    int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
    ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset, ml.actions_offset, e_w, &egr);
    search_engines[i] = new SingleAgentECBS(init_loc, goal_loc, MapLoader::orientation_t::FACE_EAST,
                                            ch.getHVals(), ch.getEstimatedGVals(),
                                            ml.get_map(), ml.rows*ml.cols,
                                            ml.moves_offset, ml.actions_offset,
                                            &egr,
                                            e_w,
                                            tweak_g_val,
											ml.cols);
  }

  // initialize allNodes_table (hash table)
  empty_node = new ECBSNode();
  empty_node->time_generated = -2; empty_node->agent_id = -2;
  deleted_node = new ECBSNode();
  deleted_node->time_generated = -3; deleted_node->agent_id = -3;
  //allNodes_table.set_empty_key(empty_node);
  //allNodes_table.set_deleted_key(deleted_node);

  // initialize all initial paths to NULL
  paths_found_initially.resize(num_of_agents);
  for (int ag=0; ag < num_of_agents; ag++)
    paths_found_initially[ag] = NULL;

  // initialize paths_found_initially
  for (int i = 0; i < num_of_agents; i++) {
    //    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
    paths = paths_found_initially;
    size_t max_plan_len = getPathsMaxLength();
    bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
    updateReservationTable(res_table, max_plan_len, i);
    //    cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
    //    printResTable(res_table, max_plan_len);
    if ( search_engines[i]->findPath ( f_w, NULL, res_table, max_plan_len ) == false)
      cout << "NO SOLUTION EXISTS";
    paths_found_initially[i] = new vector<pathEntry> (search_engines[i]->getPath());
    ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
    paths_costs_found_initially[i] = search_engines[i]->path_cost;
    LL_num_expanded += search_engines[i]->num_expanded;
    LL_num_generated += search_engines[i]->num_generated;
    delete[] res_table;
    //    cout << endl;
  }

  paths = paths_found_initially;
  ll_min_f_vals = ll_min_f_vals_found_initially;
  paths_costs = paths_costs_found_initially;

  // generate dummy start and update data structures
  dummy_start = new ECBSNode();
  dummy_start->agent_id = -1;
  dummy_start->g_val = 0;
  for (int i = 0; i < num_of_agents; i++)
    dummy_start->g_val += paths_costs[i];
  dummy_start->sum_min_f_vals = compute_hl_lower_bound();
  dummy_start->open_handle = open_list.push(dummy_start);
  dummy_start->focal_handle = focal_list.push(dummy_start);
  HL_num_generated++;
  dummy_start->time_generated = HL_num_generated;
  //allNodes_table[dummy_start] = dummy_start;
  allNodes_table.push_back(dummy_start);

  min_sum_f_vals = dummy_start->sum_min_f_vals;
  focal_list_threshold = focal_w * dummy_start->sum_min_f_vals;

  //  cout << "Paths in START (high-level) node:" << endl;
  //  printPaths();
  // cout << "SUM-MIN-F-VALS: " << dummy_start->sum_min_f_vals << endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::runECBSSearch() {
	switch (cons_strategy)
	{
	case constraint_strategy::ECBS:
		cout << "      ECBS: ";
		break;
	case constraint_strategy::N_ECBS:
		cout << "    N-ECBS: ";
		break;
	default:
		break;
	}
  // set timer
  std::clock_t start;
  start = std::clock();

  // start is already in the open_list
  while ( !focal_list.empty() && !solution_found ) {
		// break after 5 min
		runtime = (std::clock() - start);// / (double) CLOCKS_PER_SEC;
		if (runtime > TIME_LIMIT) {  // timeout after 5 minutes
		  cout << "TIMEOUT  ; " << solution_cost << " ; " << min_sum_f_vals - dummy_start->g_val << " ; " <<
			  HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			  LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " <<
			  /*num_useful_constraints * 1.0 / HL_num_expanded <<*/ endl;
		  return false;
		}

		ECBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		//    cout << "Expanding: (" << curr << ")" << *curr << " at time:" << HL_num_expanded << endl;

		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		updatePaths(curr, dummy_start);
		//    printPaths();

		vector< tuple<int, int, int, int, int> >* collision_vec = extractCollisions();  // check for collisions on updated paths
	//#ifndef NDEBUG
	//    cout << endl << "****** Expanded #" << curr->time_expanded << " with cost " << curr->g_val << " and # Collisions " << collision_vec->size() << " and |FOCAL|=" << focal_list.size() << " and focal-threshold=" << focal_list_threshold << endl;
	//#endif
		/*
		cout << "Collision found in the expanded node's paths:" << endl;
		for (vector< tuple<int,int,int,int,int> >::const_iterator it = collision_vec->begin(); it != collision_vec->end(); it++)
		  cout << "   A1:" << get<0>(*it) << " ; A2:" << get<1>(*it) << " ; L1:" << get<2>(*it) << " ; L2:" << get<3>(*it) << " ; T:" << get<4>(*it) << endl;
		cout << "Overall Col_Vec.size=" << collision_vec->size() << endl;
		 */

		if ( collision_vec->size() == 0 ) 
		{  // found a solution (and finish the while look)
		  solution_found = true;
		  solution_cost = curr->g_val;
		} else {  // generate the two successors that resolve one of the conflicts
		  int agent1_id, agent2_id, location1, location2, timestep;
		  tie(agent1_id, agent2_id, location1, location2, timestep) = earliest_conflict;  // choose differently? (used to be collision_vec->at(0))
	//#ifndef NDEBUG
	//      cout << "   Earliest collision -- A1:" << agent1_id << " ; A2: " << agent2_id
	//	   << " ; L1:" << location1 << " ; L2:" << location2*(-1)-1 << " ; T:" << timestep << endl;
	//#endif
		  ECBSNode* n1 = new ECBSNode();
		  ECBSNode* n2 = new ECBSNode();
		  n1->parent = curr;
		  n2->parent = curr;
		  n1->g_val = curr->g_val;
		  n2->g_val = curr->g_val;
		  n1->sum_min_f_vals = curr->sum_min_f_vals;
		  n2->sum_min_f_vals = curr->sum_min_f_vals;
		  
		  if (cons_strategy == constraint_strategy::N_ECBS)
		  {
			  n1->agent_id = agent1_id;
			  n2->agent_id = agent1_id;
			  n1->constraint = make_tuple(location1, location2, timestep, true);
			  n2->constraint = make_tuple(location1, location2, timestep, false);
		  }
		  else
		  {
			  n1->agent_id = agent1_id;
			  n2->agent_id = agent2_id;
			  if (location2 < 0)
			  {  // generate vertex constraint
				  n1->constraint = make_tuple(location1, -1, timestep, false);
				  n2->constraint = make_tuple(location1, -1, timestep, false);
			  }
			  else
			  {  // generate edge constraint
				  n1->constraint = make_tuple(location1, location2, timestep, false);
				  n2->constraint = make_tuple(location2, location1, timestep, false);
			  }
		  }
		  
		  generateChild(n1);
		  generateChild(n2);

		  if (open_list.size() == 0) {
			solution_found = false;
			break;
		  }
		  ECBSNode* open_head = open_list.top();
		if ( open_head->sum_min_f_vals > min_sum_f_vals ) {
	//#ifndef NDEBUG
	//	cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
	//#endif
			min_sum_f_vals = open_head->sum_min_f_vals;
			double new_focal_list_threshold = min_sum_f_vals * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
		/*cout << focal_list.size() << endl;*/
		  }
		  //            cout << " ; (after) " << focal_list_threshold << endl << endl;
		}  // end generating successors
		delete (collision_vec);
  }  // end of while loop

  // get time
  runtime = (std::clock() - start); // / (double) CLOCKS_PER_SEC;

  /*if (solution_found)
    cout << "1 ; ";
  else
    cout << "0 ; ";*/
  
  cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
      HL_num_expanded << " ; " << HL_num_generated << " ; " <<
      LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " ;
	  /*if(HL_num_expanded  > 1)
		cout << num_useful_constraints * 1.0 / (HL_num_expanded - 1);*/
	  cout <<  endl;
    //    printPaths();
	  delete (empty_node);
	  delete (deleted_node);
  return solution_found;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::findPathForSingleAgent(ECBSNode*  node, int ag)
{
	// extract all constraints on agent ag
	ECBSNode* curr = node;
	vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(curr, ag);
	// build reservation table
	size_t max_plan_len = getPathsMaxLength();
	bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
	updateReservationTable(res_table, max_plan_len, ag);
	// find a path w.r.t cons_vec (and prioretize by res_table).
	bool foundSol = search_engines[ag]->findPath(focal_w, cons_vec, res_table, max_plan_len);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	delete (cons_vec);
	delete[] res_table;
	if (foundSol)
	{
		node->paths_updated.push_back(make_tuple(ag, vector<pathEntry>(search_engines[ag]->getPath()), search_engines[ag]->path_cost, search_engines[ag]->min_f_val));
		node->g_val = node->g_val - paths_costs[ag] + search_engines[ag]->path_cost;
		node->sum_min_f_vals = node->sum_min_f_vals - ll_min_f_vals[node->agent_id] + search_engines[ag]->min_f_val;
	}
	else
	{
		delete node;
		return false;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ECBSSearch::generateChild(ECBSNode*  node)
{
	if (get<3>(node->constraint)) //positve constraint
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == node->agent_id)
				continue;
			else if (paths[ag]->at(get<2>(node->constraint)).location == get<0>(node->constraint))
			{
				if(!findPathForSingleAgent(node, ag))
					return false;
			}
		}
	}
	else // negative constraint
	{
		if (!findPathForSingleAgent(node, node->agent_id))
			return false;
	}
	// update n1's path for computing the num of colliding agents
	list<vector<pathEntry>*> temp_old_path;
	for(list<tuple<int, vector<pathEntry>, double, double>>::iterator it = node->paths_updated.begin(); it != node->paths_updated.end(); it++)
	{
		temp_old_path.push_back(paths[get<0>(*it)]);
		paths[get<0>(*it)] = &(get<1>(*it));
	}
	node->num_of_collisions = computeNumOfCollidingAgents();
	list<vector<pathEntry>*>::iterator p = temp_old_path.begin();
	for (list<tuple<int, vector<pathEntry>, double, double>>::iterator it = node->paths_updated.begin();
		it != node->paths_updated.end(); it++, p++)// restore the old path
	{
		paths[get<0>(*it)] = *p;
	}


	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->sum_min_f_vals <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
	//allNodes_table[n1] = n1;
	allNodes_table.push_back(node);
	return true;
}


ECBSSearch::~ECBSSearch() {
  for (size_t i = 0; i < search_engines.size(); i++)
    delete (search_engines[i]);
  for (size_t i = 0; i < paths_found_initially.size(); i++)
    delete (paths_found_initially[i]);
  //  for (size_t i=0; i<paths.size(); i++)
  //    delete (paths[i]);
  releaseClosedListNodes();
}
