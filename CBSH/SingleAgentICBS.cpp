#include "SingleAgentICBS.h"

/*bool SingleAgentICBS::findPathByMDD(bool* res_table, MDD &mdd)
{
	// clear data structures if they had been used before
	// (note -- nodes are deleted before findPath returns)
	mdd_open_t open;

	//add start to the OPEN list
	open.push(mdd.levels[0].front());
	while (!open.empty())
	{
		MDDNode* node = open.top();
		open.pop();
		if (node->level == mdd.levels.size() - 1) // Goal Test
		{
			path.resize(mdd.levels.size());
			for (int t = mdd.levels.size() - 1; t >= 0; t--)
			{
				path[t].location = node->location;
				node = node->parent;
			}
			path_cost = path.size() - 1;

			return true;
		}
		for (list<MDDNode*>::iterator child = node->children.begin(); child != node->children.end(); child++)
		{
			int newConfs = node->num_internal_conf
				+ numOfConflictsForStep(node->location, (*child)->location, (*child)->level, res_table, mdd.levels.size() - 1);
			if ((*child)->parent == NULL || (*child)->num_internal_conf > newConfs)
			{
				(*child)->parent = node;
				(*child)->num_internal_conf = newConfs;
				open.push(*child);
			}
		}
	}
}*/

void SingleAgentICBS::updatePath(const LLNode* goal, vector<PathEntry> &path)
{
	//path = std::shared_ptr<vector<PathEntry>>(new vector<PathEntry>(goal->timestep + 1));
	path.resize(goal->timestep + 1);
	const LLNode* curr = goal;
	// cout << "   UPDATING Path for one agent to: ";
	num_of_conf = goal->num_internal_conf;
	for(int t = goal->timestep; t >= 0; t--)
	{
		path[t].location = curr->loc;
		curr = curr->parent;
	}
}



// iterate over the constraints ( cons[t] is a list of all constraints for timestep t) and return the latest
// timestep which has a constraint involving the goal location
int SingleAgentICBS::extractLastGoalTimestep(int goal_location, const vector< list< tuple<int, int, bool> > >* cons) {
	if (cons != NULL) {
		for (int t = static_cast<int>(cons->size()) - 1; t > 0; t--) {
			for (list< tuple<int, int, bool> >::const_iterator it = cons->at(t).begin(); it != cons->at(t).end(); ++it) {
				if (get<0>(*it) == goal_location && get<1>(*it) < 0 && !get<2>(*it)) {
					return (t);
				}
			}
		}
	}
	return -1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// input: curr_id (location at time next_timestep-1) ; next_id (location at time next_timestep); next_timestep
//        cons[timestep] is a list of <loc1,loc2> of (vertex/edge) constraints for that timestep.
inline bool SingleAgentICBS::isConstrained(int curr_id, int next_id, int next_timestep, const vector< list< tuple<int, int, bool> > >* cons)  const {
	//  cout << "check if ID="<<id<<" is occupied at TIMESTEP="<<timestep<<endl;
	if (cons == NULL)
		return false;
	// check vertex constraints (being in next_id at next_timestep is disallowed)
	if (next_timestep < static_cast<int>(cons->size()))
	{
		for (list< tuple<int, int, bool> >::const_iterator it = cons->at(next_timestep).begin(); it != cons->at(next_timestep).end(); ++it)
		{
			if (get<2>(*it)) // positive constraint
			{
				if (get<0>(*it) != next_id)  //can only stay at constrained location
					return true;
			}
			else //negative constraint
			{
				if ((get<0>(*it) == next_id && get<1>(*it) < 0)//vertex constraint
					|| (get<0>(*it) == curr_id && get<1>(*it) == next_id)) // edge constraint
					return true;
			}
		}
	}
	return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int SingleAgentICBS::numOfConflictsForStep(int curr_id, int next_id, int next_timestep, const bool* res_table, int max_plan_len) {
	int retVal = 0;
	if (next_timestep >= max_plan_len) {
		// check vertex constraints (being at an agent's goal when he stays there because he is done planning)
		if (res_table[next_id + (max_plan_len - 1)*map_size] == true)
			retVal++;
		// Note -- there cannot be edge conflicts when other agents are done moving
	}
	else {
		// check vertex constraints (being in next_id at next_timestep is disallowed)
		if (res_table[next_id + next_timestep*map_size] == true)
			retVal++;
		// check edge constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		// which means that res_table is occupied with another agent for [curr_id,next_timestep] and [next_id,next_timestep-1]
		if (res_table[curr_id + next_timestep*map_size] && res_table[next_id + (next_timestep - 1)*map_size])
			retVal++;
	}
	//  cout << "#CONF=" << retVal << " ; For: curr_id=" << curr_id << " , next_id=" << next_id << " , next_timestep=" << next_timestep
	//       << " , max_plan_len=" << max_plan_len << endl;
	return retVal;
}

// $$$ -- is there a more efficient way to do that?
void SingleAgentICBS::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) {
	
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// return true if a path found (and updates vector<int> path) or false if no path exists
bool SingleAgentICBS::findPath(vector<PathEntry> &path, double f_weight, const vector < list< tuple<int, int, bool> > >* constraints, const bool* res_table, size_t max_plan_len, double lowerbound) {
	// clear data structures if they had been used before
	// (note -- nodes are deleted before findPath returns)
	

	num_expanded = 0;
	num_generated = 0;

	hashtable_t::iterator it;  // will be used for find()

	 // generate start and add it to the OPEN list
	LLNode* start = new LLNode(start_location, 0, my_heuristic[start_location], NULL, 0, 0, false);
	num_generated++;
	start->open_handle = open_list.push(start);
	start->focal_handle = focal_list.push(start);
	start->in_openlist = true;
	allNodes_table[start] = start;
	min_f_val = start->getFVal();
	lower_bound = max(lowerbound, f_weight * min_f_val);

	int lastGoalConsTime = extractLastGoalTimestep(goal_location, constraints);

	while (!focal_list.empty()) {
		//    cout << "|F|=" << focal_list.size() << " ; |O|=" << open_list.size() << endl;
		LLNode* curr = focal_list.top(); focal_list.pop();
		//    cout << "Current FOCAL bound is " << lower_bound << endl;
		//    cout << "POPPED FOCAL's HEAD: (" << curr << ") " << (*curr) << endl;
		open_list.erase(curr->open_handle);
		//    cout << "DELETED" << endl; fflush(stdout);
		curr->in_openlist = false;
		num_expanded++;

		// check if the popped node is a goal
		if (curr->loc == goal_location && curr->timestep > lastGoalConsTime) {
			updatePath(curr, path);
			releaseClosedListNodes(&allNodes_table);
			open_list.clear();
			focal_list.clear();
			allNodes_table.clear();
			return true;
		}

		int next_id;
		for (int i = 0; i < 5; i++)
		{
			next_id = curr->loc + moves_offset[i];

			int next_timestep = curr->timestep + 1;
			if (0 <= next_id && next_id < map_size && abs(next_id % moves_offset[MapLoader::valid_moves_t::SOUTH] - curr->loc % moves_offset[MapLoader::valid_moves_t::SOUTH]) < 2)
			{
				bool free = true;
				int num_row = map_size / num_col;
				//cout << "NUMBER of rows and cols: " <<num_row << " " << num_col << endl;;
				int row = next_id / num_col;
				int col = next_id % num_col;

				if (!my_map[next_id] && !isConstrained(curr->loc, next_id, next_timestep, constraints)) {
					// compute cost to next_id via curr node
					int next_g_val = curr->g_val + 1;
					int next_h_val = my_heuristic[next_id];
					int next_internal_conflicts = 0;
					if (max_plan_len > 0)  // check if the reservation table is not empty (that is tha max_length of any other agent's plan is > 0)
						next_internal_conflicts = curr->num_internal_conf + numOfConflictsForStep(curr->loc, next_id, next_timestep, res_table, max_plan_len);
					// generate (maybe temporary) node
					LLNode* next = new LLNode(next_id, next_g_val, next_h_val,	curr, next_timestep, next_internal_conflicts, false);
					// cout << "   NEXT(" << next << ")=" << *next << endl;
					// try to retrieve it from the hash table
					it = allNodes_table.find(next);

					if (it == allNodes_table.end()) {
						//          cout << "   ADDING it as new." << endl;
						next->open_handle = open_list.push(next);
						next->in_openlist = true;
						num_generated++;
						if (next->getFVal() <= lower_bound)
							next->focal_handle = focal_list.push(next);
						allNodes_table[next] = next;
					}
					else {  // update existing node's if needed (only in the open_list)
						delete(next);  // not needed anymore -- we already generated it before
						LLNode* existing_next = (*it).second;
						//          cout << "Actually next exists. It's address is " << existing_next << endl;
						if (existing_next->in_openlist == true) {  // if its in the open list
							if (existing_next->getFVal() > next_g_val + next_h_val ||
								(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) {
								// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
								//              cout << "   UPDATE its f-val in OPEN (decreased or less #conflicts)" << endl;
								//              cout << "   Node state before update: " << *existing_next;
								bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
								bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
								bool update_open = false;
								if ((next_g_val + next_h_val) <= lower_bound) {  // if the new f-val qualify to be in FOCAL
									if (existing_next->getFVal() > lower_bound)
										add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
									else
										update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
								}
								if (existing_next->getFVal() > next_g_val + next_h_val)
									update_open = true;
								// update existing node
								existing_next->g_val = next_g_val;
								existing_next->h_val = next_h_val;
								existing_next->parent = curr;
								existing_next->num_internal_conf = next_internal_conflicts;
								//              cout << "   Node state after update: " << *existing_next;
								if (update_open) {
									open_list.increase(existing_next->open_handle);  // increase because f-val improved
																					 //                cout << "     Increased in OPEN" << endl;
								}
								if (add_to_focal) {
									existing_next->focal_handle = focal_list.push(existing_next);
									//                cout << "     Inserted to FOCAL" << endl;
								}
								if (update_in_focal) {
									focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
																					 //                cout << "     Updated in FOCAL" << endl;
								}
							}
							//            cout << "   Do NOT update in OPEN (f-val for this node increased or stayed the same and has more conflicts)" << endl;
						}
						else {  // if its in the closed list (reopen)
							if (existing_next->getFVal() > next_g_val + next_h_val ||
								(existing_next->getFVal() == next_g_val + next_h_val && existing_next->num_internal_conf > next_internal_conflicts)) {
								// if f-val decreased through this new path (or it remains the same and there's less internal conflicts)
								//              cout << "   Reinsert it to OPEN" << endl;
								//              cout << "   Node state before update: " << *existing_next;
								existing_next->g_val = next_g_val;
								existing_next->h_val = next_h_val;
								existing_next->parent = curr;
								existing_next->num_internal_conf = next_internal_conflicts;
								existing_next->open_handle = open_list.push(existing_next);
								existing_next->in_openlist = true;
								//              cout << "   Node state after update: " << *existing_next;
								if (existing_next->getFVal() <= lower_bound) {
									existing_next->focal_handle = focal_list.push(existing_next);
									//                cout << "     Inserted to FOCAL" << endl;
								}
							}
							//            cout << "   Do NOT reopen" << endl;
						}  // end update a node in closed list
					}  // end update an existing node
				}  // end if case for grid not blocked
			}
		}  // end for loop that generates successors
		   // update FOCAL if min f-val increased
		if (open_list.size() == 0)  // in case OPEN is empty, no path found...
			break;
		LLNode* open_head = open_list.top();
		if (open_head->getFVal() > min_f_val) {
			double new_min_f_val = open_head->getFVal();
			double new_lower_bound = max(lowerbound, f_weight * new_min_f_val);
			/*
			cout << "LL FOCAL UPDATE! Old-f-min=" << min_f_val << " ; Old-LB=" << lower_bound << endl;
			cout << "OPEN: ";
			for (Node* n : open_list)
			cout << n << " , ";
			cout << endl;
			cout << "FOCAL: ";
			for (Node* n : focal_list)
			cout << n << " , ";
			cout << endl;
			*/
			//  cout << "Update Focal: (old_LB=" << lower_bound << " ; new_LB=" << new_lower_bound << endl;;
			for (LLNode* n : open_list) {
				//    cout << "   Considering " << n << " , " << *n << endl;
				if (n->getFVal() > lower_bound &&
					n->getFVal() <= new_lower_bound) {
					//      cout << "      Added (n->f-val=" << n->getFVal() << ")" << endl;
					n->focal_handle = focal_list.push(n);
				}
			}
			//updateFocalList(lower_bound, new_lower_bound, f_weight);
			min_f_val = new_min_f_val;
			lower_bound = new_lower_bound;
			/*
			cout << "   New-f-min=" << min_f_val << " ; New-LB=" << lower_bound << endl;
			cout << "FOCAL: ";
			for (Node* n : focal_list)
			cout << n << " , ";
			cout << endl;
			*/
		}
		if (focal_list.size() == 0)
			std::cout << "ERROR!" << std::endl;
	}  // end while loop
	   // no path found
	//path.clear();
	releaseClosedListNodes(&allNodes_table);
	open_list.clear();
	focal_list.clear();
	allNodes_table.clear();
	return false;
}

inline void SingleAgentICBS::releaseClosedListNodes(hashtable_t* allNodes_table) {
	hashtable_t::iterator it;
	for (it = allNodes_table->begin(); it != allNodes_table->end(); it++) {
		delete ((*it).second);  // Node* s = (*it).first; delete (s);
	}
}

SingleAgentICBS::SingleAgentICBS(int start_location, int goal_location,
	const bool* my_map, int map_size, const int* moves_offset, int num_col) 
{
	this->my_map = my_map;
	this->moves_offset = moves_offset;
	//this->actions_offset = actions_offset;
	this->start_location = start_location;
	this->goal_location = goal_location;
	//this->start_orientation = start_orientation;
	this->map_size = map_size;
	//this->e_weight = e_weight;
	this->num_expanded = 0;
	this->num_generated = 0;
	//this->path_cost = 0;
	this->lower_bound = 0;
	this->min_f_val = 0;
	//this->num_non_hwy_edges = 0;
	this->num_col = num_col;

	// initialize allNodes_table (hash table)
	empty_node = new LLNode();
	empty_node->loc = -1;
	deleted_node = new LLNode();
	deleted_node->loc = -2;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);

}


SingleAgentICBS::~SingleAgentICBS()
{
	delete[] my_map;
	delete (empty_node);
	delete (deleted_node);
}
