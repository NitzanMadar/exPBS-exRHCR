#include "GICBSSearch.h"
#include "utils_functions.h"

//#define ROOT
//#define DEBUG
//#define STAT


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void GICBSSearch::updatePaths(GICBSNode* curr) {
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false
												 /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
												 * because younger nodes take into account ancestors' nodes constraints. */
	while (curr->parent != NULL)
	{
		for (list<pair<int, vector<PathEntry>>>::iterator it = curr->new_paths.begin(); it != curr->new_paths.end(); it++)
		{
			if (!updated[it->first])
			{
				paths[it->first] = &(it->second);
				updated[get<0>(*it)] = true;
			}
		}
		curr = curr->parent;
	}
}

void GICBSSearch::findConflicts(GICBSNode& curr)
{
	// int window_size ;  // this is for windowed-MAPF
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();

			for (size_t timestep = 0; timestep < min_path_length; timestep++)  //search until min path length
			{
			    //  - windowed-MAPF here. if timestep>w break without adding collisions!
			    if (window_size > -1 && timestep > window_size-1) {
                        // cout << "windowed MAPF - break for agent" << a1 << endl;
                        break;
			    }

				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)  // vertex collision
				{
					curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, -1, timestep));
					return;
					//hasConflicts[a1] = true;
					//hasConflicts[a2] = true;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)  // edge collision
				{
					curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, loc2, timestep + 1));
					//hasConflicts[a1] = true;
					//hasConflicts[a2] = true;
					return;
				}
			}

			if (paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
                    //  - windowed-MAPF here. if timestep>w break without adding collisions!
                    if (window_size > -1 && timestep > window_size-1) {
                        // cout << "windowed MAPF - break for agent" << a1 << endl;
                        break;
                    }

					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						curr.conflict = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1_, a2_, loc1, -1, timestep)); // It's at least a semi conflict
						//curr.unknownConf.front()->cost1 = timestep + 1;
						//hasConflicts[a1] = true;
						//hasConflicts[a2] = true;
						return;
					}
				}
			}
		}
	}
	curr.conflict = NULL;
	return;
}

int GICBSSearch::computeCollidingTeams()
{
	int result = 0;
	//vector<bool> hasConflicts(num_of_agents, false);
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
            // cout << "windowed MAPF -  for a1" << a1 << " and a2 " << a2 << endl;
			bool isColliding = false;
			size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
            // - windowed-MAPF here. if timestep>w break without adding collisions!
            // if (window_size > -1) {
            //     // cout << "windowed MAPF - break for agent" << a1 << endl;
            //     min_path_length = window_size;
            //
            // }
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{

                //  - windowed-MAPF here. if timestep>w break without adding collisions!
                if (window_size > -1 && timestep > window_size-1) {
                    // cout << "windowed MAPF - break for agent" << a1 << endl;
                    break;
                }

				int loc1 = paths[a1]->at(timestep).location;
				int loc2 = paths[a2]->at(timestep).location;
				if (loc1 == loc2)
				{
					result++;
					isColliding = true;
					break;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths[a2]->at(timestep + 1).location
					&& loc2 == paths[a1]->at(timestep + 1).location)
				{
					result++;
					isColliding = true;
					break;
				}
			}
			if (!isColliding && paths[a1]->size() != paths[a2]->size())
			{
				int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
				int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
				int loc1 = paths[a1_]->back().location;
				for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
				{
                    //  - windowed-MAPF here. if timestep>w break without adding collisions!
                    if (window_size > -1 && timestep > window_size-1) {
                        // cout << "windowed MAPF - break for agent" << a1 << endl;
                        break;
                    }

					int loc2 = paths[a2_]->at(timestep).location;
					if (loc1 == loc2)
					{
						result++;
						break;
					}
				}
			}
		}
	}
	return result;
}


/*
return agent_id's location for the given timestep
Note -- if timestep is longer than its plan length,
then the location remains the same as its last cell)
*/
inline int GICBSSearch::getAgentLocation(int agent_id, size_t timestep) {
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return paths[agent_id]->at(timestep).location;
}


bool GICBSSearch::findPathForSingleAgent(GICBSNode*  node, int ag, double lowerbound)  // used in generateChild only
{

	// extract all constraints on agent ag
	//GICBSNode* curr = node;
	bool foundSol = true;
	vector<vector<bool>> consistent(num_of_agents, vector<bool>(num_of_agents, true));
	vector<vector<PathEntry>> new_paths(num_of_agents, vector<PathEntry>());



	vector<bool> visited(num_of_agents, true);
	for (int i = 0; i < num_of_agents; i++) {
		if (node->trans_priorities[ag][i]) {
			visited[i] = false;
		}
	}

	stack<pair<bool, int> > dfs;
	list<int> topSort;
	dfs.push(make_pair(false, ag));
	while (!dfs.empty()) {
		pair<bool, int> parent = dfs.top();
		dfs.pop();
		if (parent.first) {
			topSort.push_front(parent.second);
			continue;
		}
		visited[parent.second] = true;
		dfs.push(make_pair(true, parent.second));
		for (int i = 0; i < num_of_agents; i++) {
			if (node->priorities[parent.second][i] && !visited[i]) {
				dfs.push(make_pair(false, i));
			}
		}
	}

	for (auto iter = topSort.begin(); iter != topSort.end(); iter++) {
		int curr_agent = *iter;
		bool isColliding = false;
		for (int a2 = 0; a2 < num_of_agents; a2++) {
			if (!consistent[a2][curr_agent]) {
				size_t min_path_length = paths[curr_agent]->size() < paths[a2]->size() ? paths[curr_agent]->size() : paths[a2]->size();
				for (size_t timestep = 0; timestep < min_path_length; timestep++)
				{

                    if (window_size > -1 && timestep > window_size-1) {
                        break;
                    }

					int loc1 = paths[curr_agent]->at(timestep).location;
					int loc2 = paths[a2]->at(timestep).location;
					if (loc1 == loc2)
					{
						isColliding = true;
						break;
					}
					else if (timestep < min_path_length - 1
						&& loc1 == paths[a2]->at(timestep + 1).location
						&& loc2 == paths[curr_agent]->at(timestep + 1).location)
					{
						isColliding = true;
						break;
					}
				}
				if (!isColliding && paths[curr_agent]->size() != paths[a2]->size())
				{
					int a1_ = paths[curr_agent]->size() < paths[a2]->size() ? ag : a2;
					int a2_ = paths[curr_agent]->size() < paths[a2]->size() ? a2 : ag;
					int loc1 = paths[a1_]->back().location;
					for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
					{

                        if (window_size > -1 && timestep > window_size-1) {
                            // cout << "windowed MAPF - break for agent" << a1 << endl;
                            break;
                        }
						int loc2 = paths[a2_]->at(timestep).location;
						if (loc1 == loc2)
						{
							break;
						}
					}
				}
			}
			if (isColliding) {
				break;
			}
		}
		if (!isColliding && curr_agent != ag) {
			continue;
		}
		size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
												  //bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
												  //bool* res_table_low_prio = new bool[map_size * max_plan_len]();  // initialized to false
												  //updateReservationTable(res_table, res_table_low_prio, curr_agent, *node);
												  // find a path w.r.t cons_vec (and prioretize by res_table).
												  //pair<int, vector<PathEntry>> newPath;
												  //vector<PathEntry> newPath;
												  //newPath.first = curr_agent;
												  //foundSol = search_engines[curr_agent]->findPath(newPath.second, focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);


		if (window_size > 0){ // window_size = -1 for regular MAPf and >0 for windowed MAPF
		    max_plan_len = window_size;
		}

		foundSol = search_engines[curr_agent]->findPath(new_paths[curr_agent], focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);

		LL_num_expanded += search_engines[curr_agent]->num_expanded;
		LL_num_generated += search_engines[curr_agent]->num_generated;
		// if (fixed_prior and LL_num_generated > 1000000){
        //     cout<<"FIXED PRIORITY BREAK - LL_num_generated = " << LL_num_generated<< "< 1M;" << endl ;
        //     return false;
        // }

		//delete (cons_vec);
		//delete[] res_table;
		if (foundSol) {
			//node->new_paths.push_back(newPath);
			//new_paths[curr_agent] = newPath;
			node->g_val = node->g_val - paths[curr_agent]->size() + new_paths[curr_agent].size();
			//paths[curr_agent] = &node->new_paths.back().second;
			paths[curr_agent] = &new_paths[curr_agent]; // might be used by the next findPath() call
														//node->paths[ag] = search_engines[ag]->getPath();
			node->makespan = max(node->makespan, new_paths[curr_agent].size() - 1);
			for (int i = 0; i < num_of_agents; i++) {
				if (node->trans_priorities[curr_agent][i]) {
					consistent[curr_agent][i] = false;
				}
				if (node->trans_priorities[i][curr_agent] && !consistent[i][curr_agent]) {
					consistent[i][curr_agent] = true;
				}
			}
		}
		else {
			return false;
		}
	}

	if (foundSol) {
		for (int i = 0; i < num_of_agents; i++) {
			if (!new_paths[i].empty()) {
				node->new_paths.push_back(make_pair(i, new_paths[i]));
				paths[i] = &node->new_paths.back().second; // make sure paths[i] gets the correct pointer

			}
		}



        //  - change:
        //  add the next return in ubuntu 20,
        //  otherwise - run was return "free() invalid pointer, segmentation fault".
        //  it also possible to add "return FoundSol" at the end and delete all returns
        return true;
	}
	/*
	if (true) {
		std::queue<int> q;
		q.push(ag);
		while (!q.empty() && foundSol) {
			int curr_agent = q.front();
			q.pop();

			bool isColliding = false;
			for (int a2 = 0; a2 < num_of_agents; a2++) {
				if (!consistent[a2][curr_agent]) { // see if ag and a2 collide (because high prior agent a2 has a new path)
					size_t min_path_length = paths[curr_agent]->size() < paths[a2]->size() ? paths[curr_agent]->size() : paths[a2]->size();
					for (size_t timestep = 0; timestep < min_path_length; timestep++)
					{
						int loc1 = paths[curr_agent]->at(timestep).location;
						int loc2 = paths[a2]->at(timestep).location;
						if (loc1 == loc2)
						{
							isColliding = true;
							break;
						}
						else if (timestep < min_path_length - 1
							&& loc1 == paths[a2]->at(timestep + 1).location
							&& loc2 == paths[curr_agent]->at(timestep + 1).location)
						{
							isColliding = true;
							break;
						}
					}
					if (!isColliding && paths[curr_agent]->size() != paths[a2]->size())
					{
						int a1_ = paths[curr_agent]->size() < paths[a2]->size() ? ag : a2;
						int a2_ = paths[curr_agent]->size() < paths[a2]->size() ? a2 : ag;
						int loc1 = paths[a1_]->back().location;
						for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
						{
							int loc2 = paths[a2_]->at(timestep).location;
							if (loc1 == loc2)
							{
								break;
							}
						}
					}
				}
				if (isColliding) {
					break;
				}
			}
			if (!isColliding && curr_agent != ag) {
				continue;
			}

			//vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(node, curr_agent);
			// build reservation table
			size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
													  //bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
													  //bool* res_table_low_prio = new bool[map_size * max_plan_len]();  // initialized to false
													  //updateReservationTable(res_table, res_table_low_prio, curr_agent, *node);
													  // find a path w.r.t cons_vec (and prioretize by res_table).
													  //pair<int, vector<PathEntry>> newPath;
													  //vector<PathEntry> newPath;
													  //newPath.first = curr_agent;
													  //foundSol = search_engines[curr_agent]->findPath(newPath.second, focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
			foundSol = search_engines[curr_agent]->findPath(new_paths[curr_agent], focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
			LL_num_expanded += search_engines[curr_agent]->num_expanded;
			LL_num_generated += search_engines[curr_agent]->num_generated;
			//delete (cons_vec);
			//delete[] res_table;
			if (foundSol) {
				//node->new_paths.push_back(newPath);
				//new_paths[curr_agent] = newPath;
				node->g_val = node->g_val - paths[curr_agent]->size() + new_paths[curr_agent].size();
				//paths[curr_agent] = &node->new_paths.back().second;
				paths[curr_agent] = &new_paths[curr_agent]; // might be used by the next findPath() call
															//node->paths[ag] = search_engines[ag]->getPath();
				node->makespan = max(node->makespan, new_paths[curr_agent].size() - 1);
				for (int i = 0; i < num_of_agents; i++) {
					if (node->priorities[curr_agent][i]) {
						q.push(i);
					}
					if (node->trans_priorities[curr_agent][i]) {
						consistent[curr_agent][i] = false;
					}
					if (node->trans_priorities[i][curr_agent] && !consistent[i][curr_agent]) {
						consistent[i][curr_agent] = true;
					}
				}
			}
			else {
				return false;
			}
		}

		if (foundSol) {
			for (int i = 0; i < num_of_agents; i++) {
				if (!new_paths[i].empty()) {
					node->new_paths.push_back(make_pair(i, new_paths[i]));
					paths[i] = &node->new_paths.back().second; // make sure paths[i] gets the correct pointer
				}
			}
		}
	}
	else {
		//vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(node, ag);
		// build reservation table
		size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
		//bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
		//bool* res_table_low_prio = new bool[map_size * max_plan_len]();  // initialized to false
		//updateReservationTable(res_table, res_table_low_prio, ag, *node);
		// find a path w.r.t cons_vec (and prioretize by res_table).
		pair<int, vector<PathEntry>> newPath;
		newPath.first = ag;
		foundSol = search_engines[ag]->findPath(newPath.second, focal_w, node->trans_priorities, paths, max_plan_len, lowerbound);
		LL_num_expanded += search_engines[ag]->num_expanded;
		LL_num_generated += search_engines[ag]->num_generated;
		//delete (cons_vec);
		//delete[] res_table;
		if (foundSol) {
			node->new_paths.push_back(newPath);
			node->g_val = node->g_val - paths[ag]->size() + newPath.second.size();
			paths[ag] = &node->new_paths.back().second;
			//node->paths[ag] = search_engines[ag]->getPath();
			node->makespan = max(node->makespan, newPath.second.size() - 1);
		}
	}
	if (foundSol)
	{
#ifdef STAT
		//std::cout << paths[ag]->size() - 1 << " ; " << newPath.second.size() - 1 << " ; " << search_engines[ag]->num_expanded << " ; " << search_engines[ag]->num_generated << std::endl;
		node_stat.push_back(make_tuple(HL_num_generated, paths_found_initially[ag].size() - 1, paths[ag]->size() - 1, newPath.second.size() - 1, search_engines[ag]->num_expanded, search_engines[ag]->num_generated));
#endif
		//search_engines[ag]->path.reset();
		return true;
	}
	else
	{
		//delete node;
		return false;
	}*/

}


bool GICBSSearch::generateChild(GICBSNode*  node, GICBSNode* curr)
{

	node->parent = curr;
	node->g_val = curr->g_val;
	node->makespan = curr->makespan;
	//node->f_val = curr->f_val - curr->h_val;
	node->depth = curr->depth + 1;

    if ( (int)node->depth > MaxDepth)
        {MaxDepth = curr->depth;}

	//node->paths = curr->paths;
	//node->paths.resize(num_of_agents);
	//node->paths.assign(curr->paths.begin(), curr->paths.end());
	//node->single.resize(num_of_agents, NULL);
	std::clock_t t1;

	t1 = std::clock();

	if (get<3>(node->constraint)) // positive constraint
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{

			if (ag == node->agent_id)
				continue;
			else if (get<1>(node->constraint) < 0 && // vertex constraint
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
				//else
				//	node->paths[ag] = &(get<1>(node->paths_updated.back()));
			}
			else if (get<1>(node->constraint) >= 0 && //edge constraint
				getAgentLocation(ag, get<2>(node->constraint) - 1) == get<1>(node->constraint) &&
				getAgentLocation(ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
				else
					paths[ag] = &(node->new_paths.back().second);
			}
		}
	}
	else // negative constraint
	{

		double lowerbound;
		if (get<2>(*curr->conflict) < 0) // rectangle conflict
			lowerbound = (int)paths[node->agent_id]->size() - 1;
		else if(get<4>(*curr->conflict) >= (int)paths[node->agent_id]->size()) //conflict happens after agent reaches its goal
			lowerbound = get<4>(*curr->conflict) + 1;
		else if(!paths[node->agent_id]->at(get<4>(*curr->conflict)).single) // not cardinal
			lowerbound = (int)paths[node->agent_id]->size() - 1;
		else if(get<2>(*curr->conflict) >= 0 && get<3>(*curr->conflict) < 0) // Cardinal vertex
			lowerbound = (int)paths[node->agent_id]->size();
		else if (paths[node->agent_id]->at(get<4>(*curr->conflict) - 1).single) // Cardinal edge
			lowerbound = (int)paths[node->agent_id]->size();
		else // Not cardinal edge
			lowerbound = (int)paths[node->agent_id]->size() - 1;




		if (!findPathForSingleAgent(node, node->agent_id))
			return false;
		//else
		//	paths[node->agent_id] = &(get<1>(node->paths_updated.back()));
	}

	runtime_lowlevel += std::clock() - t1;

	node->f_val = node->g_val;

	t1 = std::clock();
	//findConflicts(*node);
	runtime_conflictdetection += std::clock() - t1;
	node->num_of_collisions = computeCollidingTeams();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	allNodes_table.push_back(node);

	// Copy single vector from parent
	/*node->single.resize(num_of_agents);
	for (int i = 0; i < curr->single.size(); i++)
	{
		if (!curr->single[i])
			continue;
		else
		{
			bool updated = false;
			for (list<int>::iterator it = node->agents_updated.begin(); it != node->agents_updated.end(); it++)
			{
				if (*it == i)
				{
					updated = true;
					break;
				}
			}
			if (!updated)
			{
				node->single[i] = curr->single[i];
			}
		}
	}*/

	return true;
}


void GICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < (int)paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}


string GICBSSearch::printLocationsForNextWindowedMapf(int h) const
{
    /* Thus function print new agents file after executing h steps. Used when solution founded, h>0, and agents fname[-8]='w'*/
    // int h;  // h = replanning rate
    int path_len;
    string locations_str;
    int last_loc_idx ;
    for (int i = 0; i < num_of_agents; i++)
    {
        // h = window_size-1; // I used here w=h for RHCR (lifelong MAPF version that decompose to windowed-MAPF queries)
        last_loc_idx = h - 1;
        path_len = paths[i]->size() - 1; // last position index is path length - 1
        if (path_len < last_loc_idx) // if path length < replanning frequency h, take last position,
            last_loc_idx = path_len;

        // std::cout << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << std::endl;
        locations_str += std::to_string(paths[i]->at(last_loc_idx).location / num_col);
        locations_str += ",";
        locations_str += std::to_string(paths[i]->at(last_loc_idx).location % num_col);
        locations_str += ",";
        locations_str += std::to_string(paths[i]->at(path_len).location / num_col);  // OK
        locations_str += ",";
        locations_str += std::to_string(paths[i]->at(path_len).location % num_col);  // OK
        locations_str += ",";
        locations_str += "\n";
    }
    return locations_str;
}


void GICBSSearch::printConstraints(const GICBSNode* n) const
{
	const GICBSNode* curr = n;
	while (curr != dummy_start)
	{
		std::cout << "<" << curr->agent_id
						<< ", " << get<0>(curr->constraint)
						<< ", " << get<1>(curr->constraint)
						<< ", " << get<2>(curr->constraint);
		if(get<3>(curr->constraint))
			std::cout << ", positive>" << std::endl;
		else
			std::cout << ", negative>" << std::endl;
		curr = curr->parent;
	}
}

// computes g_val based on current paths
inline int GICBSSearch::compute_g_val() {
	int retVal = 0;
	for (int i = 0; i < num_of_agents; i++)
		retVal += paths[i]->size() - 1;
	return retVal;
}


bool GICBSSearch::runGICBSSearch()
{
//    cout<< "use experience: "<< use_experience;
//    cout << "num_of_agents: "<< num_of_agents;
	node_stat.clear();
	switch (cons_strategy)
	{
	case constraint_strategy::ICBS:
		cout << "       ICBS: ";
		break;
	case constraint_strategy::N_ICBS:
		cout << "     N-ICBS: ";
		break;
	case constraint_strategy::CBSH:
		cout << "       CBSH: ";
		break;
	case constraint_strategy::N_CBSH:
		cout << "   N-CBSH" <<  ":";
		break;
	case constraint_strategy::CBSH_R:
		cout << "   CBSH-R" << ":";
		break;
	case constraint_strategy::CBSH_CR:
		cout << "  CBSH-CR" << ":";
		break;
	case constraint_strategy::CBSH_RM:
		cout << "  CBSH-RM" << ":";
		break;
	default:
		cout << "WRONG SOLVER!" << std::endl;
		return false;
	}
	if (solution_cost == -2) {
		runtime = pre_runtime;
		return false;
	}
	// set timer

	std::clock_t start;
	start = std::clock();
	std::clock_t t1;
	runtime_computeh = 0;
	runtime_lowlevel = 0;
	runtime_listoperation = 0;
	runtime_conflictdetection = 0;
	runtime_updatepaths = 0;
	runtime_updatecons = 0;
	// start is already in the open_list
	//upper_bound = DBL_MAX;
    bool experience_runout_HL_limit = false;


	while (!open_list.empty() && !solution_found)  // open list not empty and solution didn't found yet
	{

	    // break after x min (timeout_limit miliseconds)//
	    int timeout_limit = 0.5*60000000; // timeout after minute (60*100000 = 60000000 [milliseconds] = 60 [seconds])

		runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
		if (runtime > timeout_limit || HL_num_expanded > 1000000) // runtime break or HL expanded break
		{
			cout << "TIMEOUT " << timeout_limit << " ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << endl;

			std::cout << "	Runtime Sumarry: lowlevel = " << runtime_lowlevel << " ; listoperation = " << runtime_listoperation <<
						" ; conflictdetection = " << runtime_conflictdetection << " ; computeh = " << runtime_computeh<<
						" ; updatepaths = " << runtime_updatepaths << " ; collectcons = " << runtime_updatecons << std::endl;

			double count = 0, value = 0;//, maxDepth = 0;
			GICBSNode* curr = NULL;
			bool open_empty = false;
			if ((open_list.empty()))
				open_empty = true;
			while (!open_list.empty())
			{
				curr = open_list.top();
				open_list.pop();
				if ((int)curr->depth > MaxDepth)
                    MaxDepth = curr->depth;
				if (curr->f_val > value + 0.001)
				{
					cout << "				#(f=" << value << ") = " << count << endl;
					count = 1;
					value = curr->f_val;
				}
				else
					count++;
			}
			if (!open_empty) {
				std::cout << "Depth of last node: " << curr->depth << " ; MaxDepth = " << MaxDepth << std::endl;
			}
			else {
				std::cout << "Open List Empty!!!" << endl;
			}
			solution_found = false;
			break;

			// - added depths to output
            SolutionDepth = curr->depth;  // although here it's a failure
//            MaxDepth = maxDepth;
		}

		t1 = std::clock();

        GICBSNode* curr = open_list.top(); // original pbs / +experience if didn't pass HL node limit
        open_list.pop();

        // cout << "num_of_collisions in current HL node " << curr->num_of_collisions<<endl;
        /*
		// if we use experience and over the EXPERIENCE_HL_NODE_EXPANDED_LIMIT, run original PBS, use current node = empty node
		if (search_with_experience && !experience_runout_HL_limit){ // we want this fall back only once
		    if (!clean_experience && HL_num_expanded > EXPERIENCE_NO_CLEAN_HL_NODE_EXPANDED_LIMIT){
                cout << "PBS+Experience is run out of HL node expansion limit, running PBS+Experience+Clean..." << HL_num_expanded << ", " << LL_num_expanded << endl;
                open_list.clear();
                curr = new GICBSNode();
                empty_priority_node->open_handle = open_list.push(empty_priority_node);
                empty_priority_node->time_generated = HL_num_generated;
		        curr = empty_priority_node;
                open_list.push(empty_priority_node);
                experience_runout_HL_limit = true;
		    }
		    if (clean_experience && HL_num_expanded > EXPERIENCE_WITH_CLEAN_HL_NODE_EXPANDED_LIMIT ) {
                cout << "PBS+Experience+Clean is run out of HL node expansion limit, running original PBS..." << HL_num_expanded << ", " << LL_num_expanded << endl;
                open_list.clear();
                //
                curr = new GICBSNode();
                empty_priority_node->open_handle = open_list.push(empty_priority_node);
                empty_priority_node->time_generated = HL_num_generated;
                curr = empty_priority_node;
                open_list.push(empty_priority_node);
                curr->agent_id = -1;
//                dummy_start->open_handle = open_list.push(dummy_start);
//                dummy_start->time_generated = HL_num_generated;
//                curr = dummy_start;  // if we want to try PBS+Experience when PBS+Experience+Clean is failed
//                open_list.push(dummy_start);
                experience_runout_HL_limit = true;
            }

		}
        */

		// depth appearances dict: [i (depths), how many time we were in depth i during the search]
        depth_appearance_map[(int)curr->depth]++;
        // find largest depth
        auto max_elem_in_depth_appearance_dict = std::max_element(depth_appearance_map.begin(), depth_appearance_map.end(),
                                  [](const pair<int, int>& p1, const pair<int, int>& p2) {return p1.second < p2.second; }); //[max depth, max expansions]
        // cout << "max depth in the dictionary is: " << (int)max_elem_in_depth_appearance_dict->second << endl;
        // cout << "max_breadth=" << max_breadth <<endl;
        if ((int)max_elem_in_depth_appearance_dict->second > max_breadth){
            max_breadth = max_elem_in_depth_appearance_dict->second;

        }
        // cout << "fallback_option " << fallback_option << endl ;
        //// fallback: choose (1) or (2). (2) used in paper.
        // ===================== fallback option = (0,1] - **not discussed in the paper!!!** ==================== //
        // (1) back propagate upward to other node (reuse the current tree):
        if (abs(fallback_option) <= 1 and fallback_option != 0){  //(fallback_option = %tail, (0,1] or -(0,1] which)

            if (!fallbacked_to_original_pbs and max_elem_in_depth_appearance_dict->second >= HL_DFS_width_limit){  // didn't fallback to original already and over width limit
                /*
                // cout << "Dict before:" << endl << endl;
                // for(auto it = depth_appearance_map.cbegin(); it != depth_appearance_map.cend(); ++it){
                //     cout << "depth_appearance_map[" << it->first << "]: " << it->second << endl;
                // }
                 */

                // srand (time(NULL)); // change seed..
                fallback_option = (double) std::rand()/RAND_MAX;
                fallback_option = (fallback_option < 0.3) ? 0.3 : fallback_option;
                cout << " a = " << fallback_option << endl;
                number_of_fallback_upward++;
                // find depth to jump into
                // int depth_to_jump_back = -1;
                // depth_appearance_map[(int)max_elem_in_depth_appearance_dict->first] = 0;


                // find the root of the dead end sub tree

                // // run from leaves upward
                // for (int i = (int)depth_appearance_map.size(); i > 1 ; i--){ // run from root+1 downward
                //     if (depth_appearance_map[i] == 1){  // this means we in the root of the dead-end subtree
                //         depth_to_jump_back = abs(fallback_option)*(i - 1); // how deep before the dead-end sub-tree? fallback_option = % of the tail
                //         if (depth_to_jump_back <= 1){
                //             fallback_option = 0; // hack to avoid illegal jump upward
                //             number_of_fallback_upward = 4;  // > 3 - fallback to original PBS next time
                //             depth_to_jump_back = curr->depth;
                //         }
                //         break;
                //     }
                // }

                // run top down (from root downward)
                for (int i = 1; i < (int)depth_appearance_map.size(); i++){ // run from root+1 downward
                    if (depth_appearance_map[i] != 1){  // this means we in the root of the dead-end subtree
                        depth_to_jump_back = abs(fallback_option)*(i - 1); // how deep before the dead-end sub-tree? fallback_option = % of the tail
                        if (depth_to_jump_back <= 1){
                            cout << "fallback_option = 0 by hack"<<endl<<endl;
                            fallback_option = 0; // hack to avoid illegal jump upward
                            fallbacked_to_original_pbs = true;
                            number_of_fallback_upward = 4;  // > 3 - fallback to original PBS next time
                            depth_to_jump_back = curr->depth;
                        }
                        break;
                    }
                }



                // if (experience_strategy>0 and (number_of_fallback_upward >= 3 or depth_to_jump_back <= 5)){  // than use original  /
                if (experience_strategy>0 and  (number_of_fallback_upward >= 3 or depth_to_jump_back <= 5 or abs(fallback_option) < 0.1)){  // than use original  //

                    open_list.clear();
                    curr = new GICBSNode();
                    empty_priority_node->open_handle = open_list.push(empty_priority_node);
                    empty_priority_node->time_generated = HL_num_generated;

                    curr = empty_priority_node;
                    open_list.push(empty_priority_node);
                    depth_appearance_map.clear();
                    fallback_option = 0;
                    fallbacked_to_original_pbs = true;
                    cout << " try to jump back to depth " << depth_to_jump_back <<
                         " which is <=5 ... using original PBS fallback..." << endl;
                }
                else {  // depth_to_jump_back > 5 and didn't jump under 0...
                    // fallback_option = fallback_option-0.2;
                    // if (abs(fallback_option) <= 0.5 and fallback_option > 0)
                    //     fallback_option = fallback_option*-1;
                    cout << endl << "jump back from depth " << (int)max_elem_in_depth_appearance_dict->first <<
                         " with " << HL_DFS_width_limit << " appearances";


                    // while((int)curr->depth >= depth_to_jump_back or depth_appearance_map[(int)curr->depth] != 1) {  // update open_list and curr node
                    while((int)curr->depth >= depth_to_jump_back ) {  // update open_list and curr node
                        curr = open_list.top();
                        open_list.pop();
                    }

                    for (int idxx=depth_appearance_map.size(); idxx>=depth_to_jump_back; idxx--){
                        depth_appearance_map[idxx]=0;
                    }
                    // cout << endl << "depth_to_jump_back  =" << depth_to_jump_back<<"=== curr node depth is " << (int)curr->depth << endl;

                    // ======


                    // *** change fallback priorities matrix to P_FB-Pexp : *** //
                    if ((fallback_option < 0 and experience_strategy>0) or (number_of_fallback_upward>2 and fallback_option>0)){  // -(0,1] minus..
                        cout << "  (use P_FB-Pexp)" <<endl;
                        curr->trans_priorities = bool_matrices_a_and_not_b(curr->trans_priorities, dummy_start->priorities);
                    }

                    // change fallback_option
                    // srand (time(NULL)); // change seed..
                    fallback_option = (double) std::rand()/RAND_MAX;
                    fallback_option = (fallback_option < 0.3) ? 0.3 : fallback_option;
                    // fallback_option = fallback_option*0.5;
                    // if (abs(fallback_option) <= 0.5)
                    //     fallback_option = -fallback_option;
                    if (number_of_fallback_upward == 3){    //abs(fallback_option) <= 0.2){
                        fallback_option = 0.001;
                        fallbacked_to_original_pbs = true;
                    }
                    // cout << "fallback_option changed to....= " << fallback_option;
                    // print after
                    // cout << "P_i - P_exp = " << endl;
                    // print_bool_matrix(curr->trans_priorities);


                    // ======

                    cout << " to depth " << curr->depth << endl;

                }
            }

        }


        // ========================= fallback option >= 2 ======================== //
        // (2) try another experience:
        if (fallback_option >= 2){
            if (fallback_option == 2){ // used in the paper!
                is_fallback_used = true;  // --> do not use P_exp2, and jump straight to original PBS
            }

            if (search_with_experience and !fallbacked_to_original_pbs and max_elem_in_depth_appearance_dict->second >= HL_DFS_width_limit ) { // P_exp1 failed, width violation
            // if (search_with_experience and !fallbacked_to_original_pbs and HL_num_expanded >= 100) { // P_exp1 failed + limit HL node expanded //

                if (!is_fallback_used){ // P_exp1 failed + P_exp2 didn't used yet
                    cout << endl << "depth "<< max_elem_in_depth_appearance_dict->first << " is used " << HL_DFS_width_limit
                         << " times with P_exp1- clean open list and use fall back experience P_exp2..." << endl;
                    open_list.clear();
                    curr = new GICBSNode();
                    fallback_node->open_handle = open_list.push(fallback_node);
                    fallback_node->time_generated = HL_num_generated;
                    curr = fallback_node;
                    open_list.push(fallback_node);
                    depth_appearance_map.clear();

                    is_fallback_used = true;
                    first_experience_failed = true ;

                    cout << "       CBSH (2nd): ";
                }
                else { // P_exp1 and P_exp2 failed - now try original PBS --> used in the paper!!!!
                    cout << endl << "depth "<< max_elem_in_depth_appearance_dict->first << " is used " << HL_DFS_width_limit
                         << " times with clean open list and use original PBS..." << endl;
                    open_list.clear();
                    empty_priority_node->depth=max_elem_in_depth_appearance_dict->first;
                    curr = new GICBSNode();
                    empty_priority_node->open_handle = open_list.push(empty_priority_node);
                    empty_priority_node->time_generated = HL_num_generated;

                    curr = empty_priority_node;
                    curr->depth=HL_DFS_width_limit;
                    open_list.push(empty_priority_node);
                    depth_appearance_map.clear();

                    fallbacked_to_original_pbs = true;

                    cout << "       CBSH (original): ";
                }


            }
        }





		runtime_listoperation += std::clock() - t1;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += std::clock() - t1;
#ifdef DEBUG
		//printPaths();
#endif
		t1 = std::clock();
		findConflicts(*curr);
		runtime_conflictdetection += std::clock() - t1;
        if ( (int)curr->depth > MaxDepth){
            MaxDepth = curr->depth;
        }

		if (curr->conflict == NULL)           //// Fail to find a conflict => no conflicts (SOLUTION FOUND! :-) )
		{  // found a solution (and finish the while loop)
			runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
			solution_found = true;
			solution_cost = curr->g_val;
            priorities = curr->priorities;
            trans_priorities = curr->trans_priorities;
			cout << fixed << setprecision(0) << solution_cost << " ; " << //solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; ";// << priorities << " ; ";
			cout << endl;

            // - added depth to output
            SolutionDepth = curr->depth;


//#ifdef DEBUG
//			int conflictNum = computeNumOfCollidingAgents();
//			if(conflictNum > 0)
//				std::cout << "ERROR!" << std::endl;
//			std::cout << std::endl << "****** Solution: " << std::endl;
//			printPaths(*curr);
//#endif
			break;
		}


		// Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;

		// if (curr->depth > 0 and fallbacked_to_original_pbs) {
        //    cout << " curr depth = " << curr->depth <<
        //         ", parent depth = " << curr->parent->depth << "FB oprion = " << fallback_option << endl;
		// }
        // for depths visualization:
		/*
        // write to file HL depths: this is used for scripts/draw_HL_graph_by_depth.py - see for_depth_visualization`
        ofstream depth_file;
		if (curr->depth == 0) {
            depth_file.open("for_depth_visualization");
            depth_file << "ROOT, " << curr->time_expanded << ", " << curr->depth << endl;
		}
        else {
            depth_file.open("for_depth_visualization", ios::app);
            depth_file << curr->parent->time_expanded << ", " << curr->time_expanded << ", " << curr->depth << endl;
        }
        // depth_file << HL_num_expanded << ", " << curr->depth << endl;

        depth_file.close();
       */

#ifdef DEBUG
		std::cout << std::endl << "****** Expanded #" << curr->time_generated << " with f= " << curr->g_val <<
			"+" << curr->h_val << " (";
		for (int i = 0; i < num_of_agents; i++)
			std::cout << paths[i]->size() - 1 << ", ";
		std::cout << ")" << std::endl;
		std::cout << "Choose conflict <";
		std::cout << "A1=" << get<0>(*curr->conflict) << ",A2=" << get<1>(*curr->conflict)
			<< ",loc1=(" << get<2>(*curr->conflict) / num_col << "," << get<2>(*curr->conflict) % num_col
			<< "),loc2=(" << get<3>(*curr->conflict) / num_col << "," << get<3>(*curr->conflict) % num_col
			<< "),t=" << get<4>(*curr->conflict) << ">" << std::endl;
#endif

		GICBSNode* n1 = new GICBSNode();
		GICBSNode* n2 = new GICBSNode();

		n1->agent_id = get<0>(*curr->conflict);
		n2->agent_id = get<1>(*curr->conflict);
		if (get<3>(*curr->conflict) < 0) // vertex conflict
		{
			n1->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
			n2->constraint = make_tuple(get<2>(*curr->conflict), -1, get<4>(*curr->conflict), false);
		}
		else // edge conflict
		{
			n1->constraint = make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict), false);
			n2->constraint = make_tuple(get<3>(*curr->conflict), get<2>(*curr->conflict), get<4>(*curr->conflict), false);
		}


		bool Sol1 = false, Sol2 = false;
		vector<vector<PathEntry>*> copy(paths);
        /*
		//int lowerbound1 = max(get<4>(curr->conflict) + 1, (int)paths[n1->agent_id]->size() - 1);
		//int lowerbound2 = max(get<4>(curr->conflict) + 1, (int)paths[n2->agent_id]->size() - 1);
		//lowerbound2 = ; // The cost of path should be at least the confliting time + 1
		//if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
		//{
		//
		//}
		//else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
		//{
		//	if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
		//	{
		//		std::cout << "***********ERROR**************" << std::endl;
		//		system("pause");
		//	}
		//}
        */

        // cout << "curr->depth  = " << curr->depth << endl;

		bool gen_n1 = true, gen_n2 = true;

		if (curr->trans_priorities[n1->agent_id][n2->agent_id]) { // a1->a2 do not generate n1
			gen_n1 = false;
		}
		// else {
        //     gen_n1 = is_adding_a_higher_than_b_legal(curr->trans_priorities, n2->agent_id, n1->agent_id);
		// }

		if (curr->trans_priorities[n2->agent_id][n1->agent_id]) { // a2->a1 do not generate n2
			gen_n2 = false;
		}
		// else {
		//     gen_n2 = is_adding_a_higher_than_b_legal(curr->trans_priorities, n1->agent_id, n2->agent_id);
		// }



		if (gen_n1) {
            n1->priorities = vector < vector < bool >> (curr->priorities);
            n1->trans_priorities = vector < vector < bool >> (curr->trans_priorities);

            n1->priorities[n2->agent_id][n1->agent_id] = true; // a2->a1
            n1->trans_priorities[n2->agent_id][n1->agent_id] = true;
            for (int i = 0; i < num_of_agents; i++) { // transitivity

                if (n1->trans_priorities[n1->agent_id][i]) {
                    // i that weaker than n1.agent id, so n2.agent_id is stronger
                    n1->trans_priorities[n2->agent_id][i] = true;
                }
                if (n1->trans_priorities[i][n2->agent_id] && !n1->trans_priorities[i][n1->agent_id]) {
                    for (int j = 0; j < num_of_agents; j++) {
                        if (n1->trans_priorities[n1->agent_id][j]) {
                            n1->trans_priorities[i][j] = true;
                        }
                    }
                }
            }

        Sol1 = generateChild(n1, curr);

        // /*
        // if generated one node only - do not increase depth
        if (!gen_n2) {
            n1->depth--;
        }
        // */

		}  // end if gen_n1

		paths = copy;

		//updatePaths(curr);
		if (gen_n2) {
            n2->priorities = vector < vector < bool >> (curr->priorities);
            n2->trans_priorities = vector < vector < bool >> (curr->trans_priorities);
            n2->priorities[n1->agent_id][n2->agent_id] = true; // a1->a2, a1 before a2
            n2->trans_priorities[n1->agent_id][n2->agent_id] = true;
            for (int i = 0; i < num_of_agents; i++) { // transitivity
                if (n2->trans_priorities[n2->agent_id][i]) {
                    // i that weaker than n2.agent id, so n1.agent_id is stronger
                    n2->trans_priorities[n1->agent_id][i] = true;
                }
                if (n2->trans_priorities[i][n1->agent_id] && !n2->trans_priorities[i][n2->agent_id]) {
                    // i is better than n1.agent_id and not better than n2.agent_id
                    for (int j = 0; j < num_of_agents; j++) {
                        if (n2->trans_priorities[n2->agent_id][j]) {
                            //n2.agent_id higher than j
                            n2->trans_priorities[i][j] = true;
                        }
                    }
                }
            }

        Sol2 = generateChild(n2, curr);


        // /*
        // if generated one node only - do not increase depth  //  - this is not necessary correct, depends on implementation or HL search manipulations.
        if (!gen_n1) {  // bug fix from "!gen_n2" to "!gen_n1"
            n2->depth--;
        }
        // */

		} // end if gen_n2

#ifdef DEBUG
		if(Sol1)
		{
			std::cout	<< "Generate #" << n1->time_generated
							<< " with cost " << n1->g_val
							<< " and " << n1->num_of_collisions << " conflicts " <<  std::endl;
		}
		else
		{
			std::cout << "No feasible solution for left child! " << std::endl;
		}
		if (Sol2)
		{
			std::cout	<< "Generate #" << n2->time_generated
							<< " with cost " << n2->g_val
							<< " and " << n2->num_of_collisions << " conflicts " << std::endl;
		}
		else
		{
			std::cout << "No feasible solution for right child! " << std::endl;
		}

		if (!curr->cardinalConf.empty() || !curr->rectCardinalConf.empty()) // Resolve a cardinal conflict
		{
			if (Sol1 && abs(n1->g_val - curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
			if (Sol2 && abs(n2->g_val - curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
		}
		else if (!curr->semiConf.empty() || !curr->rectSemiConf.empty()) // Resolve a semi
		{
			if (Sol1 && Sol2 && abs(n1->g_val + n2->g_val - 2 * curr->g_val) < 0.001)
			{
				std::cout << "***********ERROR**************" << std::endl;
				system("pause");
			}
		}
#endif

		if(!Sol1)
		{
			delete (n1);
			n1 = NULL;
		}
		if(!Sol2)
		{
			delete (n2);
			n2 = NULL;
		}
		//if(curr != dummy_start) // We save dummy_start for statistics analysis later

		curr->clear();
		t1 = std::clock();
		if (open_list.size() == 0) {
			solution_found = false;
			break;
		}
		#ifdef DEBUG
        cout << " ; (after) " << focal_list_threshold << endl << endl;
		#endif
		runtime_listoperation += std::clock() - t1;

	}  // end of while loop while(!open_list.empty() && !solution_found)


	//    printPaths();
	if (open_list.empty() && solution_cost < 0 && !(runtime > TIME_LIMIT)) {  // search fail, not runtime failure


            solution_cost = -2;
            cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
                 HL_num_expanded << " ; " << HL_num_generated << " ; " <<
                 LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " <<
                 "|Open|=" << open_list.size() << endl;
            solution_found = false;

    }

	return solution_found;
}

void GICBSSearch::create_trans_priorities(vector<vector<bool>> priorities_matrix, vector<vector<bool>> *trans_priorities) {
    vector<vector<bool>> trans_priorities_matrix;
    trans_priorities_matrix = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
    for (int i = 0; i < num_of_agents; i++) {
        for (int j = 0; j < num_of_agents; j++) {
            if (priorities_matrix[i][j]){  // ai higher priority than aj
                trans_priorities_matrix[i][j] = true;
                for (int z = 0; z < num_of_agents; z++) {
                    if (priorities_matrix[j][z]){  // az lower priority than aj
                        trans_priorities_matrix[i][z] = true;  // thus ai higher than az
                    }
                }
            }
        }
    }
    *trans_priorities = trans_priorities_matrix;
}

bool GICBSSearch::is_adding_a_higher_than_b_legal(vector<vector<bool>> priorities_matrix, int a_id, int b_id){
    vector<vector<bool>> copy_priorities_matrix (priorities_matrix);
    // if b higher than a - return false
    if (priorities_matrix[b_id][a_id]){
        return false;
    }

    // // add the priority to matrix (make sure it used by value and not reference) and than detect cycles
    // priorities_matrix[a1_id][a2_id] = true; // a1->a2

    // add transitivity priorities

    for (int i = 0; i < num_of_agents; i++) { // ai which is better than a
        if (priorities_matrix[i][a_id] && !priorities_matrix[i][b_id]) {
            for (int j = 0; j < num_of_agents; j++) {  //aj which is lower than b
                if (priorities_matrix[b_id][j]) {

                    if (priorities_matrix[j][i] or priorities_matrix[j][a_id]) {
                        // if j better than i (which better than a) or better than a - return false
                        return false;
                    }
                }
            }
        }
    }

    return true;

}


// GICBSSearch::GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, const vector < vector< bool > > initial_priorities, const vector < vector< bool > > fallback_priorities, const bool use_experience, const bool use_clean, bool fixed_prior): focal_w(f_w), fixed_prior(fixed_prior)
GICBSSearch::GICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, const vector < vector< bool > > initial_priorities, const vector < vector< bool > > fallback_priorities, const int experience, const double fallback, const int width_limit, const int window_size_, bool fixed_prior): focal_w(f_w), fixed_prior(fixed_prior)
{
    window_size = window_size_;
    fallback_option = fallback;
    HL_DFS_width_limit = width_limit;
    experience_strategy = experience;
    if (experience > 0){
        search_with_experience = true;
    }
    else{
        search_with_experience = false;
    }

    if (experience == 2) {  // experience + clean priorities matrix
        clean_experience = true;
    }
    else{
        clean_experience = false;
    }

    // search_with_experience = use_experience;
    // use_original_pbs = !search_with_experience;
	cons_strategy = c;
	//focal_w = f_w;
	HL_num_expanded = 0;
	HL_num_generated = 0;
	LL_num_expanded = 0;
	LL_num_generated = 0;


	this->num_col = ml.cols;
	this->al = al;
	num_of_agents = al.num_of_agents;
	map_size = ml.rows*ml.cols;
	solution_found = false;
	solution_cost = -1;
	//ll_min_f_vals = vector <double>(num_of_agents);
	//paths_costs = vector <double>(num_of_agents);
	//ll_min_f_vals_found_initially = vector <double>(num_of_agents);
	//paths_costs_found_initially = vector <double>(num_of_agents);
	search_engines = vector < SingleAgentICBS* >(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) {
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset, ml.actions_offset, 1.0, &egr);
		search_engines[i] = new SingleAgentICBS(i, init_loc, goal_loc, ml.get_map(), ml.rows*ml.cols,
			ml.moves_offset, ml.cols);
		ch.getHVals(search_engines[i]->my_heuristic);
	}

	// initialize allNodes_table (hash table)
	//empty_node = new GICBSNode();

	//empty_node->time_generated = -2; empty_node->agent_id = -2;
	//deleted_node = new GICBSNode();
	//deleted_node->time_generated = -3; deleted_node->agent_id = -3;
	//allNodes_table.set_empty_key(empty_node);
	//allNodes_table.set_deleted_key(deleted_node);


	dummy_start = new GICBSNode();
	empty_priority_node = new GICBSNode(); ///*************************************///
    fallback_node = new GICBSNode(); ///*************************************///

	dummy_start->agent_id = -1;
    empty_priority_node->agent_id = -1; ///*************************************///
    fallback_node->agent_id = -1; ///*************************************///

                                                // vector of vector (size rows, vector inside(size cols, value col)

	dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  - : initial for false matrix and change it to initial_priority later
    empty_priority_node->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  - initial for false matrix and change it to initial_priority later
    fallback_node->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  - initial for false matrix and change it to initial_priority later

	dummy_start->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  - initial for false matrix and change it to initial_priority later
    empty_priority_node->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  -  initial for false matrix and change it to initial_priority later
    fallback_node->trans_priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false)); //  - initial for false matrix and change it to initial_priority later
    ///*************************************///
	if(fixed_prior) { // this is from original PBS implementation - using fixed priority ordering
        cout << "Running PBS with fixed total priority ordering" << endl;
        for (int i = 0; i < num_of_agents - 1; i++) {
			dummy_start->priorities[i][i + 1] = true;
			// dummy_start->trans_priorities[i][i + 1] = true;
			for (int j = i; j < num_of_agents - 1; j++) {
				dummy_start->trans_priorities[i][j + 1] = true;
				// dummy_start->priorities[i][j + 1] = true;
			}
		}
	}

    /////////////////////////////////////////////////////////////////////
	 else {                                                 // : added given priorities
        dummy_start->trans_priorities = initial_priorities;
        fallback_node->trans_priorities = fallback_priorities;
        dummy_start->priorities = initial_priorities;
        fallback_node->priorities = fallback_priorities;
        // create_trans_priorities(dummy_start->priorities , &dummy_start->trans_priorities);
        // create_trans_priorities(fallback_node->priorities , &fallback_node->trans_priorities);
	 } // end else
    /////////////////////////////////////////////////////////////////////

	// initialize paths_found_initially
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);

	clock_t start_t = std::clock();

	for (int i = 0; i < num_of_agents; i++) { //  - Low-level, find path for each agent
	    /*
		//    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
		//bool* res_table = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
		//bool* res_table_low_prio = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
		//updateReservationTable(res_table, res_table_low_prio, i, *dummy_start);
		//cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
		//printResTable(res_table, max_plan_len);
	     */
	    // cout << "agent " << i << endl;

        size_t max_Plan_length = window_size > 0 ? window_size : dummy_start->makespan + 1;

        if (search_engines[i]->findPath(paths_found_initially[i], f_w, dummy_start->trans_priorities, paths, max_Plan_length, 0) == false) {
            cout << "========"<< endl;
            if (search_with_experience) {
                cout << "NO solution founded with first experience... trying fallback experience..." << endl;
                first_experience_failed = true;
                fallback_option = 0 ;
                HL_DFS_width_limit = -1;
            }
            else {  // original PBS
                // cout << "NO SOLUTION EXISTS";
                // solution_cost = -2;
                if (fixed_prior){
                    cout << "NO SOLUTION FOUNDED with TOTAL PRIORITY, run original PBS instead" << endl;

                    fallback_option = 0 ;
                    HL_DFS_width_limit = -3;   // mark that no solution have founded using total priority ordering
                    fallback_option = 0 ;
                    first_experience_failed = true;
                    // fallback_option == 2;
                }
                else {
                    cout << "NO SOLUTION EXISTS";
                    solution_cost = -2;
                }
            }
            break;
		}
		//dummy_start->paths[i] = search_engines[i]->getPath();
		paths[i] = &paths_found_initially[i];
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
        fallback_node->makespan = max(fallback_node->makespan, paths_found_initially[i].size() - 1);
        empty_priority_node->makespan = max(empty_priority_node->makespan, paths_found_initially[i].size() - 1);
		//search_engines[i]->path.reset();
		//ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
		//paths_costs_found_initially[i] = search_engines[i]->path_cost;
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
		//delete[] res_table;
		//    cout << endl;
	}
    // cout << "success to find paths in root node" << endl;
	if (first_experience_failed) { // try fallback
        // LL_num_expanded = 0;
        // LL_num_generated = 0;

        size_t max_Plan_length_fb_node = window_size > 0 ? window_size : fallback_node->makespan + 1;

        for (int i = 0; i < num_of_agents; i++) { //  - Low-level, find path for each agent
            if (search_engines[i]->findPath(paths_found_initially[i], f_w, fallback_node->trans_priorities, paths, max_Plan_length_fb_node, 0) == false) {
                cout << "NO SOLUTION EXISTS - with fallback experience also";
                solution_cost = -2;
                break;
            }

            paths[i] = &paths_found_initially[i];
            dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
            fallback_node->makespan = max(fallback_node->makespan, paths_found_initially[i].size() - 1);
            empty_priority_node->makespan = max(empty_priority_node->makespan, paths_found_initially[i].size() - 1);
            LL_num_expanded += search_engines[i]->num_expanded;
            LL_num_generated += search_engines[i]->num_generated;

            // only if first_experience_failed = true. make sure that first priorities not used
            dummy_start->priorities = fallback_priorities;
            dummy_start->trans_priorities = fallback_priorities;
        }
	}

	//ll_min_f_vals = ll_min_f_vals_found_initially;
	//paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures

	if (solution_cost != -2) {

		dummy_start->g_val = 0;
        fallback_node->g_val = 0;
        empty_priority_node->g_val = 0;
		for (int i = 0; i < num_of_agents; i++) {
		    dummy_start->g_val += paths[i]->size() - 1;
            empty_priority_node->g_val += paths[i]->size() - 1;
            fallback_node->g_val += paths[i]->size() - 1;
        }
		dummy_start->h_val = 0;
        fallback_node->h_val = 0;
        empty_priority_node->h_val = 0;
		dummy_start->f_val = dummy_start->g_val;
        fallback_node->f_val = fallback_node->g_val;
        empty_priority_node->f_val = empty_priority_node->g_val;
		//dummy_start->ll_min_f_val = 0;
		dummy_start->depth = 0;
        fallback_node->depth = 0;
        empty_priority_node->depth = 0;


		//  - : push first a node with empty initial priority to be used in case of fail to find solution with experience //
		// note - practically, when it fail the search didn't came back to this node...
        open_list.push(empty_priority_node);


        dummy_start->open_handle = open_list.push(dummy_start);
		//dummy_start->focal_handle = focal_list.push(dummy_start);
		//dummy_start->single.resize(num_of_agents);
		//dummy_start->constraints.resize(num_of_agents);
		HL_num_generated++;
		dummy_start->time_generated = HL_num_generated;
		allNodes_table.push_back(dummy_start);
		findConflicts(*dummy_start);
		//initial_g_val = dummy_start->g_val;
		min_f_val = dummy_start->f_val;
		focal_list_threshold = min_f_val * focal_w;

		//  cout << "Paths in START (high-level) node:" << endl;
		//  printPaths();
		// cout << "SUM-MIN-F-VALS: " << dummy_start->sum_min_f_vals << endl;
	}
	pre_runtime = std::clock() - start_t;
}


inline void GICBSSearch::releaseClosedListNodes()
{
	for (list<GICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}


inline void GICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		GICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}


GICBSSearch::~GICBSSearch()
{
	for (size_t i = 0; i < search_engines.size(); i++)
		delete (search_engines[i]);
	//for (size_t i = 0; i < paths_found_initially.size(); i++)
	//	delete (paths_found_initially[i]);
	//  for (size_t i=0; i<paths.size(); i++)
	//    delete (paths[i]);
	releaseClosedListNodes();
	// releaseOpenListNodes();
	//delete (empty_node);
	//delete (deleted_node);
}
