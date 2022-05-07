#include "ICBSSearch.h"
//#define ROOT
//#define DEBUG
//#define STAT


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
inline void ICBSSearch::updatePaths(ICBSNode* curr) {
	for(int i = 0; i < num_of_agents; i++)
		paths[i] = &paths_found_initially[i];
	vector<bool> updated(num_of_agents, false);  // initialized for false
												 /* used for backtracking -- only update paths[i] if it wasn't updated before (that is, by a younger node)
												 * because younger nodes take into account ancesstors' nodes constraints. */
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


vector < list< tuple<int, int, bool> > >* ICBSSearch::collectConstraints(ICBSNode* curr, int agent_id)
{
	std::clock_t t1 = std::clock();
	// extract all constraints on agent_id
	list < tuple<int, int, int, bool> > constraints_positive;  
	list < tuple<int, int, int, bool> > constraints_negative;
	//  cout << "  Find all constraints on him:" << endl;
	int max_timestep = -1;
	//for(list<std::shared_ptr<tuple<int, int, int, bool>>>::iterator it = curr->constraints[agent_id].begin(); it != curr->constraints[agent_id].end(); it++)
	//{
	//	if (get<3>(**it)) // positive constraint
	//		constraints_positive.push_back(**it);
	//	else
	//		constraints_negative.push_back(**it);
	//	if (get<2>(**it) > max_timestep) // calc constraints' max_timestep
	//		max_timestep = get<2>(**it);
	//}
	//if(cons_strategy == constraint_strategy::N_CBSH || cons_strategy == constraint_strategy::N_ICBS)
	//for (int i = 0; i < num_of_agents; i++)
	//{
	//	if(agent_id == i)
	//		continue;
	//	for (list<std::shared_ptr<tuple<int, int, int, bool>>>::iterator it = curr->constraints[i].begin(); it != curr->constraints[i].end(); it++)
	//	{
	//		if (get<3>(**it)) // positive constraint is valid for everyone
	//		{
	//			constraints_negative.push_back(**it);
	//			if (get<2>(**it) > max_timestep) // calc constraints' max_timestep
	//				max_timestep = get<2>(**it);
	//		}
	//	}
	//}
	while (curr != dummy_start) 
	{
		if (get<3>(curr->constraint)) // positive constraint is valid for everyone
		{
			if(curr->agent_id == agent_id) // for the constrained agent, it is a landmark
				constraints_positive.push_back(curr->constraint);
			else // for the other agents, it is equalvelent to a negative constraint
				constraints_negative.push_back(curr->constraint);
			if (get<2>(curr->constraint) > max_timestep) // calc constraints' max_timestep
				max_timestep = get<2>(curr->constraint);
		}
		else if (curr->agent_id == agent_id) // negtive constraint only matters for current agent
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
		if(get<1>(*it) < 0) // vertex constraint
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), -1, true));
		else // edge constraint
		{
			cons_vec->at(get<2>(*it) - 1).push_back(make_tuple(get<0>(*it), -1, true));
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), -1, true));
		}
	}
	for (list< tuple<int, int, int, bool> >::iterator it = constraints_negative.begin(); it != constraints_negative.end(); it++) 
	{
		if(!get<3>(*it)) // it is a negetive constraint for this agent
		{
			if(get<0>(*it) < 0) // rectangle constraint
			{
				int x1 = (-get<0>(*it) - 1) / num_col, y1 = (-get<0>(*it) - 1) % num_col;
				int x2 = get<1>(*it) / num_col, y2 = get<1>(*it) % num_col;
				if(x1 == x2)
				{
					if(y1 < y2)
						for(int i = 0; i <= y2 - y1; i++)
							cons_vec->at(get<2>(*it) - i).push_back(make_tuple(x1 * num_col + y2 - i, -1, false));
					else
						for (int i = 0; i <= y1 - y2; i++)
							cons_vec->at(get<2>(*it) - i).push_back(make_tuple(x1 * num_col + y2 + i, -1, false));

				}
				else // y1== y2
				{
					if (x1 < x2)
						for (int i = 0; i <= x2 - x1; i++)
							cons_vec->at(get<2>(*it) - i).push_back(make_tuple((x2 - i) * num_col + y1, -1, false));
					else
						for (int i = 0; i <= x1 - x2; i++)
							cons_vec->at(get<2>(*it) - i).push_back(make_tuple((x2 + i) * num_col + y1, -1, false));

				}
			}
			else
				cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), get<1>(*it), false));
		}
		else if (get<1>(*it) < 0) // positive vertex constraint for other agent
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<0>(*it), -1, false));
		else // positive edge constraint for other agent
			cons_vec->at(get<2>(*it)).push_back(make_tuple(get<1>(*it), get<0>(*it),  false));
	}
	
	runtime_updatecons += std::clock() - t1;
	return cons_vec;
}

tuple<int, int, int> ICBSSearch::countRectangleConflicts(ICBSNode &root)
{
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	int cardinal = 0, semi = 0, non = 0;
	//Rectangle reasoning
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = root.semiConf.begin(); it != root.semiConf.end(); ++it)
	{
		if (CG[get<0>(**it)][get<1>(**it)]
			|| get<3>(**it) >= 0 // We only consider vertex conflict
			|| paths[get<0>(**it)]->size() <= get<4>(**it) //conflict happends before agent reaches its goal
			|| paths[get<1>(**it)]->size() <= get<4>(**it))
			continue;

		int s1_x = al.initial_locations[get<0>(**it)].first, s1_y = al.initial_locations[get<0>(**it)].second;
		int s2_x = al.initial_locations[get<1>(**it)].first, s2_y = al.initial_locations[get<1>(**it)].second;
		int g1_x = al.goal_locations[get<0>(**it)].first, g1_y = al.goal_locations[get<0>(**it)].second;
		int g2_x = al.goal_locations[get<1>(**it)].first, g2_y = al.goal_locations[get<1>(**it)].second;

		if(!isManhattanOptimal(s1_x * num_col + s1_y, g1_x * num_col +g1_y, paths[get<0>(**it)]->size() - 1)
			|| !isManhattanOptimal(s2_x * num_col + s2_y, g2_x * num_col + g2_y, paths[get<1>(**it)]->size() - 1))
			continue;
		int rst = identifyRectangleConflict(s1_x, s1_y, s2_x, s2_y, g1_x, g1_y, g2_x, g2_y, 0, 0);
		if (rst == 1)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			cardinal++;
		}
		else if (rst == 2)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			semi++;
		}
		else if (rst == 3)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			non++;
		}
	}

	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = root.nonConf.begin(); it != root.nonConf.end(); ++it)
	{
		if (CG[get<0>(**it)][get<1>(**it)]
			|| get<3>(**it) >= 0 // We only consider vertex conflict
			|| paths[get<0>(**it)]->size() <= get<4>(**it) //conflict happends before agent reaches its goal
			|| paths[get<1>(**it)]->size() <= get<4>(**it))
			continue;

		int s1_x = al.initial_locations[get<0>(**it)].first, s1_y = al.initial_locations[get<0>(**it)].second;
		int s2_x = al.initial_locations[get<1>(**it)].first, s2_y = al.initial_locations[get<1>(**it)].second;
		int g1_x = al.goal_locations[get<0>(**it)].first, g1_y = al.goal_locations[get<0>(**it)].second;
		int g2_x = al.goal_locations[get<1>(**it)].first, g2_y = al.goal_locations[get<1>(**it)].second;

		int rst = identifyRectangleConflict(s1_x, s1_y, s2_x, s2_y, g1_x, g1_y, g2_x, g2_y, 0, 0);
		if (rst == 1)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			cardinal++;
	}
		else if (rst == 2)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			semi++;
		}
		else if (rst == 3)
		{

#ifdef DEBUG
			std::cout << "Agent: " << get<0>(**it) << " (" << al.initial_locations[get<0>(**it)].first << "," << al.initial_locations[get<0>(**it)].second
				<< ")-->(" << al.goal_locations[get<0>(**it)].first << "," << al.goal_locations[get<0>(**it)].second << ")" << std::endl;
			std::cout << "Agent: " << get<1>(**it) << " (" << al.initial_locations[get<1>(**it)].first << "," << al.initial_locations[get<1>(**it)].second
				<< ")-->(" << al.goal_locations[get<1>(**it)].first << "," << al.goal_locations[get<1>(**it)].second << ")" << std::endl;
			std::cout << "Conflict (" << get<2>(**it) / num_col << "," << get<2>(**it) % num_col << ")@" << get<4>(**it) << std::endl;
#endif
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			//num_of_CGedges++;
			non++;
		}
	}
	return make_tuple(cardinal, semi, non);
}
int ICBSSearch::computeHeuristics(const ICBSNode& curr)
{
	//if (curr.cardinalConf.size() < 2)
	//	return curr.cardinalConf.size();

	// Conflict graph
	vector<vector<bool>> CG(num_of_agents);
	int num_of_CGnodes = 0, num_of_CGedges = 0;
	for (int i = 0; i < num_of_agents; i++)
		CG[i].resize(num_of_agents, false);
	for (list<std::shared_ptr<tuple<int,int,int,int,int>>>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		if(!CG[get<0>(**it)][get<1>(**it)])
		{
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			num_of_CGedges++;
		}
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.rectCardinalConf.begin(); it != curr.rectCardinalConf.end(); ++it)
	{
		if (!CG[get<0>(**it)][get<1>(**it)])
		{
			CG[get<0>(**it)][get<1>(**it)] = true;
			CG[get<1>(**it)][get<0>(**it)] = true;
			num_of_CGedges++;
		}
	}

	if (num_of_CGedges < 2)
		return num_of_CGedges;

	// Compute #CG nodes that have edges
	for (int i = 0; i < num_of_agents; i++)
	{
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG[i][j])
			{
				num_of_CGnodes++;
				break;
			}
		}
	}

	// Minimum Vertex Cover
	if (curr.parent == NULL) // root node of CBS tree
	{
		for (int i = 1; i < num_of_CGnodes; i++)
			if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, i))
				return i;
		std::cout <<"ERROR!" << std::endl;
	}
	if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val - 1))
		return curr.parent->h_val - 1;
	else if (KVertexCover(CG, num_of_CGnodes, num_of_CGedges, curr.parent->h_val))
		return curr.parent->h_val;
	else
		return curr.parent->h_val + 1;
}


/// <summary>
/// Whether there exists a k-vertex cover solution
/// </summary>
bool ICBSSearch::KVertexCover(const vector<vector<bool>>& CG, int num_of_CGnodes, int num_of_CGedges, int k)
{
	if (num_of_CGedges == 0)
		return true;
	else if (num_of_CGedges > k * num_of_CGnodes - k) 
		return false;

	vector<int> node(2);
	bool flag = true;
	for (int i = 0; i < num_of_agents - 1 && flag; i++) // to find an edge
	{
		for (int j = i + 1; j < num_of_agents && flag; j++)
		{
			if (CG[i][j])
			{
				node[0] = i;
				node[1] = j;
				flag = false;
			}
		}
	}
	for (int i = 0; i < 2; i++)
	{
		vector<vector<bool>> CG_copy(num_of_agents);
		for (int j = 0; j < num_of_agents; j++)
		{
			CG_copy[j].resize(num_of_agents);
			CG_copy.assign(CG.cbegin(), CG.cend());
		}
		int num_of_CGedges_copy = num_of_CGedges;
		for (int j = 0; j < num_of_agents; j++)
		{
			if (CG_copy[node[i]][j])
			{
				CG_copy[node[i]][j] = false;
				CG_copy[j][node[i]] = false;
				num_of_CGedges_copy--;
			}
		}
		if (KVertexCover(CG_copy, num_of_CGnodes - 1, num_of_CGedges_copy, k - 1))
			return true;
	}
	return false;
}


void ICBSSearch::findConflicts(ICBSNode& curr)
{
	if (curr.parent != NULL)
	{
		// Copy from parent
		vector<bool> copy(num_of_agents, true);
		for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
			copy[it->first] = false;
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->rectCardinalConf.begin(); it != curr.parent->rectCardinalConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.rectCardinalConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->rectSemiConf.begin(); it != curr.parent->rectSemiConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.rectSemiConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->rectNonConf.begin(); it != curr.parent->rectNonConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.rectNonConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->cardinalConf.begin(); it != curr.parent->cardinalConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.cardinalConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->semiConf.begin(); it != curr.parent->semiConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.semiConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->nonConf.begin(); it != curr.parent->nonConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.nonConf.push_back(*it);
			}
		}
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.parent->unknownConf.begin(); it != curr.parent->unknownConf.end(); ++it)
		{
			if (copy[get<0>(**it)] && copy[get<1>(**it)])
			{
				curr.unknownConf.push_back(*it);
			}
		}
		// detect new conflicts
		vector<bool> detected(num_of_agents, false);
		for (list<pair<int, vector<PathEntry>>>::const_iterator it = curr.new_paths.begin(); it != curr.new_paths.end(); it++)
		{
			int a1 = it->first;
			if(search_engines[a1]->num_of_conf == 0) // New path does not have conflicts with others before it reaches its goal
			{
				for (int a2 = 0; a2 < num_of_agents; a2++)
				{
					if (detected[a2])
						continue;
					if (paths[a1]->size() + 1 < paths[a2]->size())
					{
						int loc1 = paths[a1]->back().location;
						for (size_t timestep = paths[a1]->size(); timestep < paths[a2]->size(); timestep++)
						{
							int loc2 = paths[a2]->at(timestep).location;
							if (loc1 == loc2)
							{
								curr.unknownConf.push_front(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, -1, timestep))); // It's at least a semi conflict			
							}
						}
					}
				}
				continue;
			}
			detected[a1] = true;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (detected[a2])
					continue;
				size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
				for (size_t timestep = 0; timestep < min_path_length; timestep++)
				{
					int loc1 = paths[a1]->at(timestep).location;
					int loc2 = paths[a2]->at(timestep).location;
					if (loc1 == loc2)
					{
						curr.unknownConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int,int,int,int,int>(a1, a2, loc1, -1, timestep)));
					}
					else if (timestep < min_path_length - 1
						&& loc1 == paths[a2]->at(timestep + 1).location
						&& loc2 == paths[a1]->at(timestep + 1).location)
					{
						curr.unknownConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, loc2, timestep + 1))); // edge conflict
					}
				}
				if (paths[a1]->size() != paths[a2]->size())
				{
					int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
					int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
					int loc1 = paths[a1_]->back().location;
					for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
					{
						int loc2 = paths[a2_]->at(timestep).location;
						if (loc1 == loc2)
						{
							curr.unknownConf.push_front(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int,int,int,int,int>(a1_, a2_, loc1, -1, timestep))); // It's at least a semi conflict			
						}
					}
				}

			}
		}


	}
	else
	{
		//vector<bool> hasConflicts(num_of_agents, false);
		for(int a1 = 0; a1 < num_of_agents ; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
				for (size_t timestep = 0; timestep < min_path_length; timestep++)
				{
					int loc1 = paths[a1]->at(timestep).location;
					int loc2 = paths[a2]->at(timestep).location;
					if (loc1 == loc2)
					{
						curr.unknownConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, -1, timestep)));
						//hasConflicts[a1] = true;
						//hasConflicts[a2] = true;
					}
					else if (timestep < min_path_length - 1
						&& loc1 == paths[a2]->at(timestep + 1).location
						&& loc2 == paths[a1]->at(timestep + 1).location)
					{
						curr.unknownConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, loc1, loc2, timestep + 1)));
						//hasConflicts[a1] = true;
						//hasConflicts[a2] = true;
					}
				}
				if (paths[a1]->size() != paths[a2]->size())
				{
					int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
					int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
					int loc1 = paths[a1_]->back().location;
					for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
					{
						int loc2 = paths[a2_]->at(timestep).location;
						if (loc1 == loc2)
						{
							curr.unknownConf.push_front(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1_, a2_, loc1, -1, timestep))); // It's at least a semi conflict			
							//curr.unknownConf.front()->cost1 = timestep + 1;
							//hasConflicts[a1] = true;
							//hasConflicts[a2] = true;
						}
					}
				}
			}
		}
		/*for (int i = 0; i < num_of_agents; i++)
		{
			if (!hasConflicts[i] && curr.mdds[i] != NULL)
			{
				curr.mdds[i]->numPointers--;
				if(curr.mdds[i]->numPointers == 0)
					delete curr.mdds[i];
				curr.mdds[i] = NULL;
			}
		}*/
	}
}

void ICBSSearch::buildMDD(ICBSNode& curr, int id, int lookahead = 0)
{
#ifdef DEBUG
	std::cout << "Build MDD for Agent " << id << " with " << lookahead << " steps lookahead." << std::endl;
#endif
	MDD * mdd = new MDD();
	//curr.mdds[id] = new MDD();
	vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(&curr, id);
	mdd->buildMDD(*cons_vec, paths[id]->size(), lookahead, *search_engines[id]);
	//curr.mdds[id]->numPointers = 1;
	//curr.single[id] = std::shared_ptr<vector<bool>>(new vector<bool>(mdd->levels.size()));
	for(int i = 0; i < mdd->levels.size(); i++)
			paths[id]->at(i).single = mdd->levels[i].size() == 1;
	/*if(cons_strategy == constraint_strategy::R_CBSH && mdd->levels.size() == paths_found_initially[id]->size())
	{
		curr.mdds[id] = mdd;
		curr.mdds[id]->numPointers = 1;
	}
	else*/
		delete mdd;
	delete cons_vec;
}


std::shared_ptr<tuple<int,int,int,int,int>> ICBSSearch::chooseConflict(ICBSNode &parent)
{
	//Conflict* chosenCon = NULL;
	//findConflicts(parent);
	if (parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; //std::shared_ptr<tuple<int,int,int,int,int>>(new tuple<int,int,int,int,int>(0,0,0,0,-1)); // No conflict

	// Try to find a cardinal conflict in unknownConf
	while(!parent.unknownConf.empty())
	{
			std::shared_ptr<tuple<int,int,int,int,int>> con = parent.unknownConf.front();
			parent.unknownConf.pop_front();
		if (get<3>(*con) >= 0) // Edge conflict
		{
			bool cardinal1 = false, cardinal2 = false;
			if (!paths[get<0>(*con)]->at(0).single) //parent.mdds[con->a1] == NULL)
			{
				buildMDD(parent, get<0>(*con));
			}
			if (!paths[get<1>(*con)]->at(0).single) //parent.mdds[con->a2] == NULL)
			{
				buildMDD(parent, get<1>(*con));
			}
			cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single && paths[get<0>(*con)]->at(get<4>(*con) - 1).single;
			cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single && paths[get<1>(*con)]->at(get<4>(*con) - 1).single;

			if (cardinal1 && cardinal2) // find a cardinal conflict, return immediately
			{
				return con;
			}
			else if (cardinal1 || cardinal2)
			{
				parent.semiConf.push_back(con);
			}
			else
			{
				parent.nonConf.push_back(con);
			}
		}
		else // vertex conflict
		{
			bool cardinal1, cardinal2;
			if (get<4>(*con) >= paths[get<0>(*con)]->size())
				cardinal1 = true;
			else if (!paths[get<0>(*con)]->at(0).single) //parent.mdds[con->a1] == NULL)
			{
				buildMDD(parent, get<0>(*con));
				cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single;
			}
			else
				cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single;
			if (get<4>(*con) >= paths[get<1>(*con)]->size())
				cardinal2 = true;
			else if (!paths[get<1>(*con)]->at(0).single) //parent.mdds[con->a2] == NULL)
			{
				buildMDD(parent, get<1>(*con));
				cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single;
			}
			else
				cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single;


			if (cardinal1 && cardinal2) //cardinal
			{
				return con;
			}
			else if (cardinal1 || cardinal2) //semi (1st agent)
			{
				parent.semiConf.push_back(con);
				continue;
			}
			else //non
			{
				parent.nonConf.push_back(con);
				continue;
			}
		}
	}

	if (!parent.semiConf.empty()) // Choose the earliest semi
	{
		std::shared_ptr<tuple<int, int, int, int, int>> choose = parent.semiConf.front();
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.semiConf.begin(); it != parent.semiConf.end(); ++it)
			if (get<4>(**it) < get<4>(*choose))
				choose = (*it);
		return choose;
	}
	else // Choose the earliest non
	{
		std::shared_ptr<tuple<int, int, int, int, int>> choose = parent.nonConf.front();
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.nonConf.begin(); it != parent.nonConf.end(); ++it)
			if (get<4>(**it) < get<4>(*choose))
				choose = (*it);
		return choose;
	}


}


// Return 0 if it is not a rectnagle conflict
// Return 1 if it is a cardinal rectangle conflict
// Return 2 if it is a semi-cardinal rectangle conflict
// Return 3 if it is a non-cardinal rectangle conflict
int ICBSSearch::identifyRectangleConflict(int s1_x, int s1_y, int s2_x, int s2_y, int g1_x, int g1_y, int g2_x, int g2_y, int t1, int t2)
{
	if (s1_x == s2_x && s1_y == s2_y) // A standard cardinal conflict
		return 0;
	else if (s1_x == g1_x && s1_y == g1_y) // s1 = g1
		return 0;
	else if (s2_x == g2_x && s2_y == g2_y) // s2 = g2
		return 0;
	else if ((s1_x - g1_x) * (s2_x - g2_x) < 0 || (s1_y - g1_y) * (s2_y - g2_y) < 0) // Not move in the same direction
		return 0;
	else if ((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) // s1 always in the middle
		return 0;
	else if ((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) // s2 always in the middle
		return 0;
	
	bool cardinal1 = false;
	bool cardinal2 = false;
	//if (s1_x != s2_x && s1_y != s2_y)
	//{
	//	if ((s1_x - s2_x) * (g2_x - g1_x) >= 0)
	//		cardinal1 = true;
	//	if ((s1_y - s2_y) * (g2_y - g1_y) >= 0)
	//		cardinal2 = true;
	//	if (cardinal1 && cardinal2)
	//		return 1;
	//	else if (cardinal1 || cardinal2)
	//		return 2;
	//	else
	//		return 3;
	//}
	
	int con_x, con_y; //con1_x, con1_y, con2_x, con2_y;
	if(s1_x == g1_x)
		con_x = g1_x;
	else if(s2_x == g2_x)
		con_x = g2_x;
	else if (s1_x < g1_x)
		con_x = min(g1_x, g2_x);
	else
		con_x = max(g1_x, g2_x);
	if (s1_y == g1_y)
		con_y = g1_y;
	else if (s2_y == g2_y)
		con_y = g2_y;
	else if (s1_y < g1_y)
		con_y = min(g1_y, g2_y);
	else
		con_y = max(g1_y, g2_y);
	if (s1_x == s2_x)
	{
		if ((s1_y - s2_y) * (s2_y - con_y) >= 0)
		{
			//con1_x = s1_x;
			//con2_x = con_x;
			//con1_y = con_y;
			//con2_y = s2_y;
			if(con_x == g1_x)
				cardinal1 = true;
			if(con_y == g2_y)
				cardinal2 = true;
		}
		else
		{
			//con1_x = con_x;
			//con2_x = s2_x;
			//con1_y = s1_y;
			//con2_y = con_y;
			if (con_y == g1_y)
				cardinal1 = true;
			if (con_x == g2_x)
				cardinal2 = true;
		}
	}
	else if ((s1_x - s2_x)*(s2_x - con_x) >= 0)
	{
		//con1_x = con_x;
		//con2_x = s2_x;
		//con1_y = s1_y;
		//con2_y = con_y;
		if (con_y == g1_y)
			cardinal1 = true;
		if (con_x == g2_x)
			cardinal2 = true;
	}
	else
	{
		//con1_x = s1_x;
		//con2_x = con_x;
		//con1_y = con_y;
		//con2_y = s2_y;
		if (con_x == g1_x)
			cardinal1 = true;
		if (con_y == g2_y)
			cardinal2 = true;

	}

	if (cardinal1 && cardinal2)
		return 1;
	else if (cardinal1 || cardinal2)
		return 2;
	else
		return 3;
	
	
	//if (((s1_x < s2_x && s2_x <= g2_x && g2_x <= g1_x) ||
	//	(s1_x > s2_x && s2_x >= g2_x && g2_x >= g1_x) ||
	//	(s2_x < s1_x && s1_x <= g1_x && g1_x <= g2_x) ||
	//	(s2_x > s1_x && s1_x >= g1_x && g1_x >= g2_x)) &&
	//	((s1_y < s2_y && s2_y <= g2_y && g2_y <= g1_y) ||
	//	(s1_y > s2_y && s2_y >= g2_y && g2_y >= g1_y) ||
	//		(s2_y < s1_y && s1_y <= g1_y && g1_y <= g2_y) ||
	//		(s2_y > s1_y && s1_y >= g1_y && g1_y >= g2_y)))
	//		return true;
	//else
	//		return false;
}

inline bool ICBSSearch::isManhattanOptimal(int loc1, int loc2, int dist)
{
	return abs(loc1 / num_col -loc2 / num_col) + abs(loc1 % num_col - loc2 % num_col) == dist;
}


inline bool ICBSSearch::EqualRectConf(const tuple<int,int,int,int,int>& RectConf, const tuple<int, int, int, int, int>& conf)
{
	if(get<2>(RectConf) != get<2>(conf)) //Not same vertex C
		return false;
	else if((get<0>(RectConf) == get<0>(conf) && get<1>(RectConf) == get<1>(conf)) ||
		(get<0>(RectConf) == get<1>(conf) && get<1>(RectConf) == get<0>(conf))) // Same set of agents
		return true;
	else
		return false;
}

std::shared_ptr<tuple<int,int,int,int,int>> ICBSSearch::classifyConflicts(ICBSNode &parent)
{
	if (parent.rectCardinalConf.empty() && parent.rectSemiConf.empty() && parent.rectNonConf.empty() &&
		parent.cardinalConf.empty() && parent.semiConf.empty() && parent.nonConf.empty() && parent.unknownConf.empty())
		return NULL; //make_tuple(0,0,0,0,-1); // No conflict

	// Classify all conflicts in unknownConf
	while(!parent.unknownConf.empty())
	{
			std::shared_ptr<tuple<int,int,int,int,int>> con = parent.unknownConf.front();
			parent.unknownConf.pop_front();

			if(get<3>(*con) >= 0) // Edge conflict
			{
				bool cardinal1 = false, cardinal2 = false;
				if (!paths[get<0>(*con)]->at(0).single) //parent.mdds[con->a1] == NULL)
				{
					buildMDD(parent, get<0>(*con));
				}
				if (!paths[get<1>(*con)]->at(0).single) //parent.mdds[con->a2] == NULL)
				{
					buildMDD(parent, get<1>(*con));
				}
				cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single && paths[get<0>(*con)]->at(get<4>(*con) - 1).single;
				cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single && paths[get<1>(*con)]->at(get<4>(*con) - 1).single;

				if(cardinal1 && cardinal2) // find a cardinal conflict, return immediately
				{
					parent.cardinalConf.push_back(con);
				}
				else if(cardinal1 || cardinal2)
				{
					parent.semiConf.push_back(con);
				}
				else
				{
					parent.nonConf.push_back(con);
				}
			}
			else // vertex conflict
			{
				int a1 = get<0>(*con);
				int a2 = get<1>(*con);
				int t = get<4>(*con);

				bool cardinal1, cardinal2;
				if (get<4>(*con) >= paths[get<0>(*con)]->size())
					cardinal1 = true;
				else if(!paths[get<0>(*con)]->at(0).single) //parent.mdds[con->a1] == NULL)
				{
					buildMDD(parent, get<0>(*con));
					cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single;
				}
				else
					cardinal1 = paths[get<0>(*con)]->at(get<4>(*con)).single;
				if (get<4>(*con) >= paths[get<1>(*con)]->size())
					cardinal2 = true;
				else if (!paths[get<1>(*con)]->at(0).single) //parent.mdds[con->a2] == NULL)
				{
					buildMDD(parent, get<1>(*con));
					cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single;
				}
				else
					cardinal2 = paths[get<1>(*con)]->at(get<4>(*con)).single;


				if(cardinal1 && cardinal2) //cardinal
				{
					parent.cardinalConf.push_back(con);
					continue;
				}
				else if (cardinal1 || cardinal2)
				{
					parent.semiConf.push_back(con);
				}
				else //non
				{
					parent.nonConf.push_back(con);
				}

				//Rectangle reasoning for semi and non cardinal conflicts
				if(paths[get<0>(*con)]->size() <= get<4>(*con) || paths[get<1>(*con)]->size() <= get<4>(*con))//conflict happens after agent reaches its goal
					continue;
				if (cons_strategy == constraint_strategy::CBSH_CR)
				{
					//Identify cardinal rectangle by start and goals
					int s1_x = al.initial_locations[a1].first, s1_y = al.initial_locations[a1].second;
					int s2_x = al.initial_locations[a2].first, s2_y = al.initial_locations[a2].second;
					int g1_x = al.goal_locations[a1].first, g1_y = al.goal_locations[a1].second;
					int g2_x = al.goal_locations[a2].first, g2_y = al.goal_locations[a2].second;
					if (paths[a1]->size() - 1 == abs(s1_x - g1_x) + abs(s1_y - g1_y) && paths[a2]->size() - 1 == abs(s2_x - g2_x) + abs(s2_y - g2_y) && // Manhattan-optimal
						paths[a1]->size() > t && paths[a2]->size() > t && //conflict happens before agent reaches its goal
						(s1_x - g1_x) * (s2_x - g2_x) >= 0 && (s1_y - g1_y) * (s2_y - g2_y) >= 0 && //Move in the same direction
						(s1_x - s2_x) * (g1_x - g2_x) <= 0 && (s1_y - s2_y) * (g1_y - g2_y) <= 0 //Crossover -- cardinal requirement
						)
					{
						int x, y;
						if (s1_x == g1_x)
							x = s1_x;
						else if (s2_x == g2_x)
							x = s2_x;
						else if (s1_x < g1_x)
							x = min(g1_x, g2_x);
						else
							x = max(g1_x, g2_x);
						if (s1_y == g1_y)
							y = s1_y;
						else if (s2_y == g2_y)
							y = s2_y;
						else if (s1_y < g1_y)
							y = min(g1_y, g2_y);
						else
							y = max(g1_y, g2_y);
						int x2, y2;
						if (s1_x == x)
							x2 = s1_x;
						else if (s1_x < x)
							x2 = max(s1_x, s2_x);
						else
							x2 = min(s1_x, s2_x);
						if (s1_y == y)
							y2 = s1_y;
						else if (s1_y < y)
							y2 = max(s1_y, s2_y);
						else
							y2= min(s1_y, s2_y);
						if(x != x2 || y != y2) // The rectangle area is larger than 1	
							parent.rectCardinalConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, -1 - x * num_col - y, 0, 0)));
					}
				}
				else if (cons_strategy == constraint_strategy::CBSH_R)
				{
					//Identify rectangles by start and goals
					int s1_x = al.initial_locations[a1].first, s1_y = al.initial_locations[a1].second;
					int s2_x = al.initial_locations[a2].first, s2_y = al.initial_locations[a2].second;
					int g1_x = al.goal_locations[a1].first, g1_y = al.goal_locations[a1].second;
					int g2_x = al.goal_locations[a2].first, g2_y = al.goal_locations[a2].second;
					if (paths[a1]->size() - 1 == abs(s1_x - g1_x) + abs(s1_y - g1_y) && paths[a2]->size() - 1 == abs(s2_x - g2_x) + abs(s2_y - g2_y) && // Manhattan-optimal
						paths[a1]->size() > t && paths[a2]->size() > t && //conflict happens before agent reaches its goal
						(s1_x - g1_x) * (s2_x - g2_x) >= 0 && (s1_y - g1_y) * (s2_y - g2_y) >= 0) //Move in the same direction
					{
						bool p1, p2;
						if ((s1_x - s2_x) * (g1_x - g2_x) <= 0)
							p1 = true;
						else
							p1 = false;
						if ((s1_y - s2_y) * (g1_y - g2_y) <= 0)
							p2 = true;
						else
							p2 = false;
						int x, y;
						if (s1_x == g1_x)
							x = s1_x;
						else if (s2_x == g2_x)
							x = s2_x;
						else if (s1_x < g1_x)
							x = min(g1_x, g2_x);
						else
							x = max(g1_x, g2_x);
						if (s1_y == g1_y)
							y = s1_y;
						else if (s2_y == g2_y)
							y = s2_y;
						else if (s1_y < g1_y)
							y = min(g1_y, g2_y);
						else
							y = max(g1_y, g2_y);
						int x2, y2;
						if (s1_x == x)
							x2 = s1_x;
						else if (s1_x < x)
							x2 = max(s1_x, s2_x);
						else
							x2 = min(s1_x, s2_x);
						if (s1_y == y)
							y2 = s1_y;
						else if (s1_y < y)
							y2 = max(s1_y, s2_y);
						else
							y2 = min(s1_y, s2_y);
						if (x != x2 || y != y2) // The rectangle area is larger than 1	
						{
							if (p1 && p2) // cardinal rectangle
							{
								parent.rectCardinalConf.push_back(std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(a1, a2, -1 - x * num_col - y, 0, 0)));
							}
							else
							{
								//This conflict should be new
								ICBSNode* curr = parent.parent;
								bool isNew = true;
								std::shared_ptr<tuple<int, int, int, int, int >> new_con = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - x * num_col - y, 0, 0));
								while (curr != NULL && isNew)
								{
									if (EqualRectConf(*new_con,*curr->conflict))
										isNew = false;
									curr = curr->parent;
								}
								if (isNew)
								{
									if (p1 || p2) // semi-cardinal-rectangle
										parent.rectSemiConf.push_back(new_con);
									else // non-cardinal-rectangle
										parent.rectNonConf.push_back(new_con);
								}
							}
						}
					}
				}
				else if(cons_strategy == constraint_strategy::CBSH_RM)
				{
					vector<list<int>> t_starts(2), t_ends(2);
					vector<int> ag(2);
					ag[0] = get<0>(*con);
					ag[1] = get<1>(*con);
					for (int i = 0; i < 2; i++) 
					{
						int start = get<4>(*con), end = get<4>(*con);
						for (int t = 0; t <= get<4>(*con); t++) //Find start that is single and Manhattan-optimal to conflicting location
						{
							if (paths[ag[i]]->at(t).single && isManhattanOptimal(paths[ag[i]]->at(t).location, get<2>(*con), get<4>(*con) - t))
								t_starts[i].push_back(t);
						}
						for (int t = paths[ag[i]]->size() - 1; t >= get<4>(*con); t--) //Find end that is single and Manhattan-optimal to conflicting location
						{
							if (paths[ag[i]]->at(t).single && isManhattanOptimal(paths[ag[i]]->at(t).location, get<2>(*con), t - get<4>(*con)))
								t_ends[i].push_back(t);
						}
					}

					// Try all possible combinations
					bool found = false;
					std::shared_ptr<tuple<int, int, int, int, int>> con_add;
					int type = 0, t1, t2;
					for (list<int>::iterator t1_start = t_starts[0].begin(); t1_start != t_starts[0].end() && !found; t1_start++)
					{
						for (list<int>::iterator t1_end = t_ends[0].begin(); t1_end != t_ends[0].end() && !found; t1_end++)
						{
							if (!isManhattanOptimal(paths[ag[0]]->at(*t1_start).location , paths[ag[0]]->at(*t1_end).location , *t1_end - *t1_start))
								continue;
							for (list<int>::iterator t2_start = t_starts[1].begin(); t2_start != t_starts[1].end() && !found; t2_start++)
							{
								for (list<int>::iterator t2_end = t_ends[1].begin(); t2_end != t_ends[1].end() && !found; t2_end++)
								{
									if (!isManhattanOptimal(paths[ag[1]]->at(*t2_start).location, paths[ag[1]]->at(*t2_end).location, *t2_end - *t2_start))
										continue;
									int s1_x = paths[get<0>(*con)]->at(*t1_start).location / num_col, s1_y = paths[get<0>(*con)]->at(*t1_start).location % num_col;
									int s2_x = paths[get<1>(*con)]->at(*t2_start).location / num_col, s2_y = paths[get<1>(*con)]->at(*t2_start).location % num_col;
									int g1_x = paths[get<0>(*con)]->at(*t1_end).location / num_col, g1_y = paths[get<0>(*con)]->at(*t1_end).location % num_col;
									int g2_x = paths[get<1>(*con)]->at(*t2_end).location / num_col, g2_y = paths[get<1>(*con)]->at(*t2_end).location % num_col;
									int new_type = identifyRectangleConflict(s1_x, s1_y, s2_x, s2_y, g1_x, g1_y, g2_x, g2_y, *t1_start, *t2_start);
									if(new_type == 0)
										continue;
									int x, y; //Vertex C
									if(s1_x == g1_x)
										x = s1_x;
									else if (s2_x == g2_x)
										x = s2_x;
									else if (s1_x < g1_x)
										x = min(g1_x, g2_x);
									else
										x = max(g1_x, g2_x);
									if (s1_y == g1_y)
										y = s1_y;
									else if (s2_y == g2_y)
										y = s2_y;
									else if (s1_y < g1_y)
										y = min(g1_y, g2_y);
									else
										y = max(g1_y, g2_y);
									int x2, y2;
									if (s1_x == x)
										x2 = s1_x;
									else if (s1_x < x)
										x2 = max(s1_x, s2_x);
									else
										x2 = min(s1_x, s2_x);
									if (s1_y == y)
										y2 = s1_y;
									else if (s1_y < y)
										y2 = max(s1_y, s2_y);
									else
										y2 = min(s1_y, s2_y);
									if (x == x2 && y == y2) // The rectangle area is only 1
										continue;
									if (new_type == 1)
									{
										con_add = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - x * num_col - y, *t1_start, *t2_start));
										type = 1;
										//parent.rectCardinalConf.push_back(); 
										found = true;
									}
									else if (new_type == 2)
									{
										if (type == 0 || type == 3 || (type == 2 && get<3>(*con_add) >= *t1_start && get<4>(*con_add) >= *t2_start &&  t1 <= *t1_end &&  t2 <= *t2_end))
										{
											con_add  = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - x * num_col - y, *t1_start, *t2_start));
											t1 = *t1_end;
											t2 = *t2_end;
											type = 2;
										}
									
									}
									else if (new_type == 3)
									{
										if (type == 0 || (type == 3 && get<3>(*con_add) >= *t1_start && get<4>(*con_add) >= *t2_start &&  t1 <= *t1_end &&  t2 <= *t2_end))
										{
											con_add = std::shared_ptr<tuple<int, int, int, int, int>>(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - x * num_col - y, *t1_start, *t2_start));
											type = 3;
											t1 = *t1_end;
											t2 = *t2_end;
										}
									}
								}
							}
						}
					}
					if(type == 1)
					{
						parent.rectCardinalConf.push_back(con_add);
						continue;
					}
					else if(type == 2)
					{
						//This conflict should be new
						ICBSNode* curr = parent.parent;
						bool isNew = true;
						while (curr != NULL && isNew)
						{
							if (EqualRectConf(*con_add, *curr->conflict))
								isNew = false;
							curr = curr->parent;
						}
						if (isNew)
						{
							parent.rectSemiConf.push_back(con_add);
						}
					}
					else if(type ==3)
					{
						//This conflict should be new
						ICBSNode* curr = parent.parent;
						bool isNew = true;
						while (curr != NULL && isNew)
						{
							if (EqualRectConf(*con_add, *curr->conflict))
								isNew = false;
							curr = curr->parent;
						}
						if (isNew)
						{
							parent.rectNonConf.push_back(con_add);
						}
					}
				}
			}
	}

	std::shared_ptr<tuple<int, int, int, int, int>> choose;
	int time = INT_MAX;
	if (!parent.rectCardinalConf.empty()) // Choose the earliest rect cardinal
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectCardinalConf.begin(); it != parent.rectCardinalConf.end(); ++it)
		{	
			int s1_x = paths[get<0>(**it)]->at(get<3>(**it)).location / num_col, s1_y = paths[get<0>(**it)]->at(get<3>(**it)).location % num_col;
			int s2_x = paths[get<1>(**it)]->at(get<4>(**it)).location / num_col, s2_y = paths[get<1>(**it)]->at(get<4>(**it)).location % num_col;
			int c_x = get<2>(**it) / num_col, c_y = get<2>(**it) % num_col;
			int x, y;
			if(s1_x == c_x)
				x = s1_x;
			else if(s1_x < c_x)
				x = max(s1_x, s2_x);
			else
				x = min(s1_x, s2_x);
			if (s1_y == c_y)
				y = s1_y;
			else if (s1_y < c_y)
				y = max(s1_y, s2_y);
			else
				y = min(s1_y, s2_y);
			int new_time = get<3>(**it) + abs(s1_x - x) + abs(s1_y - y);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (!parent.cardinalConf.empty()) // or choose the earliest cardinal
	{
		if(time == INT_MAX)
			choose = parent.cardinalConf.front();
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.cardinalConf.begin(); it != parent.cardinalConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
	}
	if (time < INT_MAX)
		return choose;
	
	if (!parent.rectSemiConf.empty()) // Choose the earliest semi rect
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectSemiConf.begin(); it != parent.rectSemiConf.end(); ++it)
		{
			int s1_x = paths[get<0>(**it)]->at(get<3>(**it)).location / num_col, s1_y = paths[get<0>(**it)]->at(get<3>(**it)).location % num_col;
			int s2_x = paths[get<1>(**it)]->at(get<4>(**it)).location / num_col, s2_y = paths[get<1>(**it)]->at(get<4>(**it)).location % num_col;
			int c_x = get<2>(**it) / num_col, c_y = get<2>(**it) % num_col;
			int x, y;
			if (s1_x == c_x)
				x = s1_x;
			else if (s1_x < c_x)
				x = max(s1_x, s2_x);
			else
				x = min(s1_x, s2_x);
			if (s1_y == c_y)
				y = s1_y;
			else if (s1_y < c_y)
				y = max(s1_y, s2_y);
			else
				y = min(s1_y, s2_y);
			int new_time = get<3>(**it) + abs(s1_x - x) + abs(s1_y - y);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.semiConf.empty()) // Choose the earliest semi
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.semiConf.begin(); it != parent.semiConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
		return choose;
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.rectNonConf.empty()) // Choose a rect randomly
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.rectNonConf.begin(); it != parent.rectNonConf.end(); ++it)
		{
			int s1_x = paths[get<0>(**it)]->at(get<3>(**it)).location / num_col, s1_y = paths[get<0>(**it)]->at(get<3>(**it)).location % num_col;
			int s2_x = paths[get<1>(**it)]->at(get<4>(**it)).location / num_col, s2_y = paths[get<1>(**it)]->at(get<4>(**it)).location % num_col;
			int c_x = get<2>(**it) / num_col, c_y = get<2>(**it) % num_col;
			int x, y;
			if (s1_x == c_x)
				x = s1_x;
			else if (s1_x < c_x)
				x = max(s1_x, s2_x);
			else
				x = min(s1_x, s2_x);
			if (s1_y == c_y)
				y = s1_y;
			else if (s1_y < c_y)
				y = max(s1_y, s2_y);
			else
				y = min(s1_y, s2_y);
			int new_time = get<3>(**it) + abs(s1_x - x) + abs(s1_y - y);
			if (new_time < time)
			{
				choose = (*it);
				time = new_time;
			}
		}
	}
	if (time < INT_MAX)
		return choose;
	if (!parent.nonConf.empty())// Choose the earliest non
	{
		for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::iterator it = parent.nonConf.begin(); it != parent.nonConf.end(); ++it)
			if (get<4>(**it) < time)
			{
				choose = (*it);
				time = get<4>(**it);
			}
	}
	return choose;
}


/*
return agent_id's location for the given timestep
Note -- if timestep is longer than its plan length,
then the location remains the same as its last cell)
*/
inline int ICBSSearch::getAgentLocation(const ICBSNode & node, int agent_id, size_t timestep) {
	// if last timestep > plan length, agent remains in its last location
	if (timestep >= paths[agent_id]->size())
		return paths[agent_id]->at(paths[agent_id]->size() - 1).location;
	// otherwise, return its location for that timestep
	return paths[agent_id]->at(timestep).location;
}


/*inline bool ICBSSearch::updateICBSNode(ICBSNode* leaf_node, ICBSNode* root_node)
{
	if (get<3>(leaf_node->constraint)) // positive constraint, all other agents that conflict with current agent need to replan their paths.
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == leaf_node->agent_id)
				continue;
			else if (paths[ag]->at(get<2>(leaf_node->constraint)).location == get<0>(leaf_node->constraint))
			{
				// extract all constraints on leaf_node->agent_id
				int agent_id = ag;
				ICBSNode* curr = leaf_node;
				vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(curr, agent_id);
				// build reservation table
				size_t max_plan_len = getPathsMaxLength();
				bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
				updateReservationTable(res_table, max_plan_len, agent_id);
				// find a path w.r.t cons_vec (and prioretize by res_table).
				bool foundSol = search_engines[agent_id]->findPath(focal_w, cons_vec, res_table, max_plan_len);
				LL_num_expanded += search_engines[agent_id]->num_expanded;
				LL_num_generated += search_engines[agent_id]->num_generated;
				if (foundSol)
				{
					leaf_node->path = vector<pathEntry>(*(search_engines[agent_id]->getPath()));
					leaf_node->ll_min_f_val = search_engines[agent_id]->min_f_val;
					leaf_node->path_cost = search_engines[agent_id]->path_cost;
					delete (cons_vec);
					delete[] res_table;
				}
				else
				{
					delete (cons_vec);
					delete[] res_table;
					return false;
				}
			}
		}
	}
	else
	{
		// extract all constraints on leaf_node->agent_id
		int agent_id = leaf_node->agent_id;
		ICBSNode* curr = leaf_node;
		vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(curr, agent_id);


		// build reservation table
		size_t max_plan_len = getPathsMaxLength();
		bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
		updateReservationTable(res_table, max_plan_len, agent_id);
		// find a path w.r.t cons_vec (and prioretize by res_table).
		bool foundSol = search_engines[agent_id]->findPath(focal_w, cons_vec, res_table, max_plan_len);
		LL_num_expanded += search_engines[agent_id]->num_expanded;
		LL_num_generated += search_engines[agent_id]->num_generated;

		if (foundSol)
		{
			leaf_node->path = vector<pathEntry>(*(search_engines[agent_id]->getPath()));
			leaf_node->ll_min_f_val = search_engines[agent_id]->min_f_val;
			leaf_node->path_cost = search_engines[agent_id]->path_cost;
		}
		delete (cons_vec);
		delete[] res_table;
		return foundSol;
	}
}*/


/*
void ICBSSearch::generateChildwithCurrentCost(ICBSNode* n1, const ICBSNode* curr)
{
	// Copy MDDs from parent
	for (int i = 0; i < curr->mdds.size(); i++)
	{
		n1->mdds[i] = NULL;
		continue;
		if (i == n1->agent_id)
		{
			if (curr->mdds[i] == NULL)
			{
				n1->mdds[i] = NULL;
			}
			else
			{
				n1->mdds[i] = new MDD(*curr->mdds[i]);
				n1->mdds[i]->updateMDD(n1->constraint, num_col);
				n1->mdds[i]->numPointers = 1;
			}
		}
		else if(curr->mdds[i] != NULL)
		{
			n1->mdds[i] = curr->mdds[i];
			n1->mdds[i]->numPointers++;
		}
		else
		{
			n1->mdds[i] = NULL;
		}
	}

	size_t max_plan_len = getPathsMaxLength();
	bool* res_table = new bool[map_size * max_plan_len];  // initialized to false
	updateReservationTable(res_table, max_plan_len, n1->agent_id);
	bool foundSol = search_engines[n1->agent_id]->findPathByMDD(res_table, *n1->mdds[n1->agent_id]);
	if (foundSol)
	{
		n1->path = vector<pathEntry>(*(search_engines[n1->agent_id]->getPath()));
		n1->ll_min_f_val = search_engines[n1->agent_id]->min_f_val;
		n1->path_cost = search_engines[n1->agent_id]->path_cost;
		// new g_val equals old g_val plus the new path length found for the agent minus its old path length
		n1->g_val = curr->g_val;
		n1->ll_min_f_val = n1->path_cost;
		n1->sum_min_f_vals = curr->sum_min_f_vals;// update lower bounds and handles
		// update n1's path for computing conflicts
		vector<pathEntry>* temp_old_path = paths[n1->agent_id];
		paths[n1->agent_id] = &(n1->path);
		findConflicts(*n1);
		n1->num_of_collisions = n1->unknownConf.size() + n1->cardinalConf.size() + n1->nonConf.size() + n1->semiConf.size();//computeNumOfCollidingAgents();
		paths[n1->agent_id] = temp_old_path;  // restore the old path (for n2)	
		n1->open_handle = open_list.push(n1);
		HL_num_generated++;
		n1->time_generated = HL_num_generated;
		n1->focal_handle = focal_list.push(n1); //cost not changed, so push it into focal_list immediately
		//allNodes_table[n1] = n1;
		allNodes_table.push_back(n1);

		/*for (list<Conflict*>::iterator it = n1->semiConf.begin(); it != n1->semiConf.end(); ++it)
			delete (*it);
		for (list<Conflict*>::iterator it = n1->nonConf.begin(); it != n1->nonConf.end(); ++it)
			delete (*it);
		for (list<Conflict*>::iterator it = n1->unknownConf.begin(); it != n1->unknownConf.end(); ++it)
			delete (*it);
		n1->semiConf.clear();
		n1->nonConf.clear();
		n1->unknownConf.clear();

#ifdef DEBUG
		if (abs(n1->g_val - n1->sum_min_f_vals) > 0.001)
			cout << "h_val=" << "n1->sum_min_f_vals -  n1->g_val" << endl;
#endif
		//#ifndef NDEBUG
		//	cout << endl << "   First node generated for A" << n1->agent_id << ": g-val=" << n1->g_val << " ; h-val=" << n1->num_of_collisions << " ; LB=" << n1->sum_min_f_vals << endl;
		//#endif
	}
	else {
		std::cout << "***ERROR!*** Failed to find solution of non-cardinal conflict!" << std::endl;
		delete (n1);
		n1 = NULL;
	}
	delete[] res_table;
}
*/


bool ICBSSearch::findPathForSingleAgent(ICBSNode*  node, int ag, double lowerbound)
{
	// extract all constraints on agent ag
	ICBSNode* curr = node;
	vector < list< tuple<int, int, bool> > >* cons_vec = collectConstraints(curr, ag);
	// build reservation table
	size_t max_plan_len = node->makespan + 1; //getPathsMaxLength();
	bool* res_table = new bool[map_size * max_plan_len]();  // initialized to false
	updateReservationTable(res_table, ag, *node);
	// find a path w.r.t cons_vec (and prioretize by res_table).
	pair<int,vector<PathEntry>> newPath;
	newPath.first = ag;
	bool foundSol = search_engines[ag]->findPath(newPath.second, focal_w, cons_vec, res_table, max_plan_len, lowerbound);
	LL_num_expanded += search_engines[ag]->num_expanded;
	LL_num_generated += search_engines[ag]->num_generated;
	delete (cons_vec);
	delete[] res_table;
	if (foundSol)
	{
#ifdef STAT
		//std::cout << paths[ag]->size() - 1 << " ; " << newPath.second.size() - 1 << " ; " << search_engines[ag]->num_expanded << " ; " << search_engines[ag]->num_generated << std::endl;
		node_stat.push_back(make_tuple(HL_num_generated, paths_found_initially[ag].size() - 1, paths[ag]->size() - 1, newPath.second.size() - 1, search_engines[ag]->num_expanded, search_engines[ag]->num_generated));
#endif
		

		node->new_paths.push_back(newPath);
		node->g_val = node->g_val - paths[ag]->size() + newPath.second.size();
		paths[ag] = &node->new_paths.back().second;
		//node->paths[ag] = search_engines[ag]->getPath();
		node->makespan = max(node->makespan, newPath.second.size() - 1);
		//search_engines[ag]->path.reset();
		return true;
	}
	else
	{
		//delete node;
		return false;
	}
}

bool ICBSSearch::generateChild(ICBSNode*  node, ICBSNode* curr)
{
	node->parent = curr;
	node->g_val = curr->g_val;
	node->makespan = curr->makespan;
	//node->f_val = curr->f_val - curr->h_val;
	node->depth = curr->depth + 1;
	//node->paths = curr->paths;
	//node->paths.resize(num_of_agents);
	//node->paths.assign(curr->paths.begin(), curr->paths.end());
	//node->single.resize(num_of_agents, NULL);
	std::clock_t t1;

	t1 = std::clock();

	if (get<3>(node->constraint)) //positve constraint
	{
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == node->agent_id)
				continue;
			else if (get<1>(node->constraint) < 0 && // vertex constraint
				getAgentLocation(*node, ag, get<2>(node->constraint)) == get<0>(node->constraint))
			{
				if (!findPathForSingleAgent(node, ag))
					return false;
				//else
				//	node->paths[ag] = &(get<1>(node->paths_updated.back()));
			}
			else if (get<1>(node->constraint) >= 0 && //edge constraint
				getAgentLocation(*node, ag, get<2>(node->constraint) - 1) == get<1>(node->constraint) &&
				getAgentLocation(*node, ag, get<2>(node->constraint)) == get<0>(node->constraint))
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
		
		if (!findPathForSingleAgent(node, node->agent_id, 0))
			return false;
		//else
		//	paths[node->agent_id] = &(get<1>(node->paths_updated.back()));
	}
	
	runtime_lowlevel += std::clock() - t1;
	
	//Estimate h value
	if (node->parent->g_val == node->g_val)
	{
		node->h_val = node->parent->h_val;
	}
	else if (node->parent->h_val > 1)
	{
		node->h_val = node->parent->h_val - 1;		
	}
	else if (!node->cardinalConf.empty() || !node->rectCardinalConf.empty())
	{
		node->h_val = 1;
	}
	else
		node->h_val = 0;
	node->f_val = node->g_val + node->h_val;

	t1 = std::clock();
	findConflicts(*node);
	runtime_conflictdetection += std::clock() - t1;

	node->num_of_collisions = node->unknownConf.size() + node->cardinalConf.size() + node->semiConf.size() + node->nonConf.size() + node->rectCardinalConf.size();

	// update handles
	node->open_handle = open_list.push(node);
	HL_num_generated++;
	node->time_generated = HL_num_generated;
	if (node->f_val <= focal_list_threshold)
		node->focal_handle = focal_list.push(node);
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


void ICBSSearch::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		std::cout << "Agent " << i << " (" << paths_found_initially[i].size() - 1 << " -->" <<
			paths[i]->size() - 1 << "): ";
		for (int t = 0; t < paths[i]->size(); t++)
			std::cout << "(" << paths[i]->at(t).location / num_col << "," << paths[i]->at(t).location % num_col << ")->";
		std::cout << std::endl;
	}
}

void ICBSSearch::printConflicts(const ICBSNode &curr) const
{
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.rectCardinalConf.begin(); it != curr.rectCardinalConf.end(); ++it)
	{
		std::cout << "RectCardinal < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), " 
														<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
														<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.rectSemiConf.begin(); it != curr.rectSemiConf.end(); ++it)
	{
		std::cout << "RectSemi < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.rectNonConf.begin(); it != curr.rectNonConf.end(); ++it)
	{
		std::cout << "RectNon < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.cardinalConf.begin(); it != curr.cardinalConf.end(); ++it)
	{
		std::cout << "Cardinal < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.semiConf.begin(); it != curr.semiConf.end(); ++it)
	{
		std::cout << "Semi < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.nonConf.begin(); it != curr.nonConf.end(); ++it)
	{
		std::cout << "Non < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","//" (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
	for (list<std::shared_ptr<tuple<int, int, int, int, int>>>::const_iterator it = curr.unknownConf.begin(); it != curr.unknownConf.end(); ++it)
	{
		std::cout << "Unknown < " << get<0>(**it) << ","//" (+" << curr.paths[get<0>(**it)]->size() - dummy_start->paths[get<0>(**it)]->size() << "), "
			<< get<1>(**it) << ","// " (+" << curr.paths[get<1>(**it)]->size() - dummy_start->paths[get<1>(**it)]->size() << "), "
			<< get<2>(**it) << ", " << get<3>(**it) << ", " << get<4>(**it) << ">" << std::endl;
	}
}


void ICBSSearch::printConstraints(const ICBSNode* n) const
{
	const ICBSNode* curr = n;
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
// adding new nodes to FOCAL (those with min-f-val*f_weight between the old and new LB)
void ICBSSearch::updateFocalList(double old_lower_bound, double new_lower_bound, double f_weight) 
{
	for (ICBSNode* n : open_list) {
		if (n->f_val > old_lower_bound &&
			n->f_val <= new_lower_bound)
			n->focal_handle = focal_list.push(n);
	}
}

void ICBSSearch::updateReservationTable(bool* res_table, int exclude_agent, const ICBSNode &node) {
	for (int ag = 0; ag < num_of_agents; ag++) 
	{
		if (ag != exclude_agent && paths[ag] != NULL) 
		{
			for (size_t timestep = 0; timestep < node.makespan + 1; timestep++) 
			{
				int id;
				if (timestep >= paths[ag]->size())
					id = paths[ag]->at(paths[ag]->size() - 1).location;
				else// otherwise, return its location for that timestep
					id = paths[ag]->at(timestep).location;
				res_table[timestep * map_size + id] = true;
			}
		}
	}
}

// computes g_val based on current paths
inline int ICBSSearch::compute_g_val() {
	int retVal = 0;
	for (int i = 0; i < num_of_agents; i++)
		retVal += paths[i]->size() - 1;
	return retVal;
}

bool ICBSSearch::runICBSSearch() 
{
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
	while (!focal_list.empty() && !solution_found) 
	{
		// break after 5 min
		runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
		if (runtime > TIME_LIMIT || HL_num_expanded > 1000000)
		{  // timeout after 5 minutes
			cout << "TIMEOUT  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << endl;
			
			std::cout << "	Runtime Sumarry: lowlevel = " << runtime_lowlevel << " ; listoperation = " << runtime_listoperation << 
						" ; conflictdetection = " << runtime_conflictdetection << " ; computeh = " << runtime_computeh<<
						" ; updatepaths = " << runtime_updatepaths << " ; collectcons = " << runtime_updatecons << std::endl;

			double count = 0, value = 0, maxDepth = 0;
			while(!open_list.empty())
			{
				ICBSNode* curr = open_list.top();
				open_list.pop();
				if(curr->depth > maxDepth)
					maxDepth = curr->depth;
				if(curr->f_val > value + 0.001)
				{
					cout << "				#(f=" << value << ") = " << count << endl;
					count = 1;
					value = curr->f_val;
				}
				else
					count++;
			}
			std::cout << "Depth of last node: " << focal_list.top()->depth << " ; MaxDepth = " << maxDepth <<  std::endl;
			solution_found = false;
			break;
		}
		t1 = std::clock();
		ICBSNode* curr = focal_list.top();
		focal_list.pop();
		open_list.erase(curr->open_handle);
		runtime_listoperation += std::clock() - t1;
		// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
		t1 = std::clock();
		updatePaths(curr);
		runtime_updatepaths += std::clock() - t1;
#ifdef DEBUG
		//printPaths();
#endif
		if (cons_strategy == constraint_strategy::ICBS || cons_strategy == constraint_strategy::N_ICBS) // No heuristics
		{
			t1 = std::clock();
			curr->conflict = chooseConflict(*curr);
			runtime_conflictdetection += std::clock() - t1;
#ifdef DEBUG
			printConflicts(*curr);
#endif
		}
		else if(curr->conflict == NULL) //CBSH based, and h value has not been computed yet
		{
#ifdef ROOT
			classifyConflicts(*curr);
			int cardinal, semi, non;
			tie(cardinal, semi, non) = countRectangleConflicts(*curr);
			int num_of_conflitingPairs = 0;
			// Conflict graph
			vector<vector<bool>> CG(num_of_agents);
			int num_of_CGnodes = 0, num_of_CGedges = 0;
			for (int i = 0; i < num_of_agents; i++)
				CG[i].resize(num_of_agents, false);
			for (list<tuple<int, int, int, int, int>>::const_iterator it = curr->cardinalConf.begin(); it != curr->cardinalConf.end(); ++it)
			{
				if (!CG[get<0>(*it)][get<1>(*it)])
				{
					CG[get<0>(*it)][get<1>(*it)] = true;
					CG[get<1>(*it)][get<0>(*it)] = true;
					num_of_conflitingPairs++;
				}
			}
			for (list<tuple<int, int, int, int, int>>::const_iterator it = curr->semiConf.begin(); it != curr->semiConf.end(); ++it)
			{
				if (!CG[get<0>(*it)][get<1>(*it)])
				{
					CG[get<0>(*it)][get<1>(*it)] = true;
					CG[get<1>(*it)][get<0>(*it)] = true;
					num_of_conflitingPairs++;
				}
			}
			for (list<tuple<int, int, int, int, int>>::const_iterator it = curr->nonConf.begin(); it != curr->nonConf.end(); ++it)
			{
				if (!CG[get<0>(*it)][get<1>(*it)])
				{
					CG[get<0>(*it)][get<1>(*it)] = true;
					CG[get<1>(*it)][get<0>(*it)] = true;
					num_of_conflitingPairs++;
				}
			}
			std::cout << curr->cardinalConf.size() << "," << curr->semiConf.size() << "," 
				<< curr->nonConf.size() << "," << cardinal << "," << semi << "," << non << ","
				<< num_of_conflitingPairs <<  std::endl;
			node_stat.push_back(make_tuple(curr->cardinalConf.size(), curr->semiConf.size(), curr->nonConf.size(), 
				cardinal, semi, non, num_of_conflitingPairs));
			return false;
#endif
			
#ifdef DEBUG
			std::cout << std::endl << "****** Compute h for #" << curr->time_generated << " with f= " << curr->g_val <<
				"+" << curr->h_val << " (";
			for (int i = 0; i < num_of_agents; i++)
				std::cout << paths[i]->size() - 1 << ", ";
			std::cout << ") and #conflicts = " << curr->num_of_collisions << std::endl;
#endif
			t1 = std::clock();
			curr->conflict = classifyConflicts(*curr);
			runtime_conflictdetection += std::clock() - t1;
#ifdef DEBUG
			printConflicts(*curr);
#endif
			t1 = std::clock();
			curr->h_val = computeHeuristics(*curr);
			runtime_computeh += std::clock() - t1;
			curr->f_val = curr->g_val + curr->h_val;
//			if(curr->sum_min_f_vals > upper_bound - 0.001) // Pruning
//			{
//				curr->clear();
//				if (open_list.size() == 0) {
//					solution_found = false;
//					break;
//				}
//				t1 = std::clock();
//				ICBSNode* open_head = open_list.top();
//				if (open_head->sum_min_f_vals > min_sum_f_vals) {
//#ifdef DEBUG
//					cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
//#endif
//					min_sum_f_vals = open_head->sum_min_f_vals;
//					double new_focal_list_threshold = min_sum_f_vals * focal_w;
//					updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
//					focal_list_threshold = new_focal_list_threshold;
//#ifdef DEBUG
//					cout << focal_list.size() << endl;
//#endif
//				}
//#ifdef DEBUG
//				cout << " ; (after) " << focal_list_threshold << endl << endl;
//#endif
//				t2 = std::clock();
//				runtime_listoperation += t2 - t1;
//				continue;
//			}
//			else 
			if (curr->f_val > focal_list_threshold)
			{	
				t1 = std::clock();
				curr->open_handle = open_list.push(curr);
#ifdef DEBUG
				std::cout << "Reinsert the node with f =" << curr->g_val << "+" << curr->h_val << std::endl;
#endif
				ICBSNode* open_head = open_list.top();
				if (open_head->f_val > min_f_val) {
#ifdef DEBUG
					cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
#endif
					/*cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
					cout << " ; (after) " << focal_list_threshold << " ; #expanded=" << HL_num_expanded << " ; #generated=" <<  HL_num_generated << endl;*/
					min_f_val = open_head->f_val;
					double new_focal_list_threshold = min_f_val * focal_w;
					updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
					focal_list_threshold = new_focal_list_threshold;
#ifdef DEBUG
					cout << focal_list.size() << endl;
#endif
				}
#ifdef DEBUG
				cout << " ; (after) " << focal_list_threshold << endl << endl;
#endif
				runtime_listoperation += std::clock() - t1;
				continue;
			}
		}

		if (curr->conflict == NULL) //Fail to find a conflict => no conflicts
		{  // found a solution (and finish the while look)
			runtime = (std::clock() - start) + pre_runtime; // / (double) CLOCKS_PER_SEC;
			solution_found = true;
			solution_cost = curr->g_val;
			cout << solution_cost << " ; " << solution_cost - dummy_start->g_val << " ; " <<
				HL_num_expanded << " ; " << HL_num_generated << " ; " <<
				LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; ";
			cout << endl;
//#ifdef DEBUG
//			int conflictNum = computeNumOfCollidingAgents();
//			if(conflictNum > 0)
//				std::cout << "ERROR!" << std::endl;
//			std::cout << std::endl << "****** Solution: " << std::endl;
//			printPaths(*curr);
//#endif		
			break;
		}


		 //Expand the node
		HL_num_expanded++;
		curr->time_expanded = HL_num_expanded;
		
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

		ICBSNode* n1 = new ICBSNode();
		ICBSNode* n2 = new ICBSNode();
			

		if (get<2>(*curr->conflict) < 0) // Rectangle conflict
		{
			n1->agent_id = get<0>(*curr->conflict);
			n2->agent_id = get<1>(*curr->conflict);

			int s1_x = paths[get<0>(*curr->conflict)]->at(get<3>(*curr->conflict)).location / num_col;
			int s1_y = paths[get<0>(*curr->conflict)]->at(get<3>(*curr->conflict)).location % num_col;
			int s2_x = paths[get<1>(*curr->conflict)]->at(get<4>(*curr->conflict)).location / num_col;
			int s2_y = paths[get<1>(*curr->conflict)]->at(get<4>(*curr->conflict)).location % num_col;
			int con_x = (-1 - get<2>(*curr->conflict)) / num_col;
			int con_y = (-1 - get<2>(*curr->conflict)) % num_col;
			int con1_x, con1_y, con2_x, con2_y;
			if(s1_x == s2_x)
			{
				if ((s1_y - s2_y) * (s2_y - con_y) >= 0)
				{
					con1_x = s1_x;
					con2_x = con_x;
					con1_y = con_y;
					con2_y = s2_y;
				}
				else
				{
					con1_x = con_x;
					con2_x = s2_x;
					con1_y = s1_y;
					con2_y = con_y;
				}
			}
			else if ((s1_x - s2_x)*(s2_x - con_x ) >= 0)
			{
				con1_x = con_x;
				con2_x = s2_x;
				con1_y = s1_y;
				con2_y = con_y;
			}
			else
			{
				con1_x = s1_x;
				con2_x = con_x;
				con1_y = con_y;
				con2_y = s2_y;

			}
				int t = get<3>(*curr->conflict) + abs(con_x - s1_x) + abs(con_y - s1_y);
#ifdef DEBUG
			std::cout << "Agent " << n1->agent_id << ": (" << s1_x << "," << s1_y << ")" << std::endl;
			std::cout << "Agent " << n2->agent_id << ": (" << s2_x << "," << s2_y << ")" << std::endl;
			std::cout << "Con1<(" << con1_x << "," << con1_y << "), (" << con_x << "," << con_y << "), " << t << ">" << std::endl;
			std::cout << "Con2<(" << con2_x << "," << con2_y << "), (" << con_x << "," << con_y << "), " << t << ">" << std::endl;
#endif
			n1->constraint = make_tuple(-1 - con1_x * num_col - con1_y, -1 - get<2>(*curr->conflict), t, false);
			n2->constraint = make_tuple(-1 - con2_x * num_col - con2_y, -1 - get<2>(*curr->conflict), t, false);
		}
		else if (cons_strategy == constraint_strategy::N_ICBS || cons_strategy == constraint_strategy::N_CBSH) // New split method
		{
			n1->agent_id = get<0>(*curr->conflict);
			n2->agent_id = get<0>(*curr->conflict);
			n1->constraint = make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict), false);
			n2->constraint = make_tuple(get<2>(*curr->conflict), get<3>(*curr->conflict), get<4>(*curr->conflict), true);
			//n2->mdds.resize(num_of_agents, NULL);
		}
		else
		{
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
				
		}

		bool Sol1 = false, Sol2 = false;
		vector<vector<PathEntry>*> copy(paths);
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

		bool gen_n1 = true, gen_n2 = true;

		if (prior_branching) {
			if (curr->priorities[n1->agent_id][n2->agent_id]) { // a1->a2 do not generate n1
				gen_n1 = false;
			}
			if (curr->priorities[n2->agent_id][n1->agent_id]) { // a2->a1 do not generate n2
				gen_n2 = false;
			}

			if (gen_n1) {
				n1->priorities = vector<vector<bool>>(curr->priorities);
				n1->priorities[n2->agent_id][n1->agent_id] = true; // a2->a1
				if (trans_prior) {
					for (int i = 0; i < num_of_agents; i++) { // transitivity
						if (n1->priorities[i][n2->agent_id] && !n1->priorities[i][n1->agent_id]) {
							for (int j = 0; j < num_of_agents; j++) {
								if (n1->priorities[n1->agent_id][j]) {
									n1->priorities[i][j] = true;
								}
							}
						}
					}
				}
			}

			//updatePaths(curr);
			if (gen_n2) {
				n2->priorities = vector<vector<bool>>(curr->priorities);
				n2->priorities[n1->agent_id][n2->agent_id] = true; // a1->a2
				if (trans_prior) {
					for (int i = 0; i < num_of_agents; i++) { // transitivity
						if (n2->priorities[i][n1->agent_id] && !n2->priorities[i][n2->agent_id]) {
							for (int j = 0; j < num_of_agents; j++) {
								if (n2->priorities[n2->agent_id][j]) {
									n2->priorities[i][j] = true;
								}
							}
						}
					}
				}
			}
		}

		if (gen_n1) {
			Sol1 = generateChild(n1, curr);
		}
		paths = copy;
		if (gen_n2) {
			Sol2 = generateChild(n2, curr);
		}

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
		ICBSNode* open_head = open_list.top();
		if (open_head->f_val > min_f_val) {
			#ifdef DEBUG
			cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
			#endif
			min_f_val = open_head->f_val;
			double new_focal_list_threshold = min_f_val * focal_w;
			updateFocalList(focal_list_threshold, new_focal_list_threshold, focal_w);
			focal_list_threshold = new_focal_list_threshold;
			#ifdef DEBUG
			cout << focal_list.size() << endl;
			#endif
		}
		#ifdef DEBUG
        cout << " ; (after) " << focal_list_threshold << endl << endl;
		#endif
		runtime_listoperation += std::clock() - t1;

	}  // end of while loop


	//    printPaths();	
	if (focal_list.empty() && solution_cost < 0)
	{
		solution_cost = -2;
		cout << "No solutions  ; " << solution_cost << " ; " << min_f_val - dummy_start->g_val << " ; " <<
			HL_num_expanded << " ; " << HL_num_generated << " ; " <<
			LL_num_expanded << " ; " << LL_num_generated << " ; " << runtime << " ; " << 
			"|Open|=" << open_list.size() << endl;
		solution_found = false;
	}
	return solution_found;
}


ICBSSearch::ICBSSearch(const MapLoader& ml, const AgentsLoader& al, double f_w, const EgraphReader& egr, constraint_strategy c, bool prior_branching, bool trans_prior): focal_w(f_w), prior_branching(prior_branching), trans_prior(trans_prior)
{
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
		search_engines[i] = new SingleAgentICBS(init_loc, goal_loc, ml.get_map(), ml.rows*ml.cols,
			ml.moves_offset, ml.cols);
		ch.getHVals(search_engines[i]->my_heuristic);
	}

	// initialize allNodes_table (hash table)
	//empty_node = new ICBSNode();
	
	//empty_node->time_generated = -2; empty_node->agent_id = -2;
	//deleted_node = new ICBSNode();
	//deleted_node->time_generated = -3; deleted_node->agent_id = -3;
	//allNodes_table.set_empty_key(empty_node);
	//allNodes_table.set_deleted_key(deleted_node);


	dummy_start = new ICBSNode();
	dummy_start->agent_id = -1;
	
	if (prior_branching) {
		dummy_start->priorities = vector<vector<bool>>(num_of_agents, vector<bool>(num_of_agents, false));
	}
	
	// initialize paths_found_initially
	paths.resize(num_of_agents, NULL);
	paths_found_initially.resize(num_of_agents);

	clock_t start_t = std::clock();

	for (int i = 0; i < num_of_agents; i++) {
		//    cout << "Computing initial path for agent " << i << endl; fflush(stdout);
		bool* res_table = new bool[map_size * (dummy_start->makespan + 1)]();  // initialized to false
		updateReservationTable(res_table, i, *dummy_start);
		//cout << "*** CALCULATING INIT PATH FOR AGENT " << i << ". Reservation Table[MAP_SIZE x MAX_PLAN_LEN]: " << endl;
		//printResTable(res_table, max_plan_len);
		if (search_engines[i]->findPath(paths_found_initially[i], f_w, NULL, res_table, dummy_start->makespan + 1, 0) == false)
			cout << "NO SOLUTION EXISTS";
		//dummy_start->paths[i] = search_engines[i]->getPath();
		paths[i] = &paths_found_initially[i];
		dummy_start->makespan = max(dummy_start->makespan, paths_found_initially[i].size() - 1);
		//search_engines[i]->path.reset();
		//ll_min_f_vals_found_initially[i] = search_engines[i]->min_f_val;
		//paths_costs_found_initially[i] = search_engines[i]->path_cost;
		LL_num_expanded += search_engines[i]->num_expanded;
		LL_num_generated += search_engines[i]->num_generated;
		delete[] res_table;
		//    cout << endl;
	}

	
	//ll_min_f_vals = ll_min_f_vals_found_initially;
	//paths_costs = paths_costs_found_initially;

	// generate dummy start and update data structures
	
	dummy_start->g_val = 0;
	for (int i = 0; i < num_of_agents; i++)
		dummy_start->g_val += paths[i]->size() - 1;
	dummy_start->h_val = 0;
	dummy_start->f_val = dummy_start->g_val;
	//dummy_start->ll_min_f_val = 0;
	dummy_start->depth = 0;
	
	dummy_start->open_handle = open_list.push(dummy_start);
	dummy_start->focal_handle = focal_list.push(dummy_start);
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

	pre_runtime = std::clock() - start_t;

}

inline void ICBSSearch::releaseClosedListNodes() 
{
	for (list<ICBSNode*>::iterator it = allNodes_table.begin(); it != allNodes_table.end(); it++)
		delete *it;
}

inline void ICBSSearch::releaseOpenListNodes()
{
	while(!open_list.empty())
	{
		ICBSNode* curr = open_list.top();
		open_list.pop();
		delete curr;
	}
}

ICBSSearch::~ICBSSearch()
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
