#include "epea_search.h"

/// <summary>
/// Expands a single agent in the nodes.
/// This includes:
/// - Generating the children
/// - Inserting them into OPEN
/// - Insert node into CLOSED
/// Returns the child nodes
/// </summary>
list<EPEANode*> EPEASearch::ExpandOneAgent(list<EPEANode*>& intermediateNodes, int agent_id)
{
	list<EPEANode*> GeneratedNodes;
	for (list< EPEANode* >::iterator it = intermediateNodes.begin(); it != intermediateNodes.end(); ++it) 
	{
		/*if ((std::clock() - this->start_time) > TIME_LIMIT)
			break;*/
		EPEANode* node = *it;
		// Try all legal moves of the agents
		for (list< pair<int16_t, int16_t> >::iterator it = node->singleAgentDeltaFs->at(agent_id).begin(); it != node->singleAgentDeltaFs->at(agent_id).end(); ++it)
		{
			int next_loc = node->locs[agent_id] + moves_offset[it->first];
			if(!node->IsValid(agent_id, node->locs[agent_id], next_loc/*, vertex_cons, edge_cons*/))
				continue;
			// Using the data that describes its delta F potential before the move.
			if (it->second <= node->remainingDeltaF  // last move was good
				&&  node->existsChildForF(agent_id + 1, node->remainingDeltaF))
			{
				EPEANode *childNode = new EPEANode();
				childNode->deep_copy(*node); // Copy all except lookUp table
				//if (agent_id == 0)
				//	childNode->parent = node;
				//else
				//	childNode->parent = node->parent; // Skip temporary node objects used during expansion process.
				childNode->MoveTo(agent_id, next_loc);
				childNode->remainingDeltaF -= it->second; // Update target F
				GeneratedNodes.push_back(childNode);
			}
			else
				break;
		}
		if(agent_id > 0)
			delete node;
	}
	intermediateNodes.clear();
	return GeneratedNodes;
}

void EPEASearch::Clear()
{
	for (int i = 0; i < my_heuristics.size(); i++)
		delete[] my_heuristics[i];
	// releaseClosedListNodes
	hashtable_t::iterator it;
	for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) {
		delete ((*it).second);  // should it be .second?
	}
	allNodes_table.clear();
	delete (empty_node);
	delete (deleted_node);
}

void EPEASearch::Expand(EPEANode& node)
{
	/*if (node.index % 1000 == 0)
	{
		cout << "Expand node [" << node.index << "] {";
		int i = 0;
		for (; i < num_of_agents - 1; i++)
			cout << "(" << node.locs[i] / num_col << "," << node.locs[i] % num_col << "),";
		cout << "(" << node.locs[i] / num_col << "," << node.locs[i] % num_col << ")} with g = " << node.g << ", h = " << node.h <<
			" and target delta f = " << node.targetDeltaF << endl;
	}*/
	//cout << "Expand node [" << node.index << "] {";
	//int i = 0;
	//for (; i < num_of_agents - 1; i++)
	//	cout << "(" << node.locs[i] / num_col << "," << node.locs[i] % num_col << "),";
	//cout << "(" << node.locs[i] / num_col << "," << node.locs[i] % num_col << ")} with g = " << node.g << ", h = " << node.h <<
	//	" and target delta f = " << node.targetDeltaF << endl;
	
	//bool wasAlreadyExpanded = true;

	if (!node.alreadyExpanded)
	{
		node.calcSingleAgentDeltaFs();
		//num_expanded++;
		node.alreadyExpanded = true;
		//wasAlreadyExpanded = false;
		//node.hBonus = 0; // Locking any hbonus that doesn't come from partial expansion
		node.targetDeltaF = 0; // Assuming a consistent heuristic (as done in the paper), the min delta F is zero.
		node.remainingDeltaF = 0; // Just for the following hasChildrenForCurrentDeltaF call.
		while (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF) == false) // DeltaF=0 may not be possible if all agents have obstacles between their location and the goal
		{
			node.targetDeltaF++;
			node.remainingDeltaF = node.targetDeltaF;  // Just for the following hasChildrenForCurrentDeltaF call.
		}
		if (node.targetDeltaF > node.maxDeltaF) // Node has no possible children at all
		{
			node.Clear();
			return;
		}
	}

	// If this node was already expanded, notice its h was updated, so the deltaF refers to its original H
	//if (node.index == 176)
	//	cout << "*" << endl;
	list<EPEANode*> intermediateNodes;
	intermediateNodes.push_back(&node);

	for (int agentIndex = 0; agentIndex < num_of_agents; agentIndex++)
	{
		/*if ((std::clock() - this->start_time) > TIME_LIMIT)
			return;*/

		//cout << "Moving agent " << agentIndex << endl;

		intermediateNodes = ExpandOneAgent(intermediateNodes, agentIndex);
	}
	list<EPEANode*> finalGeneratedNodes = intermediateNodes;
	for (list< EPEANode* >::iterator it = finalGeneratedNodes.begin(); it != finalGeneratedNodes.end(); ++it)
	{
		/*if ((std::clock() - this->start_time) > TIME_LIMIT)
			return;*/
		(*it)->makespan++;
		(*it)->ClearConstraintTable();
		(*it)->targetDeltaF = 0;
		(*it)->parent = &node;
		//if (currentNode.g < currentNode.minCost)
		//{
		//	if (currentNode.h == 0)
		//		currentNode.h = 2; // Otherwise waiting at goal would expand to waiting at the goal for the same too low cost,
		//						   // which would expand to waiting at the goal, etc.
		//						   // +2 because you need a step out of the goal and another step into it.
		//	currentNode.h = Math.Max(currentNode.h, currentNode.minCost - currentNode.g);
		//}
	}

	// Enter the generated nodes into the open list
	for (list< EPEANode* >::iterator child = finalGeneratedNodes.begin(); child != finalGeneratedNodes.end(); ++child)
	{
		if ((*child)->h + (*child)->g <= this->maxCost)
			// Assuming h is an admissable heuristic, no need to generate nodes that won't get us to the goal
			// within the budget
		{
			// try to retrieve it from the hash table
			hashtable_t::iterator it = allNodes_table.find(*child);
			// If in closed list - only reopen if F is lower or node is otherwise preferred
			if (it != allNodes_table.end()) // Notice the agents may have gotten to their location from a different direction in this node.
			{
				//++this->closedListHits;
				EPEANode* existing = (*it).second;

				bool compare;  
				if((*child)->g + (*child)->h == existing->g + existing->h/* + existing->targetDeltaF*/)
					compare = (*child)->h < existing->h/* + existing->targetDeltaF*/;
				else
					compare = (*child)->g + (*child)->h < existing->g + existing->h/* + existing->targetDeltaF*/;


				if (compare)// This node has smaller f, or preferred due to other consideration.
				{
					existing->Update(**child);
					/*cout << "Update node [" << existing->index << "] into {";
					int i = 0;
					for (; i < num_of_agents - 1; i++)
						cout << "(" << existing->locs[i] / num_col << "," << existing->locs[i] % num_col << "),";
					cout << "(" << existing->locs[i] / num_col << "," << existing->locs[i] % num_col << ")} with g = " << existing->g << " and h = " << existing->h << endl;*/
					if (!existing->in_openlist) // reopen 
					{
						existing->open_handle = this->open_list.push(existing);
						existing->in_openlist = true;
					}
				}
				delete (*child);
			}
			else  // add the newly generated node to open_list and hash table
			{
				allNodes_table[(*child)] = *child;
				this->num_generated++;
				(*child)->index = num_generated;
				(*child)->open_handle = open_list.push(*child);
				(*child)->in_openlist = true;

				//if((*child)->index % 10000 == 0)
				{
					/*cout << "Generate new node [" << (*child)->index  << "] {";
					int i = 0;
					for (; i < num_of_agents - 1; i++)
						cout << "(" << (*child)->locs[i] / num_col << "," << (*child)->locs[i] % num_col << "),";
					cout << "(" << (*child)->locs[i] / num_col << "," << (*child)->locs[i] % num_col << ")} with g = " << (*child)->g << " and h = " << (*child)->h << endl;*/
				}

				//currentNode.expandedCountWhenGenerated = this.expanded;
			}
		}
	}

	if (node.alreadyExpanded == false)
	{
		// Node was cleared during expansion.
		// It's unnecessary and unsafe to continue to prepare it for the next partial expansion.
		return;
		// TODO: Is there a prettier way to do this?
	}




	node.targetDeltaF++; // This delta F was exhausted
	node.remainingDeltaF = node.targetDeltaF;
	
	while (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF) == false)
	{
		node.targetDeltaF++;
		node.remainingDeltaF = node.targetDeltaF; // Just for the following hasChildrenForCurrentDeltaF call.
	}

	if (node.targetDeltaF <= node.maxDeltaF && node.existsChildForF(0, node.remainingDeltaF)
			&& node.h + node.g + node.targetDeltaF <= this->maxCost)
	{
		// Re-insert node into open list
		//cout << "Re-insert node into open list with target delta f = " << node.targetDeltaF << endl;
		node.open_handle = open_list.push(&node);
		node.in_openlist = true;
	}
	else
	{
		node.Clear();
	}
}

/// <summary>
/// Runs the algorithm until the problem is solved or memory/time is exhausted
/// </summary>
/// <returns>True if solved</returns>
bool EPEASearch::runEPEASearch()
{
	cout << "      EPEA: ";
	// set timer
	std::clock_t start;
	start = std::clock();

	int initialEstimate = open_list.top()->h; // g = targetDeltaF = 0 initially

	int lastF = -1;

	while (!open_list.empty())
	{
		// Check if max time has been exceeded
		runtime = std::clock() - start;
		if (runtime > TIME_LIMIT)
		{
			solution_cost = -1;
			solution_depth = open_list.top()->g + open_list.top()->h + open_list.top()->targetDeltaF  - initialEstimate; // A minimum estimate
			cout << "TIMEOUT  ; " << solution_cost << " ; " << solution_depth << " ; " <<
				num_expanded << " ; " << num_generated << " ; " <<
				num_expanded << " ; " << num_generated << " ; " << runtime << endl;
			this->Clear();
			return false;
		}

		EPEANode* curr = open_list.top();
		open_list.pop();
		
		lastF = curr->g + curr->h + curr->targetDeltaF;

		// Check if node is the goal
		if ((int)round(curr->h) == 0)
		{
			runtime = std::clock() - start;
			this->solution_cost = curr->g;
			this->paths = curr->GetPlan();
			this->solution_depth = curr->g - initialEstimate;
			cout << solution_cost << " ; " << solution_depth << " ; " <<
				num_expanded << " ; " << num_generated << " ; " <<
				num_expanded << " ; " << num_generated << " ; " << runtime << endl;
			this->Clear();
			return true;
		}

		// Expand
		Expand(*curr);
		num_expanded++;
	}
	solution_cost = -2;
	this->solution_depth = lastF - initialEstimate;
	runtime = std::clock() - start;
	cout << "NO SOLUTION  ; " << solution_cost << " ; " << solution_depth << " ; " <<
		num_expanded << " ; " << num_generated << " ; " << runtime << endl;
	this->Clear();
	return false;
}

EPEASearch::EPEASearch(const MapLoader& ml, const AgentsLoader& al, const EgraphReader &egr):
	al(al)
{
	this->num_col = ml.cols;
	this->num_row = ml.rows;
	this->moves_offset = ml.moves_offset;
	this->num_of_agents = al.num_of_agents;

	// initialize allNodes_table (hash table)
	empty_node = new EPEANode();
	empty_node->locs[0] = -2;
	deleted_node = new EPEANode();
	deleted_node->locs[0] = -3;
	allNodes_table.set_empty_key(empty_node);
	allNodes_table.set_deleted_key(deleted_node);
	
	// compute heuristic lookup table
	my_heuristics.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++) 
	{
		int init_loc = ml.linearize_coordinate((al.initial_locations[i]).first, (al.initial_locations[i]).second);
		int goal_loc = ml.linearize_coordinate((al.goal_locations[i]).first, (al.goal_locations[i]).second);
		ComputeHeuristic ch(init_loc, goal_loc, ml.get_map(), ml.rows, ml.cols, ml.moves_offset, ml.actions_offset, 1.0, &egr);
		my_heuristics[i] = ch.getHVals();
	}

	// generate root node
	EPEANode* root_node = new EPEANode(ml, al, my_heuristics);
	root_node->open_handle = this->open_list.push(root_node);
	root_node->in_openlist = true;
	allNodes_table[root_node] = root_node;
	num_generated++;
	root_node->index = num_generated;

}

EPEASearch::~EPEASearch()
{
}
