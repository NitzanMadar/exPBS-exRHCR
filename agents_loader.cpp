//=======================================================================

#include "agents_loader.h"
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <algorithm>  // for remove_if
#include <ctime>
using namespace boost;
using namespace std;

int RANDOM_WALK_STEPS = 100000;

AgentsLoader::AgentsLoader(string fname, const MapLoader &ml, int agentsNum = 0){

  string line;

  ifstream myfile (fname.c_str());

  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    this->num_of_agents = atoi ( (*beg).c_str() );
    //    cout << "#AG=" << num_of_agents << endl;
    for (int i=0; i<num_of_agents; i++) {
      getline (myfile, line);
      tokenizer< char_separator<char> > col_tok(line, sep);
      tokenizer< char_separator<char> >::iterator c_beg=col_tok.begin();
      pair<int,int> curr_pair;
      // read start [row,col] for agent i
      curr_pair.first = atoi ( (*c_beg).c_str() );
    //  : add this->q_vector which is row stack [S_x1, S_y1, G_x1, G_y1, ..., S_xn, S_yn, G_xn, G_yn], same format as the database
      this->q_vector.push_back( atoi((*c_beg).c_str()) ); //  - push S_xi
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      this->q_vector.push_back( atoi((*c_beg).c_str()) ); //  - push S_yi
      //      cout << "AGENT" << i << ":   START[" << curr_pair.first << "," << curr_pair.second << "] ; ";
      this->initial_locations.push_back(curr_pair);

      // read goal [row,col] for agent i
      c_beg++;
      curr_pair.first = atoi ( (*c_beg).c_str() );
      this->q_vector.push_back( atoi((*c_beg).c_str()) ); //  - push G_xi
      c_beg++;
      curr_pair.second = atoi ( (*c_beg).c_str() );
      this->q_vector.push_back( atoi((*c_beg).c_str()) ); //  - push G_yi
      //      cout << "GOAL[" << curr_pair.first << "," << curr_pair.second << "]" << endl;
      this->goal_locations.push_back(curr_pair);
      // read max velocity and accelration for agent i
     /* c_beg++;
      this->max_v.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_w.push_back(atof((*c_beg).c_str()));
      c_beg++;
      this->max_a.push_back(atof((*c_beg).c_str()));*/
    }
    myfile.close();
  }
  else if(agentsNum > 0) // Generate agents randomly if file name is not exist
  {
      cout << "Given _.agents file not exist! Generate randomly instance and run..." << endl;
	  this->num_of_agents = agentsNum;
	  bool* starts = new bool[ml.rows * ml.cols];
	  for (int i = 0; i<ml.rows * ml.cols; i++)
		  starts[i] = false;
	  bool* goals = new bool[ml.rows * ml.cols];
	  for (int i = 0; i<ml.rows * ml.cols; i++)
		  goals[i] = false;
	  // Choose random start locations
	  for (int k = 0; k < agentsNum; k++)
	  {
		  int x = rand() % ml.rows, y = rand() % ml.cols;
		  int start = x * ml.cols +y;
		  if (!ml.my_map[start] && !starts[start])
		  {
				// update start
				this->initial_locations.push_back(make_pair(x,y));
				starts[start] = true;

				// random walk
				int loc = start;
				bool* temp_map = new bool[ml.rows * ml.cols];
				for (int walk = 0; walk < RANDOM_WALK_STEPS; walk++)
				{
					int directions[] = {0, 1, 2, 3, 4};
					random_shuffle(directions, directions + 5);
					int i = 0;
					for(; i< 5; i++)
					{
						int next_loc = loc + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols &&! ml.my_map[next_loc])
						{
							loc = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						system("pause");
					}
				}
				// find goal
				bool flag = false;
				int goal = loc;
				while (!flag)
				{
					int directions[] = { 0, 1, 2, 3, 4 };
					random_shuffle(directions, directions + 5);
					int i = 0;
					for (; i< 5; i++)
					{
						int next_loc = goal + ml.moves_offset[directions[i]];
						if (0 <= next_loc && next_loc < ml.rows * ml.cols && !ml.my_map[next_loc])
						{
							goal = next_loc;
							break;
						}
					}
					if (i == 5)
					{
						cout << "--------------ERROR!-----------------" << endl;
						system("pause");
					}
					flag = true;
					if (goals[goal])
						flag = false;
				}
				//update goal
				this->goal_locations.push_back(make_pair(goal / ml.cols, goal % ml.cols));
				goals[goal] = true;
				// update others
				/*this->max_v.push_back(1);
				this->max_w.push_back(1);
				this->max_a.push_back(1);*/
		  }
		  else
		  {
			  k--;
		  }
	  }
	  saveToFile(fname);
  }
}

void AgentsLoader::printAgentsInitGoal () {
  cout << "AGENTS:" << endl;;
  for (int i=0; i<num_of_agents; i++) {
    cout << "Agent" << i << " : I=(" << initial_locations[i].first << "," << initial_locations[i].second << ") ; G=(" <<
      goal_locations[i].first << "," << goal_locations[i].second << ")" << endl;
  }
  cout << endl;
}

AgentsLoader::~AgentsLoader() {
  // vectors are on stack, so they are freed automatically
}

// create an empty object
AgentsLoader::AgentsLoader() {
  num_of_agents = 0;
}

// returns the agents' ids if they occupy [row,col] (first for start, second for goal)
pair<int, int> AgentsLoader::agentStartOrGoalAt(int row, int col) {
  int f = -1;
  int s = -1;
  for (vector< pair<int, int> >::iterator it = initial_locations.begin(); it != initial_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      f = std::distance(initial_locations.begin(), it);
  for (vector< pair<int, int> >::iterator it = goal_locations.begin(); it != goal_locations.end(); ++it)
    if ( it->first == row && it->second == col )
      s = std::distance(goal_locations.begin(), it);
  return make_pair(f, s);
}


void AgentsLoader::clearLocationFromAgents(int row, int col) {
  pair<int, int> idxs = agentStartOrGoalAt(row, col);
  if ( idxs.first != -1 ) {  // remove the agent who's start is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.first );
    goal_locations.erase ( goal_locations.begin() + idxs.first );
    num_of_agents--;
  }
  idxs = agentStartOrGoalAt(row, col);
  if ( idxs.second != -1 ) {  // remove the agent who's goal is at [row,col]
    initial_locations.erase( initial_locations.begin() + idxs.second );
    goal_locations.erase( goal_locations.begin() + idxs.second );
    num_of_agents--;
  }
}


// add an agent
void AgentsLoader::addAgent(int start_row, int start_col, int goal_row, int goal_col) {
  this->initial_locations.push_back(make_pair(start_row, start_col));
  this->goal_locations.push_back(make_pair(goal_row, goal_col));
  num_of_agents++;
}

void AgentsLoader::saveToFile(std::string fname) {
  ofstream myfile;
  myfile.open(fname);
  myfile << num_of_agents << endl;
  for (int i = 0; i < num_of_agents; i++)
    myfile << initial_locations[i].first << "," << initial_locations[i].second << ","
           << goal_locations[i].first << "," << goal_locations[i].second << ","
		   /*<< max_v[i] << "," << max_a[i] << "," << max_w[i] << ","*/  << endl;
  myfile.close();
}
