

#ifndef PBS_EXPERIENCE_UTILS_FUNCTIONS_H
#define PBS_EXPERIENCE_UTILS_FUNCTIONS_H
//#include <iostream>


#include <string>
#include <vector>
#include <utility>
#include "map_loader.h"
using namespace std;


void read_priorities_file(const string fname, vector<vector<bool>> *initial_priorities, int *constraints_num);

void clean_outliers_priorities(vector<int> q_exp, vector<int> q_vector, const double dist_clean_threshold, vector<vector<bool>> &initial_priorities, int &ConstraintsMeasure);

vector<vector<bool>> bool_matrices_a_and_not_b(const vector< vector<bool> > a, const vector< vector<bool> > b);

double vectors_distances (const std::vector<int> &a, const std::vector<int> &b, vector<int> *dif_vec_k);//, int *avg);

double vectors_distances_start_locations_Euclidean_squared (const std::vector<int> &a, const std::vector<int> &b);

int worst_agent_distance(const std::vector<int> &a, const std::vector<int> &b);

double average_of_better_than_total_average(const std::vector<int> &a, const std::vector<int> &b);

int how_many_bigger_than_avg(const std::vector<int> &a, const std::vector<int> &b);

bool sortby2ndcol( const vector<int>& v1, const vector<int>& v2 );

void BuildDatabase (const string &AgentsFileName, const int AgentNumber, const int DB_size, vector<vector<int>> *DatabaseMat, vector<string> *DatabasePrioritiesFileName);

void NearestQuery(const int given_priorities_fname, const vector<vector<int>> DatabaseMat, const int experience,const int cleaning_threshold, const vector<string> DatabasePrioritiesFileName, vector<int> q_vector,vector<vector<bool>> *initial_priorities, vector<vector<bool>> *fallback_priorities, int *NearestNeighborName, double *NearestNeighborDistance, int *ConstraintsMeasure);

#endif //PBS_EXPERIENCE_UTILS_FUNCTIONS_H

void print_bool_matrix(const vector< vector<bool> > a);