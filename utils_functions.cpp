
#include <iostream>
#include <fstream>
#include <sstream>

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include <cstdlib>
#include <cmath>
#include <numeric>
#include <chrono>
#include <algorithm>
#include<boost/tokenizer.hpp>
#include <fstream>

#include "utils_functions.h"

using namespace std;

void read_priorities_file(const string fname, vector<vector<bool>> *initial_priorities, int *constraints_num){
    // cout << fname << endl;
    int counter=0; // count how many '1' in the priorities matrix

    ifstream in (fname);                                        // open the file
    string line;                                                // data from line, one by one
    vector<vector<bool>> output_matrix;
    while (getline(in, line)){                                  // loop reading the input line by line
        istringstream iss(line);                                //
        vector<bool> myvector;                                  // push the input line into vector
        char bit;                                               // single char in the line
        while (iss>>bit) {
            myvector.push_back(bit != '.');                     // push false if bit = '.' or true if '@'
            if (bit != '.'){
                counter++;
            }
        }
        output_matrix.push_back(myvector);                      // push vector line to the matrix
    }

    *initial_priorities = output_matrix;
    *constraints_num = counter;
}

// used for cleaning experience by average
void clean_outliers_priorities(vector<int> q_exp, vector<int> q_vector, const double dist_clean_threshold, vector<vector<bool>> &initial_priorities, int &ConstraintsMeasure){
    std::vector<int> dif (q_vector.size(),0);
    std::vector<int> dif_power (q_vector.size(),0);
    std::vector<int> dif_out (q_vector.size()/4,0);
    double curr_dist = 0.0;
    // minus vectors
    std::transform (q_vector.begin(), q_vector.end(), q_exp.begin(), dif.begin(), std::minus<int>());

    // my abs() function
    int (*myabs)(int) = &std::abs;

    // absolute dif vector
    std::transform (dif.begin(), dif.end(), dif.begin(), myabs);

    // (element)^2 into dif_power
    transform(dif.begin(), dif.end(), dif_power.begin(), [](int x){return x*x;});

    for (int i = 0; i < (int)q_vector.size()/4 ; ++i) { //element_size/4 because [... ,s_ix,s_iy,g_ix,g_iy, ...]
//        curr_dist = dif[4*i] + dif[4*i+1];                                // S_i Manhattan
//        curr_dist =  dif_power[4*i+2] + dif_power[4*i+3];                 // G_i Euclidean^2
//        curr_dist = dif_power[4*i] + dif_power[4*i+1];                    // S_i Euclidean^2
        curr_dist = sqrt(dif_power[4 * i] + dif_power[4 * i + 1]);       // S_i euclidean (not squared)
//        curr_dist = sqrt(dif_power[4 * i] + dif_power[4 * i + 1]) + sqrt(dif_power[4*i+2] + dif_power[4*i+3]); // S_i euclidean + G_i euclidean (not ^2)
           //
        if (dist_clean_threshold < 0) {
            cout << endl << "ERROR! use clean_outliers_priorities with threshold < 0 (should be filtered before call, at function NearestQuery)" << endl << endl;
        } // id dist_clean_threshold = -1 --> no priority matrix cleaning; else - clean if bigger than threshold
        if (curr_dist > dist_clean_threshold) {
            for (int j = 0; j < (int) initial_priorities.size(); ++j) {
                if (initial_priorities[i][j]) {          // if there is a True -> change to False
                    initial_priorities[i][j] = false;   // zero row i
                    ConstraintsMeasure--;
                }
                if (initial_priorities[j][i]) {          // if there is a True -> change to False
                    initial_priorities[j][i] = false;   // zero column i
                    ConstraintsMeasure--;
                }
            }
        }

    }
}

vector<vector<bool>> bool_matrices_a_and_not_b(const vector< vector<bool> > a, const vector< vector<bool> > b){
    // Input: boolean matrix "a" and boolean matrix "b", that have same dimensions.
    // Output: a and not be (a && !b). Used for fallback - find the current priority matrix minus the experience matrix

    // input sizes check:
    if (a.size() != b.size() or a[0].size() != b[0].size() ) {
        cout << endl << "ERROR!!! BAD INPUT TO FUNCTION:"<< endl <<
             "use function bool_matrix_subtract with different matrices size!!! the matrices should be with the same size!!!"
             << endl << endl;
    }

    int size_row = a.size();
    int size_col = a[0].size();
    vector<vector<bool>> output = vector<vector<bool>>(size_row, vector<bool>(size_col, false));

    for (int i=0; i <size_row; i++){
        for(int j=0; j<size_col; j++){
            if(a[i][j] and !b[i][j]){
                output[i][j] = true;
            }
        }
    }
    return output;
}

// todo - choose metric - this is relevant for offline experience database and not L-MAPF
double vectors_distances (const std::vector<int> &a, const std::vector<int> &b, vector<int> *dif_vec_k){
    // Input: vector a and vector b (one represent the data base query q_i and the other the asked new query tilde(q)
    // Output: the sum of vector absolute subtraction OR sum of vector subtraction squared


    double sum_of_elems = 0;
    std::vector<int> dif (a.size(),0);
    std::vector<int> dif_power (a.size(),0);
    std::vector<int> dif_out (a.size()/4,0);
//    dif.resize(a.size());
//    dif_power.resize(a.size());

    // minus vectors
    std::transform (a.begin(), a.end(), b.begin(), dif.begin(), std::minus<int>());

    // my abs() function
    int (*myabs)(int) = &std::abs;

    // absolute dif vector
    std::transform (dif.begin(), dif.end(), dif.begin(), myabs);

    // (element)^2 into dif_power
    transform(dif.begin(), dif.end(), dif_power.begin(), [](int x){return x*x;});


    //// *** elements sum TODO - {Choose criterion of difference (hide the other)} ***

    //   * option 1: Manhattan distances sum *
//    sum_of_elems = std::accumulate(dif.begin(), dif.end(), 0); //Manhattan distances sum (Si and Gi)

    //   * option 2: (euclidean)^2 distances sum *
//    sum_of_elems = std::accumulate(dif_power.begin(), dif_power.end(), 0); // (euclidean)^2 distances sum

    //   * seperated options: (euclidean)^2 distances sum *
    // next line sum only s_i
    for (int i = 0; i < (int)a.size()/4 ; ++i) { //element_size/4 because [... ,s_ix,s_iy,g_ix,g_iy, ...]
//        sum_of_elems = sum_of_elems + dif_power[4*i] + dif_power[4*i+1];    // S_i euclidean^2
        sum_of_elems = sum_of_elems + sqrt(dif_power[4 * i] + dif_power[4 * i + 1]);    // S_i euclidean (not ^2)
//        sum_of_elems = sum_of_elems + sqrt(dif_power[4 * i] + dif_power[4 * i + 1]) + sqrt(dif_power[4*i+2] + dif_power[4*i+3]); // S_i euclidean  + G_i euclidean (not ^2)
//        sum_of_elems = sum_of_elems + dif[4*i] + dif[4*i+1]; // S_i Manhattan
//        sum_of_elems = sum_of_elems + dif_power[4*i+2] + dif_power[4*i+3];  // G_i euclidean^2
//
        //todo zero g elements:
        dif_out[i] = dif_power[4*i] + dif_power[4*i+1];

    }
    //// *** end - elements sum *** ////

    // todo - output avg + dif_power
//    int avg_tmp;
//    avg_tmp = sum_of_elems / a.size() / 2;
//    *avg = avg_tmp;
    *dif_vec_k = dif_out;

    return sum_of_elems;
}

double vectors_distances_start_locations_Euclidean_squared (const std::vector<int> &a, const std::vector<int> &b)
{
    // Input: vector a and vector b (one represent the data base query q_i and the other the asked new query tilde(q)
    // Output: the sum of vectors squared euclidean distances
    // Example: a = {1,5,8,9} and b = {2,5,6,3} --> SUM{|a-b|} = |1-2| + |5-5| + |8-6| + |9-3| = |-1|+0+2+6 = 9
    //                                          OR --> (1-2)^2 + (5-5)^2 + (8-6)^2 + (9-3)^2 = 1+0+4+36 = 41

    double sum_of_elems = 0;
    std::vector<int> dif (a.size(),0);
    std::vector<int> dif_power (a.size(),0);


    // minus vectors
    std::transform (a.begin(), a.end(), b.begin(), dif.begin(), std::minus<int>());

    // my abs() function
    int (*myabs)(int) = &std::abs;

    // absolute dif vector
    std::transform (dif.begin(), dif.end(), dif.begin(), myabs);

    // (element)^2 into dif_power
    transform(dif.begin(), dif.end(), dif_power.begin(), [](int x){return x*x;});

    //// *** elements sum ***
    for (int i = 0; i < (int)a.size()/4 ; ++i) { //element_size/4 because [... ,s_ix,s_iy,g_ix,g_iy, ...]
        sum_of_elems = sum_of_elems + dif_power[4*i] + dif_power[4*i+1]; // S_i Euclidean^2
    }

    return sum_of_elems;
}

int worst_agent_distance(const std::vector<int> &a, const std::vector<int> &b){
    // Input: vector a and vector b (one represent the data base query q_i and the other the asked new query tilde(q))
    //        each vector is - [..., Sx_i, Sy_i, Gx_i, Gy_i, ...], size 4kx1.
    // Output: the worst agent distance according to some metric
    // Example: 2 agents, Si and Gi squared Euclidean distance, a = [(1,1,1,1),(2,2,2,2)], b = [(1,2,3,4), (2,1,2,1)]
    // --> worst_agent_dist = dist(a_2) = (1-1)^2+(1-2)^2+(1-3)^2+(1-4)^2 = 0+1+4+9 = 14

    int worst_agent_dist;
    int curr_agent_dist;

    std::vector<int> dif (a.size(),0);
    std::vector<int> dif_power (a.size(),0);

    // minus vectors
    std::transform (a.begin(), a.end(), b.begin(), dif.begin(), std::minus<int>());

    // my abs() function
    int (*myabs)(int) = &std::abs;

    // absolute dif vector
    std::transform (dif.begin(), dif.end(), dif.begin(), myabs);

    // (element)^2 into dif_power
    transform(dif.begin(), dif.end(), dif_power.begin(), [](int x){return x*x;});


    for (int i = 0; i < (int)a.size()/4 ; ++i) { //element_size/4 = k = number of agents, because vector are [... ,s_ix,s_iy,g_ix,g_iy, ...]
        // curr_agent_dist = dif[4*i] + dif[4*i+1];                                                     // S_i Manhattan
        // curr_agent_dist =  dif_power[4*i+2] + dif_power[4*i+3];                                      // G_i Euclidean^2
        // curr_agent_dist = dif_power[4*i] + dif_power[4*i+1];                                         // S_i Euclidean^2
        curr_agent_dist = sqrt(dif_power[4*i] + dif_power[4*i+1]);                                   // S_i Euclidean
        // curr_agent_dist = dif_power[4*i] + dif_power[4*i+1] + dif_power[4*i+2] + dif_power[4*i+3];   // S_i Euclidean^2 + G_i Euclidean^2

        if (i == 0){  // initialize
            worst_agent_dist = curr_agent_dist;
        }
        else {  // i > 0
            if (worst_agent_dist < curr_agent_dist){  //a_i is worse than worst_agent_dist between [0,1,...,i-1]
                worst_agent_dist = curr_agent_dist;
            }
        }
    }

    return worst_agent_dist;
}

double average_of_better_than_total_average(const std::vector<int> &a, const std::vector<int> &b){

    double total_avg;
    double curr_agent_dist;
    double sum_of_elems=0;
    int number_of_agents = (int)a.size()/4;  //element_size/4 = k = number of agents, because vector are [... ,s_ix,s_iy,g_ix,g_iy, ...]
    int smaller_than_avg = 0;
    double better_agents_sum_dist = 0;
    double average_of_bigger_than_total_average;

    std::vector<int> dif (a.size(),0);
    std::vector<int> dif_power (a.size(),0);

    // minus vectors
    std::transform (a.begin(), a.end(), b.begin(), dif.begin(), std::minus<int>());

    // my abs() function
    int (*myabs)(int) = &std::abs;

    // absolute dif vector
    std::transform (dif.begin(), dif.end(), dif.begin(), myabs);

    // (element)^2 into dif_power
    transform(dif.begin(), dif.end(), dif_power.begin(), [](int x){return x*x;});

    //   * option 1: Manhattan distances sum *
//    sum_of_elems = std::accumulate(dif.begin(), dif.end(), 0); //Manhattan distances sum (Si and Gi)

    //   * option 2: (euclidean)^2 distances sum *
//    sum_of_elems = std::accumulate(dif_power.begin(), dif_power.end(), 0); // (euclidean)^2 distances sum

    //   * option 3: (euclidean)^2 distances sum *
    // next line sum only s_i TODO - {choose S_i or g_i and dif or dif_power - ***first*** distance criterion}
    for (int i = 0; i < number_of_agents ; ++i) { //element_size/4 because [... ,s_ix,s_iy,g_ix,g_iy, ...]
//        sum_of_elems = sum_of_elems + dif_power[4 * i] + dif_power[4 * i + 1];    // S_i euclidean^2
        sum_of_elems = sum_of_elems + sqrt(dif_power[4 * i] + dif_power[4 * i + 1]); // S_i euclidean (not ^2)
//        sum_of_elems = sum_of_elems + sqrt(dif_power[4 * i] + dif_power[4 * i + 1])+ sqrt(dif_power[4*i+2] + dif_power[4*i+3]); // S_i euclidean + G_i euclidean (not ^2)
//        sum_of_elems = sum_of_elems + dif[4*i] + dif[4*i+1]; // S_i Manhattan
//        sum_of_elems = sum_of_elems + dif_power[4*i+2] + dif_power[4*i+3];  // G_i euclidean^2
    }

    total_avg = sum_of_elems/number_of_agents;

    for (int i = 0; i < number_of_agents ; ++i) {
        // curr_agent_dist = dif[4*i] + dif[4*i+1];                                                     // S_i Manhattan
        // curr_agent_dist =  dif_power[4*i+2] + dif_power[4*i+3];                                      // G_i Euclidean^2
        // curr_agent_dist = dif_power[4*i] + dif_power[4*i+1];                                         // S_i Euclidean^2
        curr_agent_dist = sqrt(dif_power[4*i] + dif_power[4*i+1]);                                   // S_i Euclidean (not squared...)
//        curr_agent_dist = sqrt(dif_power[4*i] + dif_power[4*i+1]) + sqrt(dif_power[4*2] + dif_power[4*i+3]);// S_i + G_i Euclidean (not squared...)
        // curr_agent_dist = dif_power[4*i] + dif_power[4*i+1] + dif_power[4*i+2] + dif_power[4*i+3];   // S_i Euclidean^2 + G_i Euclidean^2

        if (curr_agent_dist <= total_avg){  //a_i is worse than worst_agent_dist between [0,1,...,i-1]
            smaller_than_avg++;
            better_agents_sum_dist = better_agents_sum_dist + curr_agent_dist;
        }
    }

    average_of_bigger_than_total_average = better_agents_sum_dist/smaller_than_avg;

    return average_of_bigger_than_total_average;
}

// helper function to sort matrix (vector of vectors) by the second column
bool sortby2ndcol( const vector<int>& v1, const vector<int>& v2 ) {
    return v1[1] < v2[1]; //"1" sort by second column
}

bool sort_vector_by_column_n( const vector<int>& v1, const vector<int>& v2, int n ) {
    // note - vector first index is 0, so sort by column n means by index n-1
    return v1[n-1] < v2[n-1]; //"1" sort by n column
}

bool sort_by_second_column_double(const vector<double> &v1, const vector<double> &v2){
    // note - vector first index is 0, so index 1 is second column
    return v1[1] < v2[1]; //"1" sort by second column
}

void BuildDatabase (const string &AgentsFileName, const int AgentNumber, const int DB_size, vector<vector<int>> *DatabaseMat, vector<string> *DatabasePrioritiesFileName)
{
    /// The code use database, but if we use the exRHCR which based on experience from previous queries this is irrelevant and the database if of size one (we just reused the offline implementation)
    vector<vector<int>> Mat; // each vector is the agents start and goal location in row stack [S_x1,S_y1,G_x1,G_y1, ..., S_xn,S_yn,G_xn,G_yn]
    vector<string> names;
    string c;
//    string db_size = DB_size.as<string>();
    string data_fname;
    data_fname = AgentsFileName.substr(0, AgentsFileName.size()-7);     //remove ".agents"
    c = data_fname.back();
    while (isdigit(data_fname.back()) || c == "t" || c=="_" || c=="w") {
        data_fname = data_fname.substr(0, data_fname.size()-1);         // remove query number
        c = data_fname.back();
    }
    data_fname = data_fname.substr(0, data_fname.size()-1);             // remove "-"
    // data_fname.append("-");                                        // add suffix .database
    // data_fname.append(to_string(DB_size));                                    // add database size
    data_fname.append(".database");                                 // add suffix .database
    //// for zero-distance queries:
//    data_fname = "files/10obs-20x20map-100agents/10obs-20x20map-100agents_with_tests.database"; // similarity difference = 0, solve with priority from solution
    cout << "Database: " << data_fname << endl;
    ifstream in (data_fname.c_str());                                   // open the file

    //  - : added "if" statement to check if this _.database file exist
    if (!in.good()){ // if the database file is NOT exist ...
        cerr << "database not exist!" << endl;
    }

    else {  // if the database file is exist ...

        string line;
        vector<int> myvect;
        string filename;
        while (getline(in, line)) // run each line
        {

            stringstream linestream(line);
            string value;
            vector<int> myvect;

            while (getline(linestream, value, ',')) // run each value in line
            {
                string filename;
                if (value.length() < 5 &&
                    value != " ") {                      // the value length is < 5, most of the time <=2,
//                cout << stoi(value) << ",";                             // test values
                    myvect.push_back(stoi(value));                          // convert to int and push into vector
//                cout << myvect[-1] << ",";
                }
                if (value.length() > 5 && value != " ") {  //big string is the first element - the priorities file name
                    filename = value;
                    names.push_back(filename);
//                cout << filename << ", "; // test
                }

            }
//        cout << myvect[0] << ", ";
            Mat.push_back(myvect);
//        cout << Mat[0][0] << ", ";
//        cout << endl; //"Line Finished"
        }
        *DatabaseMat = Mat;
//    cout << myvect[0];
        *DatabasePrioritiesFileName = names;
//    cout << names[0]; // V
    }
}

void NearestQuery(const int given_priorities_fname, const vector<vector<int>> DatabaseMat, const int experience,const int cleaning_threshold, const vector<string> DatabasePrioritiesFileName, vector<int> q_vector,vector<vector<bool>> *initial_priorities,vector<vector<bool>> *fallback_priorities, int *NearestNeighborName, double *NearestNeighborDistance, int *ConstraintsMeasure)
{
    double min;
    double temp;
    int min_idx;
    int second_min_idx;
    double squared_euclidean_avg_dist;
    double avg_dist;
    string fname;
    string fallback_fname;
    vector<vector<bool>> output_matrix;
    vector <int> mins_idxs; //mins_idxs is a vector because of the case of more than 1 minimums - than take rand between them
    vector <int> dif_vec_k;
    vector<vector<double>> idx_dist_mat( DatabaseMat.size() , vector<double> (3, 0));  // [i] [distance (q, q_i)]

    if (DatabaseMat.size() == 1) {
        // dummy EDB for online experience gaining
        min_idx = 0;
        second_min_idx = 0;
        // *NearestNeighborName = 0;
        min =  vectors_distances(DatabaseMat[0], q_vector, &dif_vec_k);
        // *NearestNeighborDistance = vectors_distances(DatabaseMat[0], q_vector, &dif_vec_k);
        cout << "given dummy EDB of size 1 (Windowed-MAPF, no offline database)" << endl;

    }
    else {
        /////// take one distance as a criterion to detect similarity
        for (int i = 0; i < (int) DatabaseMat.size(); i++) {
            //choose metric w.r.t. cleaning method
            if (experience == 1) { // this mean no cleaning, use Start locations euclidean metric
                temp = vectors_distances(DatabaseMat[i], q_vector, &dif_vec_k); // current distance
                idx_dist_mat[i] = {(double) i,  // convert i to double to use it in 2XN double matrix
                                   vectors_distances(DatabaseMat[i], q_vector, &dif_vec_k)};
            } else if (experience == 2) { // this mean avg cleaning, use average_of_better_than_total_average metric
                //temp = average_of_better_than_total_average(DatabaseMat[i], q_vector);  // todo - temp change to int - fix it!
                idx_dist_mat[i] = {(double) i,  // convert i to double to use it in 2XN double matrix
                                   average_of_better_than_total_average(DatabaseMat[i], q_vector)};
            } else if (cleaning_threshold > 0) { // use specific cleaning threshold
                //temp = vectors_distances_start_locations_Euclidean_squared(DatabaseMat[i], q_vector); // current distance
                // temp = worst_agent_distance(DatabaseMat[i], q_vector);
                idx_dist_mat[i] = {(double) i,  // convert i to double to use it in 2XN double matrix
                                   vectors_distances_start_locations_Euclidean_squared(DatabaseMat[i], q_vector)};
            }


            /*
            if (i==0){ // initialize
                mins_idxs={i};
                min = temp;
                min_idx = 0;
            }
            else {  // i > 0
                if (min == temp) { // more than one minimum
                    mins_idxs.push_back(i);
                }
                if (min > temp) { // current distance is smaller
                    mins_idxs={i};
                    min = temp; // update min value
                }
            }
            */

        }

        // sort by second column (each row is [i, distance]):
        sort(idx_dist_mat.begin(), idx_dist_mat.end(), sort_by_second_column_double);




        /////// old - take one distance as a criterion to detect similarity

        /*
        //    //// two criteria (KNN w.r.t. some criterion and choose the best of them w.r.t. second criterion)
        //    // create Nx3 matrix [DB query index, first criterion distance, second criterion distance
        //    for ( int i = 0; i < (int)DatabaseMat.size() ; i++ ){
        //        idx_dist_mat[i] = {i,
        //                           vectors_distances(DatabaseMat[i], q_vector, &dif_vec_k),//, &avg_distance),
        //                           vectors_distances_second_criterion(DatabaseMat[i], q_vector) };
        //        // check the matrix:
        //        // cout << idx_dist_mat[i][0] << ", " << idx_dist_mat[i][1] << ", " << idx_dist_mat[i][2] << endl;
        //    }
        //    // sort by 2nd column (Euclidean Distance)
        //    sort(idx_dist_mat.begin(), idx_dist_mat.end(),sortby2ndcol);
        //
        //
        //    // between the top K (K = ?) nearest neighbor according to Euclidean distance, take the one with best Manhattan distance
        //    for ( int i = 0; i < 5 ; i++ ){ // now, K=5
        //        if (i==0){ //initialize
        //            mins_idxs={idx_dist_mat[i][0]}; //mins_idxs is a vector because of the case of more than 1 minimums - than take rand between them
        //            min = idx_dist_mat[i][2];
        //            temp = idx_dist_mat[i][2];
        //            min_idx = 0;
        //        }
        //        else{ // i != 0
        //            temp = idx_dist_mat[i][2];
        //            if (min == temp) { // more than one minimum
        //                mins_idxs.push_back(idx_dist_mat[i][0]);
        //            }
        //            if (min > temp) { // current distance is smaller
        //                mins_idxs={idx_dist_mat[i][0]}; // update minimum index
        //                min = temp; // update min value
        //            }
        //        }
        //    }
        //    //// END - two criteria (KNN w.r.t. some criterion and choose the best of them w.r.t. second criterion)
        */


        //take the minimal value / random from minimal mins_idxs
        /*
    //    min_idx=*min_element(distances.begin(), distances.end());
        if (mins_idxs.size()>1) {
            int my_rand = rand() % mins_idxs.size();
    //        cout << mins_idxs[0] <<", " << mins_idxs[1];
            min_idx = mins_idxs[my_rand]; //choose random from all the candidates
            cout << endl << "more than one nearest neighbor, we have " << mins_idxs.size() << " and choose- #" << my_rand << " which is " << min_idx << endl;
        }
        else {
    //        cout << "one nearest neighbor....";
            min_idx=mins_idxs[0];
        }
        */


        min_idx = idx_dist_mat[0][0];  // minimal index in the distance sorted matrix
        second_min_idx = idx_dist_mat[1][0];  // minimal index in the distance sorted matrix
        min = idx_dist_mat[0][1];
        //  - use the min_idx to to take the _.priorities filename from DatabasePrioritiesFileName and read it into initial_priorities...............
        if (given_priorities_fname > -1) {
            min_idx = given_priorities_fname; //  enforce the priority to be as the input
            min = vectors_distances(DatabaseMat[given_priorities_fname], q_vector,
                                    &dif_vec_k);// doesn't matter which metric, min_idx is given
        }

    }

    // cout << "min query index = " << min_idx << "; min distance = "<< min << endl; // V
    // --- READ THE NEAREST QUERY EXPERIENCE MATRIX --- //
    fname = DatabasePrioritiesFileName[min_idx];

    fallback_fname = DatabasePrioritiesFileName[second_min_idx];

    // bully-nerds cleaning // clean dominating agent - old, irrelevant
    if (given_priorities_fname == -2){
        cout << "use 'bully-nerd' priorities cleaning" << endl;
        fname = fname.substr(0, fname.size()-11); // remove ".priorities" suffix
        fname.append("-cleaned.priorities");  // in this case we'll used the cleaned priorities file
    }

    // find avg for cleaning -
    avg_dist = vectors_distances(DatabaseMat[min_idx], q_vector, &dif_vec_k) / (q_vector.size()/4);  // q_vector.size()/4 = k = number of agents, so avg is total distance / number of agents
    squared_euclidean_avg_dist = vectors_distances_start_locations_Euclidean_squared(DatabaseMat[min_idx], q_vector) / (q_vector.size()/4);

    read_priorities_file(fname, initial_priorities, ConstraintsMeasure);
    // cout << "priority fname: " << fname;
    int ConstraintsMeasure2 = 0;
    read_priorities_file(fallback_fname, fallback_priorities, &ConstraintsMeasure2);


    //// ======= priority matrix cleaning: =======
    if (experience == 2){  // clean by average
//        clean_outliers_priorities(DatabaseMat[min_idx], q_vector, 1*squared_euclidean_avg_dist, *initial_priorities, *ConstraintsMeasure);  // use average distance as cleaning threshold
        clean_outliers_priorities(DatabaseMat[min_idx], q_vector, 1.0*avg_dist, *initial_priorities, *ConstraintsMeasure);  // use average distance as cleaning threshold
        cout << "cleaning by average distance " << avg_dist << " or " << squared_euclidean_avg_dist << endl;
    }
    else if (cleaning_threshold > 0) {  // clean by given threshold
        clean_outliers_priorities(DatabaseMat[min_idx], q_vector, cleaning_threshold, *initial_priorities, *ConstraintsMeasure);
        cout << "cleaning by given threshold: " << cleaning_threshold << endl;
    }
    // else (cleaning_threshold==-1) - do not clean..
    //// ======= END - priority matrix cleaning =======

    /*
    int counter=0; // count how many '1' in the priorities matrix
    ifstream in (fname);                                        // open the file
    string line;                                                // data from line, one by one
    while (getline(in, line)){                                  // loop reading the input line by line
        istringstream iss(line);                                //
        vector<bool> myvector;                                  // push the input line into vector
        char bit;                                               // single char in the line
        while (iss>>bit) {
//            myvector.push_back(bit == '@' ? true : false);

            myvector.push_back(bit != '.');                     // push false if bit = '.' or true if '@'
            if (bit != '.'){
                counter++;

            }
        }
        output_matrix.push_back(myvector);                      // push vector line to the matrix
    }



    *initial_priorities = output_matrix;
        // counter/(0.5*K*(K-1)) is a measure for how contrained is the matrix, (0.5*K*(K-1)) is the maximum constraints available
    // [where output_matrix.size() = K = AgentNumber]
//    *ConstraintsMeasure = counter / (0.5 * output_matrix.size() * (output_matrix.size()-1));
    *ConstraintsMeasure = counter;
    */



//    *NearestNeighborName = fname.substr(fname.size()+20, fname.size()-7);
    *NearestNeighborName = min_idx;
    *NearestNeighborDistance = min;

}

void print_bool_matrix(const vector< vector<bool> > a){
    for (int i=0; i <a.size(); i++){
        for (int j=0; j <a[0].size(); j++) {
            cout << a[i][j] << ", ";

        }
        cout << endl;
    }
}