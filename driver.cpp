#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
//#include "ecbs_search.h"
#include "epea_search.h"
#include "ICBSSearch.h"
#include "GICBSSearch.h"

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include "Timer.hpp"

// new includes: //
#include <numeric>
#include <chrono>
#include <algorithm>
#include<boost/tokenizer.hpp>
#include <fstream>
#include "utils_functions.h"
// **************** //

#include "boost/program_options.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

namespace pt = boost::property_tree;
using namespace std;

int main(int argc, char** argv) {

	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents (override by next query if W-MAPF used, a solution founded and agent_fname[-8)='w'")
		("output,o", po::value<std::string>()->required(), "output file name (will be at same directory as driver)")
		("solver,s", po::value<std::string>()->default_value("CBSH"), "Use Default! solvers (EPEA, ECBS, ICBS, CBSH, N-ECBS, N-ICBS, N-CBSH, CBSH-CR, CBSH-R, CBSH-RM. This is from original PBS paper, use CBSH")  // use default CBSH..
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("priority,p", po::value<int>()->default_value(2), "priority branching (0 = lazy generating, 1 = aggressive generating, 2 = aggressive greedy, 3 = fixed priority). This is from original PBS, use 2")
		("priorityMatrix,q", po::value<int>()->default_value(-1), "choose priority matrix, -1 means no specific choose, -2 means use bully-nerd cleaning, -3 artificial experience, >0 is choose specific P matrix") // : added priorities matrix as input (Q_ij means i<j, i has higher priority than j)
		("experience,e", po::value<int>()->default_value(0), "using experience or not: 0 - no experience, 1 - use experience, 2 - use experience and also clean the priority matrix used")
		("cleaning_threshold,c", po::value<double>()->default_value(-1), "cleaning the priority matrix that distant > cleaning_threshold. -1 means do not clean, full experience matrix (equivalent to infinity), -2 clean larger than average distance, 0 clean all, that is original PBS")
		("fallback,f", po::value<double>()->default_value(0), "which fallback to use: 0 - no fallback, (0,1] - upward fallback, the number detect % of tail to jump up, 2 - original PBS fallback (P_exp -> no exp), 3 - other experience and original PBS fallback (P_exp1 -> P_exp2 -> no experience). Use the same input with minus to use P_d-P_exp when fallback is used")
		("width_limit_hl,b", po::value<int>()->default_value(-1), "high-level WL-DFS width limit (default -1, means no limit)")
		("to_save_P_matrix,x", po::value<int>()->default_value(0), "0 or 1, default 0, set 1 to create _.priorities file (save priorities matrix as txt file, if solution exist)")
		("windowed_mapf,w", po::value<int>()->default_value(-1), "solve regular MAPF if w=-1 (default) or windowed-MAPF with window size w as given")
		("frequency_mapd,h", po::value<int>()->default_value(-1), "replaning frequency for MAPD (-1 do nothing)")

	;

    po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}

	po::notify(vm);
	srand((int)time(0));

	// read the map file and construct its two-dim array
	MapLoader ml(vm["map"].as<string>());

	// read agents' start and goal locations
	AgentsLoader al(vm["agents"].as<string>(), ml, vm["agentNum"].as<int>());

	// read the egraph --- we don't use highway here //  - try PBS with highways
	EgraphReader egr;


///////////////////////////////////////// experience database //////////////////////////////////////////////////////////
//// Previously, we used an offline experience database. For lifelong MAPF with exRHCR, the experience is given from
/// previous W-MAPF sub-query. Thus we create a "dummy" database, of size one, which hold the experience name only.
// Read _.database file into matrix of start goal locations and vector of priority ordering corresponding names
    vector<vector<bool>> initial_priorities;
    vector<vector<bool>> fallback_priorities;  // when bad HL tree with initial_priorities is detected
    vector<vector<int>> DatabaseMat;        // each vector is the agents start and goal location in row stack [S_x1, S_y1, G_x1, G_y1, ..., S_xn, S_yn, G_xn, G_yn]
    double NearestNeighborDistance = -1; // Nearest Neighbor query distance
    std::clock_t time_stamp; // set timer
    double load_database_runtime = -1; // set timer
    double find_nearest_query_runtime = -1; // set timer
    int NearestNeighborName = -1;
    int ConstraintsMeasure = -1;
    bool use_experience = false;
    bool use_clean = false;
    if (vm["experience"].as<int>() == 0 and (vm["fallback"].as<double>() > 1 or vm["fallback"].as<double>() < 0) ) {
        throw invalid_argument("\n\n***ERROR! INVALID INPUT: cannot use experience = 0 (original PBS) with fallback that not in [0,1] (no fallback or upward fallback with Pd)***\n\n");
        cerr << "ERROR! cannot use fallback > 1 (other experience / original PBS fall back) with experience = 0 (original PBS)" ;
    }

    if (vm["fallback"].as<double>() != 0 and vm["width_limit_hl"].as<int>()  <= 1 ) {
        throw invalid_argument("\n\n*** ERROR! INVALID INPUT: got fallback input (!=0) without HL width limit (<=1) ***\n\n" );
    }

    if (vm["fallback"].as<double>() == 0 and vm["width_limit_hl"].as<int>()  != -1 ) {
        throw invalid_argument("\n\n*** ERROR! INVALID INPUT: got HL width limit (!=-1) but no fallback (==0) ***\n\n" );
    }

    if (vm["fallback"].as<double>() < -1) {
        throw invalid_argument("\n\n*** ERROR! INVALID INPUT: fallback option should be greater than -1 ***\n\n" );
    }

    if (vm["cleaning_threshold"].as<double>()  != -1) {
        use_clean = true;
    }

    if (vm["priorityMatrix"].as<int>() == -3){
        string art_fname;
        art_fname = vm["agents"].as<string>().substr(0, vm["agents"].as<string>().size()-7); // remove ".agents" suffix
        art_fname.append("-artificial.priorities");  // in this case we'll used the cleaned priorities file
        // read priority file
        read_priorities_file(art_fname, &initial_priorities, &ConstraintsMeasure);
        fallback_priorities = initial_priorities;
        NearestNeighborName = -3;
    }
    else if (vm["experience"].as<int>() > 0) {      // if experience>0 - read the database and find the nearest neighbor from database and insert into &initial_priorities
        use_experience = true;
        vector<string> DatabaseAgentsFileName;
        time_stamp = std::clock();
        BuildDatabase(vm["agents"].as<string>().c_str(), vm["agentNum"].as<int>(), vm["experience"].as<int>(), &DatabaseMat, &DatabaseAgentsFileName);
        load_database_runtime = std::clock() - time_stamp;

        // database build runtime relevant for offline experience only!
        // cout << "Database building runtime = " << load_database_runtime;

        time_stamp = std::clock();

        double cleaning_threshold = vm["cleaning_threshold"].as<double>();

        NearestQuery(vm["priorityMatrix"].as<int>(), DatabaseMat, vm["experience"].as<int>(), cleaning_threshold, DatabaseAgentsFileName, al.q_vector, &initial_priorities, &fallback_priorities, &NearestNeighborName, &NearestNeighborDistance, &ConstraintsMeasure); //using the database to find the nearest query and insert the initial priority to &initial_priority

        find_nearest_query_runtime = std::clock() - time_stamp;
        // cout << "Nearest query runtime = " << find_nearest_query_runtime << endl;
        if (vm["windowed_mapf"].as<int>() > 0){
            fallback_priorities = vector<vector<bool>>(al.num_of_agents, vector<bool>(al.num_of_agents, false)) ;
        }

    } // end if PBS+experience
    else if (vm["experience"].as<int>() == 0){ // no experience (original PBS)

        initial_priorities = vector<vector<bool>>(al.num_of_agents, vector<bool>(al.num_of_agents, false)) ;   // boolean vector of vectors, initial to be false (NO experience)
        fallback_priorities = initial_priorities;
        cout << "Running original PBS (without initial priority ordering)" << endl;

    }

//////////////////////////////////// END - experience database //////////////////////////////////////////////////////////



    // todo remove (from original PBS implementation)
    if(vm["solver"].as<string>() == "EPEA")
	{
		EPEASearch epea(ml, al, egr);
		bool res;
		res = epea.runEPEASearch();
		ofstream stats;
		stats.open(vm["output"].as<string>(), ios::app);
		stats << epea.runtime << "," << "," << "," <<
			epea.num_expanded << "," << epea.num_generated << "," <<
			vm["agents"].as<string>() << "," << epea.solution_cost << "," << epea.solution_depth << "," <<
			vm["solver"].as<string>() << endl;
		stats.close();
	}


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	else //ICBS & CBSH
	{

        constraint_strategy s;
		if (vm["solver"].as<string>() == "ICBS")
			s = constraint_strategy::ICBS;
		else if (vm["solver"].as<string>() == "N-ICBS")
			s = constraint_strategy::N_ICBS;
		else if (vm["solver"].as<string>() == "CBSH")  //// --> use CBSH for exPBS and exRHCR. other solvers are from original PBS implementation
			s = constraint_strategy::CBSH;
		else if (vm["solver"].as<string>() == "N-CBSH")
			s = constraint_strategy::N_CBSH;
		else if (vm["solver"].as<string>() == "CBSH-CR")
			s = constraint_strategy::CBSH_CR;
		else if (vm["solver"].as<string>() == "CBSH-R")
			s = constraint_strategy::CBSH_R;
		else if (vm["solver"].as<string>() == "CBSH-RM")
			s = constraint_strategy::CBSH_RM;

		else
		{
			std::cout <<"WRONG SOLVER NAME!" << std::endl;
			return -1;
		}

		if (vm["priority"].as<int>() <= 1) {
			bool lazy_gen = true;
			if (vm["priority"].as<int>() > 0) {
				lazy_gen = false;
			}
			ICBSSearch icbs(ml, al, 1.0, egr, s, lazy_gen);
            bool res;
			res = icbs.runICBSSearch();
			if (!icbs.node_stat.empty())
			{
				ofstream stats;                                 // Stats is the output that will be saved in the "output" file
				stats.open(vm["output"].as<string>(), ios::app);
				stats << get<0>(icbs.node_stat.front()) << "," <<
					get<1>(icbs.node_stat.front()) << "," <<
					get<2>(icbs.node_stat.front()) << "," <<
					get<3>(icbs.node_stat.front()) << "," <<
					get<4>(icbs.node_stat.front()) << "," <<
					get<5>(icbs.node_stat.front()) << "," <<
					get<6>(icbs.node_stat.front()) << endl;
				stats.close();
				return 0;
			}
			ofstream stats;                                     // Stats is the output that will be saved in the "output" file
			stats.open(vm["output"].as<string>(), ios::app);
			stats << icbs.runtime << "," <<
				icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
				icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
                vm["agents"].as<string>() << "," << vm["priorityMatrix"].as<string>() << "," << icbs.solution_cost << "," <<
				icbs.min_f_val - icbs.dummy_start->g_val << "," <<
				vm["solver"].as<string>() << endl;

			stats.close();


		}
		else {                                                  // priority = 2 or 3
			bool fixed_prior = false;
            if (vm["priority"].as<int>() >= 3) {                // priority = 3 (FIX, total fixed priority ordering)
				fixed_prior = true;
			}
            /////// priority = 2 + ICBS/CBSH - use this for exPBS! //////////////
			GICBSSearch icbs(ml, al, 1.0, egr, s, initial_priorities, fallback_priorities, vm["experience"].as<int>(), vm["fallback"].as<double>(), vm["width_limit_hl"].as<int>(), vm["windowed_mapf"].as<int>(), fixed_prior); // GICBS + fixed=False for PBS //  added initial_priorities as input
            /////////////////////////////////////////////////////////////////////
			bool res;

			res = icbs.runGICBSSearch(); // <--
			if (!icbs.node_stat.empty())
			{
				ofstream stats;
				stats.open(vm["output"].as<string>(), ios::app);
				stats << get<0>(icbs.node_stat.front()) << "," <<
					get<1>(icbs.node_stat.front()) << "," <<
					get<2>(icbs.node_stat.front()) << "," <<
					get<3>(icbs.node_stat.front()) << "," <<
					get<4>(icbs.node_stat.front()) << "," <<
					get<5>(icbs.node_stat.front()) << "," <<
					get<6>(icbs.node_stat.front()) << endl;
				stats.close();
				return 0;
			}
			ofstream stats;
			stats.open(vm["output"].as<string>(), ios::app);

            // : print to Output csv file
            int database_size = vm["experience"].as<int>();
            if (vm["cleaning_threshold"].as<double>() == -2){  // that means cleaning by average - not used in exRHCR
                database_size++;
            }

            // writing output file:
            stats << fixed << setprecision(0) <<
                  icbs.runtime << "," <<
                  // load_database_runtime << "," << find_nearest_query_runtime  << "," <<
                  vm["agents"].as<string>() << "," << database_size << "," <<
                  NearestNeighborName << "," << NearestNeighborDistance << "," << //  - add (6) similarity criteria ?;
                  ConstraintsMeasure << "," <<
                  icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
                  icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
                  icbs.solution_cost << "," << icbs.SolutionDepth << "," << vm["width_limit_hl"].as<int>() << "," <<
                  setprecision(2) << vm["fallback"].as<double>()  <<"," << setprecision(0)  << icbs.max_breadth << endl;
            stats.close();
            cout << endl<< endl<< endl<< "max width = " << icbs.max_breadth <<endl <<
            "solution depth = " << icbs.SolutionDepth << endl;
            // icbs.printPaths();

            //// If windowed MAPF used, and it suffix is 'w' and solution is founded - override with next query:
            if (vm["windowed_mapf"].as<int>()>0 and vm["agents"].as<string>().at(vm["agents"].as<string>().size() -8) == 'w' and icbs.solution_cost>0)
            {
                int h = (int)vm["frequency_mapd"].as<int>();

                if (h < 1) { // enforce w = h if w is given and not h
                    h = (int)vm["windowed_mapf"].as<int>();
                }
                // overwrite the agents file with the new windowed-MAPF query
                ofstream agents_file;
                agents_file.open(vm["agents"].as<string>(), ios::trunc);
                agents_file << std::to_string(vm["agentNum"].as<int>()) << endl << icbs.printLocationsForNextWindowedMapf(h);
                agents_file.close();

            }


            //// save priorities:
            bool to_save_P_matrix = false;  // flag to avoid saving priority matrix
            if (vm["to_save_P_matrix"].as<int>() == 1){
                to_save_P_matrix = true;
            }
            if (to_save_P_matrix and vm["experience"].as<int>() == 0 and icbs.solution_cost>0) {

                ///////////////////////////// Save final priorities (_.priorities files)//
                // Save the final priorities to .priority file
                if (icbs.solution_cost > 0 && vm["experience"].as<int>() == 0 && vm["priorityMatrix"].as<int>() !=
                                                                                 -3) { // if solution founded and priority input in empty... save _.priorities file
                    string priority_fname; //priorities output file name
                    priority_fname = vm["agents"].as<string>().substr(0, vm["agents"].as<string>().size() -
                                                                         7); // same name as agents file without the suffix
                    priority_fname.append(".priorities"); // make the output suffix _.priorities

                    ofstream myfile;
                    myfile.open(priority_fname); //open file
                    //            myfile << icbs.priorities.size() << "," << icbs.priorities.size() << endl; // first line number_of_agents,number_of_agents - the matrix in kxk
                    for (int i = 0; i < (int) icbs.priorities.size(); i++) {
                        if (i > 0)
                            myfile << endl; // seperate lines
                        for (int j = 0; j < (int) icbs.priorities[0].size(); j++) {
                            if (icbs.priorities[i][j]) {
                                myfile << "@"; // make same format as map files
                            } else {
                                myfile << "."; // make same format as map files
                            }
                        } //end for j
                    } //end for i
                    myfile.close();

                    cout << "Generate file: " << priority_fname << endl;

                }  // end - if solution cost > 0 and no experience
            }
    ///////////////////////////////////////////////////////////////////////////// end save _.priorities files

		}  // end - priority = 2 or 3
	}   // END - ICBS & CBSH


	return 0;

}
