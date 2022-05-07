#include "map_loader.h"
#include "agents_loader.h"
#include "egraph_reader.h"
//#include "ecbs_search.h"
#include "epea_search.h"
#include "ICBSSearch.h"

#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
// #include "ecbs_node.h"
#include <cstdlib>
#include <cmath>
#include "Timer.hpp"

#include "boost/program_options.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include<boost/tokenizer.hpp>




namespace pt = boost::property_tree;
using namespace std;

int main(int argc, char** argv) {

	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("map,m", po::value<std::string>()->required(), "input file for map")
		("agents,a", po::value<std::string>()->required(), "input file for agents")
		("output,o", po::value<std::string>()->required(), "output file for schedule")
		("solver,s", po::value<std::string>()->required(), "solvers (EPEA, ECBS, ICBS, CBSH, N-ECBS, N-ICBS, N-CBSH, CBSH-CR, CBSH-R, CBSH-RM")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("priority,p", po::value<int>()->default_value(0), "priority branching")
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

	// read the egraph --- we don't use highway here
	EgraphReader egr;
 
	
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
	//else if (vm["solver"].as<string>() == "ECBS" || vm["solver"].as<string>() == "N_ECBS") //ECBS
	//{
	//	constraint_strategy s;
	//	if(vm["solver"].as<string>() == "ECBS")
	//		s = constraint_strategy::ECBS;
	//	else
	//		s = constraint_strategy::N_ECBS;
	//	ECBSSearch ecbs(ml, al, egr, 1.0, 1.0, false, s);
	//	bool res;
	//	res = ecbs.runECBSSearch();
	//	ofstream stats;
	//	stats.open(vm["output"].as<string>(), ios::app);
	//	stats << ecbs.runtime << "," <<
	//		ecbs.HL_num_expanded << "," << ecbs.HL_num_generated << "," <<
	//		ecbs.LL_num_expanded << "," << ecbs.LL_num_generated << "," <<
	//		vm["agents"].as<string>() << "," << ecbs.solution_cost << "," <<
	//		ecbs.min_sum_f_vals - ecbs.dummy_start->g_val << "," <<
	//		vm["solver"].as<string>() << endl;
	//	stats.close();
	//}
	else //ICBS & CBSH
	{ 
		constraint_strategy s;
		if (vm["solver"].as<string>() == "ICBS")
			s = constraint_strategy::ICBS;
		else if (vm["solver"].as<string>() == "N-ICBS")
			s = constraint_strategy::N_ICBS;
		else if (vm["solver"].as<string>() == "CBSH")
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

		bool priority_branching = false;
		bool trans_priority = false;
		if (vm["priority"].as<int>() > 0) {
			priority_branching = true;
		}
		if (vm["priority"].as<int>() > 1) {
			trans_priority = true;
		}
		ICBSSearch icbs(ml, al, 1.0, egr, s, priority_branching, trans_priority);
		bool res;
		res = icbs.runICBSSearch();
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
		stats << icbs.runtime << "," <<
			icbs.HL_num_expanded << "," << icbs.HL_num_generated << "," <<
			icbs.LL_num_expanded << "," << icbs.LL_num_generated << "," <<
			vm["agents"].as<string>() << "," << icbs.solution_cost << "," << 
			icbs.min_f_val - icbs.dummy_start->g_val << "," <<
			vm["solver"].as<string>() << vm["priority"].as<int>() << endl;
		stats.close();
	}

	return 0;

}
