CXX = g++ # the compiler

CXXFLAGS = -Wall -DNDEBUG -I /usr/include/boost/ -I /usr/local/lib -std=c++11 -lboost_program_options -lboost_graph -O3 # the compiler flags (add -funroll-loops -finline-functions?)

#add -DNDEBUG for better Boost performance (when done debugging) and shut down my cout-s
#add -O3 for optimization (when done debugging)
#remove -g (when done debugging)
#to debug: gdb driver (then run and parameters after 'r', e.g., r --map <your_map.map> --agents <your_agents.agents> ...)
#debug gdb commands: backtrace

map_loader.o: map_loader.cpp map_loader.h
	$(CXX) $(CXXFLAGS) -c map_loader.cpp

agents_loader.o: agents_loader.cpp agents_loader.h
	$(CXX) $(CXXFLAGS) -c agents_loader.cpp

compute_heuristic.o: compute_heuristic.cpp compute_heuristic.h
	$(CXX) $(CXXFLAGS) -c compute_heuristic.cpp

node.o: node.cpp node.h
	$(CXX) $(CXXFLAGS) -c node.cpp

MDD.o: MDD.cpp MDD.h
	$(CXX) $(CXXFLAGS) -c MDD.cpp

LLNode.o: LLNode.cpp LLNode.h
	$(CXX) $(CXXFLAGS) -c LLNode.cpp

egraph_reader.o: egraph_reader.cpp egraph_reader.h
	$(CXX) $(CXXFLAGS) -c egraph_reader.cpp

single_agent_ecbs.o: single_agent_ecbs.cpp single_agent_ecbs.h
	$(CXX) $(CXXFLAGS) -c single_agent_ecbs.cpp

SingleAgentICBS.o: SingleAgentICBS.cpp SingleAgentICBS.h
	$(CXX) $(CXXFLAGS) -c SingleAgentICBS.cpp

ecbs_search.o: ecbs_search.cpp ecbs_search.h
	$(CXX) $(CXXFLAGS) -c ecbs_search.cpp

epea_node.o: epea_node.cpp epea_node.h
	$(CXX) $(CXXFLAGS) -c epea_node.cpp

epea_search.o: epea_search.cpp epea_search.h
	$(CXX) $(CXXFLAGS) -c epea_search.cpp

GICBSNode.o: GICBSNode.cpp GICBSNode.h
	$(CXX) $(CXXFLAGS) -c GICBSNode.cpp

GICBSSearch.o: GICBSSearch.cpp GICBSSearch.h
	$(CXX) $(CXXFLAGS) -c GICBSSearch.cpp

ICBSNode.o: ICBSNode.cpp ICBSNode.h
	$(CXX) $(CXXFLAGS) -c ICBSNode.cpp

ICBSSearch.o: ICBSSearch.cpp ICBSSearch.h
	$(CXX) $(CXXFLAGS) -c ICBSSearch.cpp

ecbs_node.o: ecbs_node.cpp ecbs_node.h
	$(CXX) $(CXXFLAGS) -c ecbs_node.cpp

utils_functions.o: utils_functions.cpp utils_functions.h
	$(CXX) $(CXXFLAGS) -c utils_functions.cpp

driver.o: driver.cpp
	$(CXX) $(CXXFLAGS) $(GUIFLAGS) -c driver.cpp

clean:
	rm -rf *o *~ driver driver.log core

backup:
	tar czvf backup.tar.gz *h *cpp *cxx Makefile

executable: driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o single_agent_ecbs.o ecbs_node.o ecbs_search.o ICBSSearch.o ICBSNode.o GICBSSearch.o GICBSNode.o epea_search.o epea_node.o SingleAgentICBS.o LLNode.o utils_functions.o
	$(CXX) driver.o map_loader.o agents_loader.o egraph_reader.o compute_heuristic.o node.o single_agent_ecbs.o ecbs_node.o ecbs_search.o ICBSSearch.o ICBSNode.o GICBSSearch.o GICBSNode.o epea_search.o epea_node.o SingleAgentICBS.o LLNode.o utils_functions.o $(CXXFLAGS) -o driver

profiler:
	echo "zzz" >> callgrind.out
	rm callgrind.out*
	valgrind --tool=callgrind ./driver corridor1.map corridor1.agents corridor1.hwy 2.0 1.1
	kcachegrind callgrind.out*

make_tags:
	echo "find . -maxdepth 1 -regex '.*/.*\.\(c\|cpp\|h\|cxx\)$' -print | etags -"
