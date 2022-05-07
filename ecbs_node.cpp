#include "ecbs_node.h"
#include <vector>

ECBSNode::ECBSNode() {
  agent_id = -1;
  //constraints[0] = make_tuple(-1, -1, -1);
  g_val = 0;
  num_of_collisions = 0;
  time_expanded = -1;
  sum_min_f_vals = -1;
  //path_cost = -1;
}

ECBSNode::ECBSNode(int agent_id, ECBSNode* parent, double g_val, double num_of_collisions, int time_expanded, double sum_min_f_vals):
    agent_id(agent_id), parent(parent), g_val(g_val), num_of_collisions(num_of_collisions), time_expanded(time_expanded), sum_min_f_vals(sum_min_f_vals) {
  //constraint = make_tuple(-1, -1, -1);
}


bool ECBSNode::isEqual(const ECBSNode* n1, const ECBSNode* n2) {
  return (n1->parent == n2->parent &&
          n1->agent_id == n2->agent_id &&
          n1->constraint== n2->constraint);
  // $$$ -- should we add time_expanded? should we have only time_expanded?
  // $$$ -- anyone calls this function?
}


ECBSNode::~ECBSNode() {
}


std::ostream& operator<<(std::ostream& os, const ECBSNode& n) {
  os << "THIS NODE HAS: g_val=" << n.g_val << " and num_of_collisions=" << n.num_of_collisions << ". It constrains agent " << n.agent_id <<
      " on loc1[" << get<0>(n.constraint) << "] to loc2[" << get<1>(n.constraint) << "] at time[" << get<2>(n.constraint) << "]" <<
      " ; And sum_min_f_vals=" << n.sum_min_f_vals << endl;
  return os;
}
