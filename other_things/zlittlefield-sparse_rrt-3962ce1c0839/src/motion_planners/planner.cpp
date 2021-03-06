/**
 * @file planner.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "motion_planners/planner.hpp"

#include <stack>

// Returns a matrix of states for Python plotting
MatrixXd planner_t::tree_to_matrix_states() {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of parents.
	MatrixXd states(4, number_of_nodes); // Hard code this shiet
	int index = 0;

	std::stack<tree_node_t*> fringe;
	fringe.push(root);
	while (fringe.size() > 0) {
		tree_node_t* candidate = fringe.top();
		fringe.pop();

		states.col(index++) << candidate->point[0], candidate->point[1], candidate->point[2], candidate->point[3];

		for (std::list<tree_node_t*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}

	return states.leftCols(index);

}

// Returns a matrix of states for Python plotting
MatrixXd planner_t::tree_to_matrix_parents() {

	// Populate this matrix by an iterative DFS. MUST follow the same protocol
	// as sister function for creating matrix of parents.
	MatrixXd states(4, number_of_nodes); // Hard code this shiet
	int index = 0;

	std::stack<tree_node_t*> fringe;
	fringe.push(root);
	while (fringe.size() > 0) {
		tree_node_t* candidate = fringe.top();
		fringe.pop();

		if (candidate != root) {
			tree_node_t* cand_parent = candidate->parent;
			states.col(index++) << cand_parent->point[0], cand_parent->point[1], cand_parent->point[2], cand_parent->point[3];
		} else {
			states.col(index++) << candidate->point[0], candidate->point[1], candidate->point[2], candidate->point[3];
		}

		for (std::list<tree_node_t*>::iterator child = candidate->children.begin(); child != candidate->children.end(); ++child) {
			fringe.push(*child);
		}
	}

	return states.leftCols(index);

}

MatrixXd planner_t::get_solution_path() {
	MatrixXd goal_states(4, number_of_nodes);

	// Fetch goal states
	std::vector<std::vector<double> > states;
	get_solution_states(states);

	int index = 0;
	for(std::vector<std::vector<double> >::iterator it = states.begin(); it != states.end(); it++) {
		std::vector<double> st = *it;
		goal_states.col(index++) << st[0], st[1], st[2], st[3]; 		
		//goal_states.col(index++) << st[0], st[1], st[2], st[3], st[4], st[5], st[6], st[7]; 
	}

	return goal_states.leftCols(index);
}

MatrixXd planner_t::get_obstacles() {

	std::string file_name = "../../random_scene_generation/double_integrator/scene_" + std::to_string(params::scene_number);

	std::ifstream fin(file_name);
	std::vector<double> data;

	std::copy(std::istream_iterator<double>(fin), // this denotes "start of stream"
        	  std::istream_iterator<double>(),   // this denodes "end of stream"
        	  std::back_inserter<std::vector<double> >(data));

	MatrixXd obstacles(4, data.size()/4); obstacles.setZero();

	int index = 0;
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < data.size()/4; j++) {
			obstacles(i, j) = data[index++];
		}
	}

	return obstacles;
}


void planner_t::set_start_state(double* in_start)
{
	if(start_state==NULL)
		start_state = system->alloc_state_point();
	system->copy_state_point(start_state,in_start);
}

void planner_t::set_goal_state(double* in_goal,double in_radius)
{
	if(goal_state==NULL)
		goal_state = system->alloc_state_point();
	system->copy_state_point(goal_state,in_goal);
	goal_radius = in_radius;
}

void planner_t::visualize_tree(int image_counter)
{
	std::stringstream s;
    s<<"tree_"<<image_counter<<".svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

    visualize_edge(root,doc,dimensions);

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->cost > x->cost)
		{
			nodes[j] = nodes[j-1];
			j--;
		}
		nodes[j] = x;
	}
}

void planner_t::visualize_nodes(int image_counter)
{
	std::stringstream s;
    s<<"nodes_"<<image_counter<<".svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));
    sorted_nodes.clear();
    get_max_cost();
    sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i],doc,dimensions);
	}

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}
void planner_t::visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

		traj_line<<system->visualize_point(node->point,dim);
		traj_line<<system->visualize_point((*i)->point,dim);
		doc<<traj_line;

		visualize_edge(*i,doc,dim);

	}

}

void planner_t::visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	svg::Circle circle(system->visualize_point(node->point,dim),params::node_diameter,svg::Fill( svg::Color((node->cost/max_cost)*255,(node->cost/max_cost)*255,(node->cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

void planner_t::visualize_solution_path( svg::Document& doc, svg::Dimensions& dim)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(last_solution_path[i]->point,dim);
		}
		doc<<traj_line;
	}
}
void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{

	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			svg::Circle circle(system->visualize_point(last_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}



void planner_t::get_max_cost()
{
	max_cost = 0;
	get_max_cost(root);
}

void planner_t::get_max_cost(tree_node_t* node)
{
	sorted_nodes.push_back(node);
	if(node->cost > max_cost)
		max_cost = node->cost;
	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i);
	}
}

