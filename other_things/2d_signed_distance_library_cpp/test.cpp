/*
 * test.cpp
 *
 *  Created on: Jul 28, 2014
 *      Author: ChrisXie
 */
#include <iostream>
#include "signedDistancePolygons.hpp"

#include <boost/python.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/tuple.hpp>
#include <boost/numpy.hpp>
#include <boost/filesystem.hpp>

namespace py = boost::python;
namespace np = boost::numpy;

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

using namespace signed_distance_2d;

inline double uniform(double low, double high) {
	return (high - low)*(rand() / double(RAND_MAX)) + low;
}

// Python plotting stuff
np::ndarray eigen_to_ndarray(const MatrixXd& m) {
	if (m.cols() == 1) {
		py::tuple shape = py::make_tuple(m.rows());
		np::dtype dtype = np::dtype::get_builtin<float>();
		np::ndarray n = np::zeros(shape, dtype);
		for(int i=0; i < m.rows(); ++i) {
			n[py::make_tuple(i)] = m(i);
		}
		return n;
	} else {
		py::tuple shape = py::make_tuple(m.rows(), m.cols());
		np::dtype dtype = np::dtype::get_builtin<float>();
		np::ndarray n = np::zeros(shape, dtype);
		for(int i=0; i < m.rows(); ++i) {
			for(int j=0; j < m.cols(); ++j) {
				n[py::make_tuple(i,j)] = m(i,j);
			}
		}

		return n;
	}
}

py::object init_display() {
    // necessary initializations
	Py_Initialize();
	np::initialize();
	py::numeric::array::set_module_and_type("numpy", "ndarray");

	// use boost to find directory of python_plot.py
	std::string working_dir = boost::filesystem::current_path().normalize().string();
	working_dir += "/python_plotting/";

	// necessary imports
	py::object main_module = py::import("__main__");
	py::object main_namespace = main_module.attr("__dict__");
	py::exec("import sys, os", main_namespace);
	// add working_dir to sys.path
	py::exec(py::str("sys.path.append('"+working_dir+"')"), main_namespace);
	// get python file module
	py::object plot_mod = py::import("python_plot_test_signed_distance");

	// get function from module
	py::object plotter = plot_mod.attr("plot");
	return plotter;
}

void plot(py::object plotter, np::ndarray t1, np::ndarray t2,
    np::ndarray points) {

	try {
		// pass control to python now
		plotter(t1, t2, points);
	}
	catch(py::error_already_set const &) {
		// will pass python errors to cpp for printing
		PyErr_Print();
	}
}

int main() {

	//std::cout << "Answer is:\n " << bools << std::endl;

	Matrix<double, 4, 2> p1;
	p1 << 0, 1,
		  0, 0,
		  1, 0,
		  1, 1;
	Matrix<double, 4, 2> p2;
	p2 << 1.5, 1.5,
		  .5, 1.5,
		  .5, .5,
		  1.5, .5;
	Matrix<double, 4, 2> p3;
	p3 << 2, 1,
		  2, 0,
		  3, 0,
		  3, 1;
	Matrix<double, 4, 2> p4;
	p4 << 1, 1,
		  1, 0,
		  2, 0,
		  2, 1;

	Matrix<double, 2, 2> point_robot;
	point_robot << .6, .6, .6, .6;

	Matrix<double, 4, 2> p5;
	p5 << .25, 2,
		  .25, -1,
		  .75, -1,
		  .75, 2;

	Matrix<double, 4, 2> p6;
	p6 << .25, .3,
		  .25, .6,
		  .75, .6,
		  .75, .3;

	Matrix<double, 4, 2> p7;
	p7 << .2, .25,
		  .2, .75,
		  .7, .75,
		  .7, .25;

	// Plot in Python
    std::cout << "Initializing display...\n";
	py::object plotter = init_display(); // Initialize python interpreter and pyplot plot

	np::ndarray p1_np = eigen_to_ndarray(p1);
	np::ndarray p2_np = eigen_to_ndarray(p2);
	np::ndarray p3_np = eigen_to_ndarray(p3);
	np::ndarray p4_np = eigen_to_ndarray(p4);
	np::ndarray p5_np = eigen_to_ndarray(p5);
	np::ndarray p6_np = eigen_to_ndarray(p6);
	np::ndarray p7_np = eigen_to_ndarray(p7);
	np::ndarray point_robot_np = eigen_to_ndarray(point_robot);

	// Plot random triangles between [0, 1] x [0, 1]
	srand(time(NULL));
	Matrix<double, 3, 2> t1;
	Matrix<double, 3, 2> t2;
	int j=0,k=0;
	for (int i = 0; i < 6; i++) {
		t1(j,k) = uniform(0,1);
		if (++j == 3) {
			j = 0;
			k++;
		}
	}
	j=0;k=0;
	for (int i = 0; i < 6; i++) {
		t2(j,k) = uniform(0,1);
		if (++j == 3) {
			j = 0;
			k++;
		}
	}

	t1 << .5, .48,
		  .5, .52,
		  .5, .48;
	t2 << .1, .1,
		   1, .5,
		   0, 1;

	MatrixXd ans = signedDistancePolygons(t1, t2);
	np::ndarray ans_np = eigen_to_ndarray(ans);
	np::ndarray t1_np = eigen_to_ndarray(t1);
	np::ndarray t2_np = eigen_to_ndarray(t2);

	std::cout << "Plotting...\n";
	plot(plotter, t1_np, t2_np, ans_np);

}


