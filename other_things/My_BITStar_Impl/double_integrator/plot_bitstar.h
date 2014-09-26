#ifndef DOUBLE_INTEGRATOR_PLOT_BITSTAR_H_
#define DOUBLE_INTEGRATOR_PLOT_BITSTAR_H_

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include <boost/python.hpp>
#include <boost/python/numeric.hpp>
#include <boost/python/tuple.hpp>
#include <boost/numpy.hpp>
//#include <boost/filesystem.hpp>

namespace py = boost::python;
namespace np = boost::numpy;

np::ndarray di_eigen_to_ndarray(const MatrixXd& m) {
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

py::object di_init_display() {
	// necessary initializations
	Py_Initialize();
	np::initialize();
	py::numeric::array::set_module_and_type("numpy", "ndarray");

	// use boost to find directory of python_plot.py
	std::string working_dir = "/Users/ChrisXie/school/research/RRTSTAR_TrajOpt_Project/RRTSTAR_and_TrajOpt/other_things/My_BITStar_Impl/double_integrator";
	std::string plot_cpp_dir = working_dir + "/";

	// necessary imports
	py::object main_module = py::import("__main__");
	py::object main_namespace = main_module.attr("__dict__");
	py::exec("import sys, os", main_namespace);
	// add plot_cpp_dir to sys.path
	py::exec(py::str("sys.path.append('"+plot_cpp_dir+"')"), main_namespace);
	// get python file module
	py::object plot_mod = py::import("python_plot");

	// get function from module
	py::object plotter = plot_mod.attr("plot");
	return plotter;
}

void di_plot(py::object& plotter, np::ndarray& states, np::ndarray& parents, np::ndarray& goal_path, np::ndarray& obstacles, int iters, double cost) {

	try {
		// pass control to python now
		plotter(states, parents, goal_path, obstacles, iters, cost);
	}
	catch(py::error_already_set const &) {
		// will pass python errors to cpp for printing
		PyErr_Print();
	}
}

#endif
