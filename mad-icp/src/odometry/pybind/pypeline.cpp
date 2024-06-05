// pybind11
#include <pybind11/chrono.h>
#include <pybind11/complex.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

// std stuff
#include <Eigen/Core>
#include <memory>
#include <vector>

#include "../pipeline.h"
#include "eigen_stl_bindings.h"

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);

namespace py11 = pybind11;
using namespace py11::literals;

PYBIND11_MODULE(pypeline, m) {
  auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
    m, "VectorEigen3d", "std::vector<Eigen::Vector3d>", py11::py_array_to_vectors_double<Eigen::Vector3d>);

  auto pipeline = py11::class_<Pipeline>(m, "Pipeline")
                    .def(py11::init<double, bool, double, double, double, double, double, int, int, bool>(),
                         py11::arg("sensor_hz"),
                         py11::arg("deskew"),
                         py11::arg("b_max"),
                         py11::arg("rho_ker"),
                         py11::arg("p_th"),
                         py11::arg("b_min"),
                         py11::arg("b_ratio"),
                         py11::arg("num_keyframes"),
                         py11::arg("num_threads"),
                         py11::arg("realtime"))
                    .def("currentPose", &Pipeline::currentPose)
                    .def("trajectory", &Pipeline::trajectory)
                    .def("keyframePose", &Pipeline::keyframePose)
                    .def("isInitialized", &Pipeline::isInitialized)
                    .def("isMapUpdated", &Pipeline::isMapUpdated)
                    .def("currentID", &Pipeline::currentID)
                    .def("keyframeID", &Pipeline::keyframeID)
                    .def("modelLeaves", &Pipeline::modelLeaves)
                    .def("currentLeaves", &Pipeline::currentLeaves)
                    .def("currentVelocity", &Pipeline::currentVelocity)
                    .def("compute", &Pipeline::compute);
}
