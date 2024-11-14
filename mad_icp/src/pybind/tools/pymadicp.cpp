// Copyright 2024 R(obots) V(ision) and P(erception) group
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "mad_icp_wrapper.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <tools/constants.h>

namespace py = pybind11;

PYBIND11_MODULE(pymadicp, m) {
  py::class_<MADicpWrapper>(m, "MADicp")
    .def(py::init<int>(), py::arg("num_threads"))
    .def("setQueryCloud", &MADicpWrapper::setQueryCloud, py::arg("query"), py::arg("b_max") = 0.2, py::arg("b_min") = 0.1)
    .def("setReferenceCloud",
         &MADicpWrapper::setReferenceCloud,
         py::arg("reference"),
         py::arg("b_max") = 0.2,
         py::arg("b_min") = 0.1)
    .def("compute",
         &MADicpWrapper::compute,
         py::arg("T"),
         py::arg("icp_iterations") = MAX_ICP_ITS,
         py::arg("rho_ker")        = 0.1,
         py::arg("b_ratio")        = 0.02,
         py::arg("print_stats")    = false);
}
