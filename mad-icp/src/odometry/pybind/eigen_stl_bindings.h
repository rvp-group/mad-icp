// This file has been adapted from Open3D, specialized only 
// on std::vector of Eigen::Vector3d using the following files: 
// https://github.com/isl-org/Open3D/blob/main/cpp/pybind/open3d_pybind.h 
// https://github.com/isl-org/Open3D/blob/main/cpp/pybind/utility/eigen.cpp

// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

namespace py11 = pybind11;

namespace pybind11 {

py11::handle static_property =
      py11::handle((PyObject*)py11::detail::get_internals().static_property_type);

template <typename Vector,
          typename holder_type = std::unique_ptr<Vector>,
          typename... Args>
py11::class_<Vector, holder_type> bind_vector_without_repr(
        py11::module &m, std::string const &name, Args &&... args) {
    // hack function to disable __repr__ for the convenient function
    // bind_vector()
    using Class_ = py11::class_<Vector, holder_type>;
    Class_ cl(m, name.c_str(), std::forward<Args>(args)...);
    cl.def(py11::init<>());
    cl.def(
            "__bool__", [](const Vector &v) -> bool { return !v.empty(); },
            "Check whether the list is nonempty");
    cl.def("__len__", &Vector::size);
    return cl;
}

// - This function is used by Pybind for std::vector<SomeEigenType> constructor.
//   This optional constructor is added to avoid too many Python <-> C++ API
//   calls when the vector size is large using the default biding method.
//   Pybind matches np.float64 array to py11::array_t<double> buffer.
// - Directly using templates for the py11::array_t<double> and py11::array_t<int>
//   and etc. doesn't work. The current solution is to explicitly implement
//   bindings for each py array types.
template <typename EigenVector>
std::vector<EigenVector> py_array_to_vectors_double(
        py11::array_t<double, py11::array::c_style | py11::array::forcecast> array) {
    int64_t eigen_vector_size = EigenVector::SizeAtCompileTime;
    if (array.ndim() != 2 || array.shape(1) != eigen_vector_size) {
        throw py11::cast_error();
    }
    std::vector<EigenVector> eigen_vectors(array.shape(0));
    auto array_unchecked = array.mutable_unchecked<2>();
    for (auto i = 0; i < array_unchecked.shape(0); ++i) {
        // The EigenVector here must be a double-typed eigen vector, since only
        // open3d::Vector3dVector binds to py_array_to_vectors_double.
        // Therefore, we can use the memory map directly.
        eigen_vectors[i] = Eigen::Map<EigenVector>(&array_unchecked(i, 0));
    }
    return eigen_vectors;
}

}  // namespace pybind11

namespace {

template <typename EigenVector,
          typename Vector = std::vector<EigenVector>,
          typename holder_type = std::unique_ptr<Vector>,
          typename InitFunc>
py11::class_<Vector, holder_type> pybind_eigen_vector_of_vector(
        py11::module &m,
        const std::string &bind_name,
        const std::string &repr_name,
        InitFunc init_func) {
    typedef typename EigenVector::Scalar Scalar;
    auto vec = py11::bind_vector_without_repr<std::vector<EigenVector>>(
            m, bind_name, py11::buffer_protocol());
    vec.def(py11::init(init_func));
    vec.def_buffer([](std::vector<EigenVector> &v) -> py11::buffer_info {
        size_t rows = EigenVector::RowsAtCompileTime;
        return py11::buffer_info(v.data(), sizeof(Scalar),
                               py11::format_descriptor<Scalar>::format(), 2,
                               {v.size(), rows},
                               {sizeof(EigenVector), sizeof(Scalar)});
    });
    vec.def("__repr__", [repr_name](const std::vector<EigenVector> &v) {
        return repr_name + std::string(" with ") + std::to_string(v.size()) +
               std::string(" elements.\n") +
               std::string("Use numpy.asarray() to access data.");
    });
    vec.def("__copy__", [](std::vector<EigenVector> &v) {
        return std::vector<EigenVector>(v);
    });
    vec.def("__deepcopy__", [](std::vector<EigenVector> &v, py11::dict &memo) {
        return std::vector<EigenVector>(v);
    });

    // py11::detail must be after custom constructor
    using Class_ = py11::class_<Vector, std::unique_ptr<Vector>>;
    py11::detail::vector_if_copy_constructible<Vector, Class_>(vec);
    py11::detail::vector_if_equal_operator<Vector, Class_>(vec);
    py11::detail::vector_modifiers<Vector, Class_>(vec);
    py11::detail::vector_accessor<Vector, Class_>(vec);

    return vec;
}

template <typename EigenVector,
          typename EigenAllocator = Eigen::aligned_allocator<EigenVector>,
          typename Vector = std::vector<EigenVector, EigenAllocator>,
          typename holder_type = std::unique_ptr<Vector>,
          typename InitFunc>
py11::class_<Vector, holder_type> pybind_eigen_vector_of_vector_eigen_allocator(
        py11::module &m,
        const std::string &bind_name,
        const std::string &repr_name,
        InitFunc init_func) {
    typedef typename EigenVector::Scalar Scalar;
    auto vec = py11::bind_vector_without_repr<
            std::vector<EigenVector, EigenAllocator>>(m, bind_name,
                                                      py11::buffer_protocol());
    vec.def(py11::init(init_func));
    vec.def_buffer(
            [](std::vector<EigenVector, EigenAllocator> &v) -> py11::buffer_info {
                size_t rows = EigenVector::RowsAtCompileTime;
                return py11::buffer_info(v.data(), sizeof(Scalar),
                                       py11::format_descriptor<Scalar>::format(),
                                       2, {v.size(), rows},
                                       {sizeof(EigenVector), sizeof(Scalar)});
            });
    vec.def("__repr__",
            [repr_name](const std::vector<EigenVector, EigenAllocator> &v) {
                return repr_name + std::string(" with ") +
                       std::to_string(v.size()) + std::string(" elements.\n") +
                       std::string("Use numpy.asarray() to access data.");
            });
    vec.def("__copy__", [](std::vector<EigenVector, EigenAllocator> &v) {
        return std::vector<EigenVector, EigenAllocator>(v);
    });
    vec.def("__deepcopy__",
            [](std::vector<EigenVector, EigenAllocator> &v, py11::dict &memo) {
                return std::vector<EigenVector, EigenAllocator>(v);
            });

    // py11::detail must be after custom constructor
    using Class_ = py11::class_<Vector, std::unique_ptr<Vector>>;
    py11::detail::vector_if_copy_constructible<Vector, Class_>(vec);
    py11::detail::vector_if_equal_operator<Vector, Class_>(vec);
    py11::detail::vector_modifiers<Vector, Class_>(vec);
    py11::detail::vector_accessor<Vector, Class_>(vec);

    return vec;
}

}  // unnamed namespace