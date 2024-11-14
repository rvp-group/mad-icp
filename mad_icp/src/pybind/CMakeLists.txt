# simple std vector of eigen vectors
pybind11_add_module(pyvector MODULE pyvector.cpp)
target_link_libraries(pyvector PRIVATE
    Eigen3::Eigen
)

install(TARGETS pyvector DESTINATION .)

# lidar odometry
pybind11_add_module(pypeline MODULE pypeline.cpp)
target_link_libraries(pypeline PRIVATE
    Eigen3::Eigen
    odometry
    tools
)

install(TARGETS pypeline DESTINATION .)

########## others utils ##########

# nn search with madtree
pybind11_add_module(pymadtree MODULE tools/pymadtree.cpp)
target_link_libraries(pymadtree PRIVATE
    Eigen3::Eigen
    tools
)

install(TARGETS pymadtree DESTINATION .)

# align two clouds with madicp
pybind11_add_module(pymadicp MODULE tools/pymadicp.cpp)
target_link_libraries(pymadicp PRIVATE
    Eigen3::Eigen
    odometry
    tools
)

install(TARGETS pymadicp DESTINATION .)