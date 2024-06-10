pybind11_add_module(pypeline MODULE pypeline.cpp)
target_link_libraries(pypeline PRIVATE
    odometry
    tools
)

install(TARGETS pypeline DESTINATION .)