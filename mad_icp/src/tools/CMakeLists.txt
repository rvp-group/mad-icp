add_library(tools STATIC
  mad_tree.cpp
)

target_link_libraries(tools)
target_compile_features(tools PUBLIC)