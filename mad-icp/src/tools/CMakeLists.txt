add_library(tools SHARED
  mad_tree.cpp
)

target_link_libraries(tools)
target_compile_features(tools PUBLIC)