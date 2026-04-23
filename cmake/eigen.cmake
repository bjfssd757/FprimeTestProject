add_library(eigen INTERFACE)

target_include_directories(eigen INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../lib/eigen")

add_library(eigen::eigen ALIAS eigen)
