add_library(algorithm)

target_sources(algorithm
    PRIVATE
    src/PIDAlgorithm.cpp
    PUBLIC
    include/PIDAlgorithm.hpp)

target_include_directories(algorithm PUBLIC include)

add_subdirectory(test)