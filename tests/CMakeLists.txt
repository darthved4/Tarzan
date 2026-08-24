cmake_minimum_required(VERSION 3.20)
project(TarzanHostTests LANGUAGES C)

enable_testing()

add_executable(pid_test
    pid_test.c
    ../lib/pid.c
)
target_include_directories(pid_test PRIVATE ../include)
target_link_libraries(pid_test PRIVATE m)
target_compile_features(pid_test PRIVATE c_std_11)
target_compile_options(pid_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME pid_test COMMAND pid_test)

add_executable(madgwick_filter_test
    madgwick_filter_test.c
    ../lib/madgwick.c
)
target_include_directories(madgwick_filter_test PRIVATE ../include)
target_link_libraries(madgwick_filter_test PRIVATE m)
target_compile_features(madgwick_filter_test PRIVATE c_std_11)
target_compile_options(madgwick_filter_test PRIVATE -Wall -Wextra -Wpedantic)
add_test(NAME madgwick_filter_test COMMAND madgwick_filter_test)
