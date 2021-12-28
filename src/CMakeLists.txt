#add_library(cvTest SHARED helloWorld.cpp)
add_executable(opencv_test.app opencv_test.cpp)
add_executable(g2o_test.app g2o_test.cpp)

target_link_libraries(opencv_test.app      ${THIRD_PARTY_LIBS})
target_link_libraries(g2o_test.app      ${THIRD_PARTY_LIBS})