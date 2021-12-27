#add_library(cvTest SHARED helloWorld.cpp)
add_executable(opencv_test.app opencv_test.cpp)
target_link_libraries(opencv_test.app      ${THIRD_PARTY_LIBS})