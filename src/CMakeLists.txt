#add_library(cvTest SHARED helloWorld.cpp)
add_executable(opencv_test.test opencv_test.cpp)
add_executable(g2o_test.test g2o_test.cpp)
add_executable(mouseTracking.app mouseTracking.cpp)

target_link_libraries(opencv_test.test      ${THIRD_PARTY_LIBS})
target_link_libraries(g2o_test.test     ${THIRD_PARTY_LIBS})
target_link_libraries(mouseTracking.app    ${THIRD_PARTY_LIBS})