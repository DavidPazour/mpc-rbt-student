ament_add_gtest(solutionTest solutionTest.cpp
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        APPEND_LIBRARY_DIRS ${PROJECT_NAME}
        TIMEOUT 900
)
target_link_libraries(solutionTest ${PROJECT_NAME})
