# CMake generated Testfile for 
# Source directory: /home/d/Kuka_moveIt2/kuka_agilus_support
# Build directory: /home/d/Kuka_moveIt2/build/kuka_agilus_support
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_test_kr_agilus.py "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/d/Kuka_moveIt2/build/kuka_agilus_support/test_results/kuka_agilus_support/test_test_kr_agilus.py.xunit.xml" "--package-name" "kuka_agilus_support" "--output-file" "/home/d/Kuka_moveIt2/build/kuka_agilus_support/launch_test/test_test_kr_agilus.py.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/d/Kuka_moveIt2/kuka_agilus_support/test/test_kr_agilus.py" "--junit-xml=/home/d/Kuka_moveIt2/build/kuka_agilus_support/test_results/kuka_agilus_support/test_test_kr_agilus.py.xunit.xml" "--package-name=kuka_agilus_support")
set_tests_properties(test_test_kr_agilus.py PROPERTIES  LABELS "launch_test" TIMEOUT "60" WORKING_DIRECTORY "/home/d/Kuka_moveIt2/build/kuka_agilus_support" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/d/Kuka_moveIt2/kuka_agilus_support/CMakeLists.txt;16;add_launch_test;/home/d/Kuka_moveIt2/kuka_agilus_support/CMakeLists.txt;0;")
