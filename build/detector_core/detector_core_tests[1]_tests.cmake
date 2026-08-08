add_test([=[Point2DTest.StoresCoordinates]=]  /home/jayce/projects/LiDar-Object-Detector/build/detector_core/detector_core_tests [==[--gtest_filter=Point2DTest.StoresCoordinates]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[Point2DTest.StoresCoordinates]=]  PROPERTIES WORKING_DIRECTORY /home/jayce/projects/LiDar-Object-Detector/build/detector_core SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  detector_core_tests_TESTS Point2DTest.StoresCoordinates)
