/*
 * test_grid_map.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(nullptr));
  return RUN_ALL_TESTS();
}
