#include <gtest/gtest.h>
#include <ros/package.h>
#include "dual_quaternion_registration.h"
TEST (SquareRootTest, ZeroAndNegativeNos) { 
    ASSERT_EQ (0.0, 0.0);
    ASSERT_EQ (-1, 0);
    
}
int main(int argc, char **argv) {
  std::string str1 = "hello.txt";
  std::string str2 = "goodbye.txt";
  // long double* result = qf_register(str1.c_str(), str1.c_str(),
		// 							1.0, 100, 20,
		// 							.001, .001, 300);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}