#include <iostream>
#include <sstream>
#include<string>
#include <cassert>
#include <gtest/gtest.h>
#include <turtlelib/rigid2d.hpp>


using namespace turtlelib;
using namespace std;

struct Test_Vector2D_Input: public testing::TestWithParam<std::tuple<string, Vector2D>> {};
struct Test_Vector2D_Normalization: public testing::TestWithParam<Vector2D> {};
struct Test_Twist2D_Input: public testing::TestWithParam<std::tuple<string, Twist2D>> {};
struct Test_Transform2D_Constructors: public testing::TestWithParam<std::tuple<Transform2D, float, float, float>> {};



TEST_P(Test_Vector2D_Input, vector_2d_input) {

   string input = std::get<0>(GetParam());
   Vector2D expected = std::get<1>(GetParam());
   
   stringstream stream(input);
   Vector2D read_vector;
   stream >> read_vector;
  
  EXPECT_TRUE(expected == read_vector);
}


TEST_P(Test_Vector2D_Normalization, vector_2d_normalization) {
   Vector2D input = GetParam();
   Vector2D normalized = input.normalize();
   EXPECT_TRUE(almost_equal(normalized.magnitude(), 1.));
}


TEST_P(Test_Twist2D_Input, twist_2d_input) {

   string input = std::get<0>(GetParam());
   Twist2D expected = std::get<1>(GetParam());
   
   stringstream stream(input);
   Twist2D read_twist;
   stream >> read_twist;
  
  EXPECT_TRUE(expected == read_twist);
}


TEST_P(Test_Transform2D_Constructors, transform_2d_constructors) {

    Transform2D input = std::get<0>(GetParam());
    float expected_x = std::get<1>(GetParam());
    float expected_y = std::get<2>(GetParam());
    float expected_theta = std::get<3>(GetParam());
    Vector2D position = input.translation();

    EXPECT_TRUE(position.x == expected_x);
    EXPECT_TRUE(position.y == expected_y);
    EXPECT_TRUE(input.rotation() == expected_theta);
}




INSTANTIATE_TEST_SUITE_P(
    read_vector2d_test_suite,
    Test_Vector2D_Input,
    testing::Values(std::make_tuple("[0., 0.]", Vector2D(0., 0.)), 
                    std::make_tuple("[1., 2.]", Vector2D(1., 2.)),
                    std::make_tuple("1, 5", Vector2D(1., 5.))
    ));

INSTANTIATE_TEST_SUITE_P(
    test_vector_normalization_test_suite,
    Test_Vector2D_Normalization,
    testing::Values(Vector2D(5., 200.), Vector2D(5., 200.), Vector2D(-0.5, 0.01)
    
    ));

INSTANTIATE_TEST_SUITE_P(
    read_twist_2d_test_suite,
    Test_Twist2D_Input,
    testing::Values(std::make_tuple("[0., 0., 0.]", Twist2D(0., 0., 0.)),
                    std::make_tuple("[3., 1., 2.]", Twist2D(1., 2., 3.)),
                    std::make_tuple("3., 1., 2.", Twist2D(1., 2., 3.))
    ));

INSTANTIATE_TEST_SUITE_P(
    transform_2d_constructors_test_suite,
    Test_Transform2D_Constructors,
    testing::Values(std::make_tuple(Transform2D(Vector2D(0., 0.), 2.), 0., 0., 2.),
                    std::make_tuple(Transform2D(Vector2D(1, 2.), -0.5), 1., 2., -0.5)       
                    
    ));


TEST(rigid2d, vector){

    Vector2D v;
    ASSERT_EQ(4, 2 + 2);
}








int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}