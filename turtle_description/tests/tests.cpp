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
struct Test_Transforming_Vector: public testing::TestWithParam<std::tuple<Transform2D, Vector2D, Vector2D>> {};
struct Test_Transforming_Twist2D: public testing::TestWithParam<std::tuple<Transform2D, Twist2D, Twist2D>> {};
struct Test_Transform_Equals: public testing::TestWithParam<std::tuple<Transform2D, Transform2D, bool>> {};
struct Test_Transform_Inverse: public testing::TestWithParam<Transform2D> {};
struct Test_Transform_Composition: public testing::TestWithParam<std::tuple<Transform2D, Transform2D, Transform2D>> {};
struct Test_Transform_Composition_Assignment: public testing::TestWithParam<std::tuple<Transform2D, Transform2D, Transform2D>> {};
struct Test_Twist_Equals: public testing::TestWithParam<std::tuple<Twist2D, Twist2D, bool>> {};
struct Test_Twist_Transformation: public testing::TestWithParam<std::tuple<Transform2D, Twist2D, Twist2D>> {};




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

    EXPECT_TRUE(almost_equal(position.x, expected_x));
    EXPECT_TRUE(almost_equal(position.y, expected_y));
    EXPECT_TRUE(almost_equal(input.rotation(), expected_theta, 0.001));
}


TEST_P(Test_Transforming_Vector, transform_vector) {

    Transform2D transform = std::get<0>(GetParam());
    Vector2D input_vector = std::get<1>(GetParam());
    Vector2D expected_transformed_vector = std::get<2>(GetParam());

    Vector2D transformed_vector = transform(input_vector);

    EXPECT_TRUE(transformed_vector == expected_transformed_vector);
}

TEST_P(Test_Transforming_Twist2D, transform_twist) {

    Transform2D transform = std::get<0>(GetParam());
    Twist2D input_twist = std::get<1>(GetParam());
    Twist2D expected_twist = std::get<2>(GetParam());

    Twist2D transformed_twist = transform(input_twist);

    EXPECT_TRUE(transformed_twist == expected_twist);
}


TEST_P(Test_Transform_Equals, transform_equals) {

    Transform2D transform_1 = std::get<0>(GetParam());
    Transform2D transform_2 = std::get<1>(GetParam());
    bool expected_equal = std::get<2>(GetParam());

    EXPECT_TRUE((transform_1 == transform_2) == expected_equal);
}

TEST_P(Test_Transform_Inverse, transform_inverse) {

    Transform2D transform = GetParam();
    EXPECT_TRUE(transform == transform.inv().inv());
}

TEST_P(Test_Transform_Composition, transform_composition) {

    Transform2D a_b = std::get<0>(GetParam());
    Transform2D b_c = std::get<1>(GetParam());
    Transform2D a_c = std::get<2>(GetParam());
    EXPECT_TRUE((a_b * b_c) == a_c);
}

TEST_P(Test_Transform_Composition_Assignment, transform_composition_assignment) {

    Transform2D a_b = std::get<0>(GetParam());
    Transform2D b_c = std::get<1>(GetParam());
    Transform2D a_c = std::get<2>(GetParam());
    a_b *= b_c;
    EXPECT_TRUE(a_b == a_c);
}

TEST_P(Test_Twist_Equals, twist_equals) {

    Twist2D a = std::get<0>(GetParam());
    Twist2D b = std::get<1>(GetParam());
    bool expected_equals = std::get<2>(GetParam());
    
    EXPECT_TRUE((a == b) == expected_equals);
}

TEST_P(Test_Twist_Transformation, test_twist_transformation) {

    Transform2D transform = std::get<0>(GetParam());
    Twist2D twist = std::get<1>(GetParam());
    Twist2D expected_twist = std::get<2>(GetParam());

    Twist2D transformed_twist = transform(twist);

    EXPECT_TRUE(transformed_twist == expected_twist);
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
    testing::Values(std::make_tuple(Transform2D(), 0., 0., 0.),
                    std::make_tuple(Transform2D(3.1), 0., 0., 3.1),
                    std::make_tuple(Transform2D(Vector2D(0., 0.), 2.), 0., 0., 2.),
                    std::make_tuple(Transform2D(Vector2D(1, 2.), -0.5), 1., 2., -0.5)
    ));

INSTANTIATE_TEST_SUITE_P(
    transform_vectors_test_suite,
    Test_Transforming_Vector,
    testing::Values(std::make_tuple(Transform2D(), Vector2D(10., 20.), Vector2D(10., 20.))
    ));

INSTANTIATE_TEST_SUITE_P(
    transform_twists_test_suite,
    Test_Transforming_Twist2D,
    testing::Values(std::make_tuple(Transform2D(), Twist2D(0., 0., 0.), Twist2D(0., 0., 0.)),
                    std::make_tuple(Transform2D(Vector2D(0., 1.), deg2rad(90)), Twist2D(1., 1., 1.), Twist2D(0., 1., 1.))
));

INSTANTIATE_TEST_SUITE_P(
    transform_equals_test_suite,
    Test_Transform_Equals,
    testing::Values(std::make_tuple(Transform2D(), Transform2D(), true),
                    std::make_tuple(Transform2D(PI), Transform2D(PI), true),
                    std::make_tuple(Transform2D(PI/2), Transform2D(PI), false)

));

INSTANTIATE_TEST_SUITE_P(
    transform_inverse_test_suite,
    Test_Transform_Inverse,
    testing::Values(Transform2D(), 
                    Transform2D(Vector2D(1., 2.), PI),
                    Transform2D(1.95),
                    Transform2D(Vector2D(1., 2.))

));


INSTANTIATE_TEST_SUITE_P(
    transform_composition_test_suite,
    Test_Transform_Composition,
    testing::Values(std::make_tuple(Transform2D(Vector2D(1., 2.)), Transform2D(), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(), Transform2D(Vector2D(1., 2.)), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(), Transform2D(Vector2D(1., 2.)), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(2., 0.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(0., 0.), deg2rad(90.)), Transform2D(Vector2D(1., 0.), deg2rad(90.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(0., 1.)), Transform2D(Vector2D(1., 1.))),
                    std::make_tuple(Transform2D(deg2rad(90.)), Transform2D(deg2rad(45)), Transform2D(deg2rad(135.))),
                    std::make_tuple(Transform2D(deg2rad(90.)), Transform2D(deg2rad(180)), Transform2D(deg2rad(270.)))

));

INSTANTIATE_TEST_SUITE_P(
    transform_composition_assignment_test_suite,
    Test_Transform_Composition_Assignment,
    testing::Values(std::make_tuple(Transform2D(Vector2D(1., 2.)), Transform2D(), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(), Transform2D(Vector2D(1., 2.)), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(), Transform2D(Vector2D(1., 2.)), Transform2D(Vector2D(1., 2.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(2., 0.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(0., 0.), deg2rad(90.)), Transform2D(Vector2D(1., 0.), deg2rad(90.))),
                    std::make_tuple(Transform2D(Vector2D(1., 0.)), Transform2D(Vector2D(0., 1.)), Transform2D(Vector2D(1., 1.))),
                    std::make_tuple(Transform2D(deg2rad(90.)), Transform2D(deg2rad(45)), Transform2D(deg2rad(135.))),
                    std::make_tuple(Transform2D(deg2rad(90.)), Transform2D(deg2rad(180)), Transform2D(deg2rad(270.)))

));

INSTANTIATE_TEST_SUITE_P(
    transform_twist_equals_test_suite,
    Test_Twist_Equals,
    testing::Values(std::make_tuple(Twist2D(1., 2., 3.), Twist2D(1., 2., 3.), true),
                    std::make_tuple(Twist2D(), Twist2D(0., 0., 3.14), false)
));

INSTANTIATE_TEST_SUITE_P(
    transform_twist_test_suite,
    Test_Twist_Transformation,
    testing::Values(std::make_tuple(Transform2D(), Twist2D(1., 2., 3.), Twist2D(1., 2., 3.)),
                    std::make_tuple(Transform2D(Vector2D(0., 1.), deg2rad(90.)), Twist2D(1., 1., 1.), Twist2D(0., 1., 1.)),
                    std::make_tuple(Transform2D(Vector2D(0., 1.), deg2rad(-90.)), Twist2D(1., 1., 1.), Twist2D(2., -1., 1))
));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}