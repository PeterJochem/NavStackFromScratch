#include <iostream>
#include "turtlelib/rigid2d.hpp"

int main() {

    using namespace turtlelib;

    Transform2D t_a_b = Transform2D(Vector2D(0., 1.), deg2rad(90));
    Transform2D t_b_c = Transform2D(Vector2D(1., 0.), deg2rad(90));

    Transform2D t_b_a = t_a_b.inv();
    Transform2D t_c_b = t_b_c.inv();
    Transform2D t_a_c = t_a_b * t_b_c;
    Transform2D t_c_a = t_a_c.inv(); 

    // Print the original two transforms.
    std::cout << "T_a_b\n" << t_a_b << std::endl;
    std::cout << "T_b_c\n" << t_b_c << std::endl;

    // Print the computed transforms.
    std::cout << "T_b_a\n" << t_b_a << std::endl;
    std::cout << "T_c_b\n" << t_c_b << std::endl;

    std::cout << "T_a_c\n" << t_a_c << std::endl;
    std::cout << "T_c_a\n" << t_c_a << std::endl;

    // Enter vector v_b
    Vector2D vector_v_b = Vector2D(1., 1.);
    Vector2D v_b_hat = vector_v_b.normalize();
    Vector2D v_a = t_a_b(vector_v_b);
    Vector2D v_c = t_c_b(vector_v_b);

    std::cout << "v_b_hat: " << v_b_hat << std::endl;
    std::cout << "v_a: " << v_a << std::endl;
    std::cout << "v_b: " << vector_v_b << std::endl;
    std::cout << "v_c: " << v_c << std::endl;


    // Enter twist v_b
    Twist2D twist_v_b = Twist2D(1., 1., 1.);
    Twist2D twist_v_a = t_a_b(twist_v_b); 
    Twist2D twist_v_c = t_c_b(twist_v_b);

    std::cout << "\n\nv_a: " << twist_v_a << std::endl;
    std::cout << "v_b: " << twist_v_b << std::endl;
    std::cout << "v_c: " << twist_v_c << std::endl;

    return 0;
}
