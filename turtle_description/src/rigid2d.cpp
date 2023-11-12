#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

double rotation_angle(const Eigen::MatrixXd& matrix) {

    if (matrix.cols() != 3 || matrix.rows() != 3) {
       throw std::runtime_error("The Eigen matrix must represent a 2D transformation."); 
    }

    double cos_theta = matrix(0, 0);
    double sin_theta = matrix(1, 0);
    return std::atan2(sin_theta, cos_theta);
}

Vector2D Vector2D::normalize() const {
    double magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    if (almost_equal(magnitude, 0.0)) {
        throw std::runtime_error("Cannot normalize a vector whose magnitude is 0.");
    }
    Vector2D normalized_vector;
    normalized_vector.x = x / magnitude;
    normalized_vector.y = y / magnitude;

    return normalized_vector;
}

bool Vector2D::operator == (Vector2D &rhs) {
    return almost_equal(x, rhs.x) && almost_equal(y, rhs.y);
}

float Vector2D::magnitude() const {
    return sqrt((x * x) + (y * y));
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
    os << "[" << v.x << ", " << v.y << "]";
    return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v) {

    std::string s;
    std::getline(is, s);

    if (s.size() < 4) {
        throw std::runtime_error("The input stream is too short to contain a vector.");
    }

    bool has_opening_bracket = s[0] == '[';
    bool has_closing_bracket = s[s.size() - 1] == ']';

    if (has_opening_bracket != has_closing_bracket) {
        throw std::runtime_error("The stream does not conform to the input pattern for a vector.");
    }
    else if (has_opening_bracket && has_closing_bracket) {
        s = s.substr(1, s.size() - 2);
    }

    std::string delimiter = ",";
    size_t pos = 0;
    std::string x, y;
    if ((pos = s.find(delimiter)) != std::string::npos) {
        x = s.substr(0, pos);
        y = s.substr(pos + 1, s.size());
    }
    else { 
        throw std::runtime_error("The stream does not conform to the input pattern for a vector.");
    }

    // Remove whitespace
    x.erase(std::remove(x.begin(), x.end(), ' '), x.end());
    y.erase(std::remove(y.begin(), y.end(), ' '), y.end());

    v.x = stod(x);
    v.y = stod(y);

    return is;
}

Vector2D::Vector2D(double x, double y) {
    this->x = x;
    this->y = y;
}

Vector2D::Vector2D() {
    this->x = 0.0;
    this->y = 0.0;
}

Transform2D::Transform2D() {
    matrix = Eigen::MatrixXd::Identity(3, 3);

}

bool Transform2D::operator == (const Transform2D& rhs) {

    auto rhs_translation = rhs.translation();
    return (translation() == rhs_translation) && almost_equal(rotation(), rhs.rotation());
}

void Transform2D::set_rotation(Eigen::MatrixXd& matrix, double radians) {
    double cos_theta = std::cos(radians);
    double sin_theta = std::sin(radians);
    matrix(0, 0) = cos_theta;
    matrix(0, 1) = -sin_theta;
    matrix(1, 0) = sin_theta;
    matrix(1, 1) = cos_theta; 
}

void Transform2D::set_translation(Eigen::MatrixXd& matrix, Vector2D trans) {
    matrix(0, 2) = trans.x;
    matrix(1, 2) = trans.y;
}

void Transform2D::set_matrix(Eigen::MatrixXd matrix2) {
    this->matrix = matrix2;
}

Transform2D::Transform2D(Vector2D trans) {
    matrix = Eigen::MatrixXd::Identity(3, 3);
    set_translation(matrix, trans);
}

Transform2D::Transform2D(double radians) {
    matrix = Eigen::MatrixXd::Identity(3, 3);
    set_rotation(matrix, radians);
}


Transform2D::Transform2D(Vector2D trans, double radians) {
    matrix = Eigen::MatrixXd::Identity(3, 3);
    double cos_theta = std::cos(radians);
    double sin_theta = std::sin(radians);
    matrix(0, 0) = cos_theta;
    matrix(0, 1) = -sin_theta;
    matrix(1, 0) = sin_theta;
    matrix(1, 1) = cos_theta;
    
    matrix(0, 2) = trans.x;
    matrix(1, 2) = trans.y;
}

Vector2D Transform2D::operator()(Vector2D v) const {

    Eigen::MatrixXd homogenous_column_vector = Eigen::MatrixXd(3, 1);
    homogenous_column_vector(0, 0) = v.x;
    homogenous_column_vector(1, 0) = v.y;
    homogenous_column_vector(2, 0) = 1.0;

    Eigen::MatrixXd v_transformed = matrix * homogenous_column_vector;
    double x = v_transformed(0, 0);
    double y = v_transformed(1, 0);
    return Vector2D(x, y);
}

Transform2D Transform2D::inv() const {

    double theta = rotation_angle(this->matrix);
    /*
    auto transform = translation();
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    auto inverse_matrix = Eigen::MatrixXd::Identity(3, 3);

    inverse_matrix(0, 0) = cos_theta; 
    inverse_matrix(1, 0) = -sin_theta;
    
    inverse_matrix(0, 1) = sin_theta;
    inverse_matrix(1, 1) = cos_theta;

    inverse_matrix(0, 2) = (-transform.x * cos_theta) - (transform.y * sin_theta);
    inverse_matrix(1, 2) = (-transform.y * cos_theta) + (transform.x * sin_theta);
    */

    auto inverse_matrix = matrix.inverse();
    Transform2D inverse_transform = Transform2D();
    inverse_transform.set_matrix(inverse_matrix);
    return inverse_transform;
}

Eigen::MatrixXd Transform2D::get_matrix() const {
    return matrix;
}


Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
    auto resulting_matrix = matrix * rhs.get_matrix();
    set_matrix(resulting_matrix);
    return *this;
}

void Transform2D::set_translation(Vector2D vector) {
    set_translation(matrix, vector);
}

void Transform2D::set_rotation(double radians) {
    set_rotation(matrix, radians);
}



Vector2D Transform2D::translation() const {
    double x = matrix(0, 2);
    double y = matrix(1, 2);
    return Vector2D(x, y);
}

double Transform2D::rotation() const {
    return rotation_angle(matrix);
}

std::ostream & operator<<(std::ostream & os, const Transform2D & transform) {

    auto translation = transform.translation();
    double rotation = transform.rotation();
    os << "Translation: " << "[" << translation.x << ", " << translation.y << "]" << std::endl;
    os << "Rotation (degrees): " << rad2deg(rotation) << std::endl;
    return os;
}

Transform2D operator*(Transform2D& lhs, const Transform2D & rhs) {
    Eigen::MatrixXd resulting_matrix = lhs.get_matrix() * rhs.get_matrix();
    Transform2D result;
    result.set_matrix(resulting_matrix);
    return result;
}

std::istream & operator>>(std::istream & is, Transform2D & transform) {

    std::string s;
    std::getline(is, s);

    if (s.size() < 9) {
        throw std::runtime_error("The input stream is too short to contain a Transform2D.");
    }

    bool has_opening_bracket = s[0] == '[';
    bool has_closing_bracket = s[s.size() - 1] == ']';

    if (has_opening_bracket != has_closing_bracket) {
        throw std::runtime_error("The stream does not conform to the input pattern for a Transform2D.");
    }
    else if (has_opening_bracket && has_closing_bracket) {
        s = s.substr(1, s.size() - 2);
    }

    std::string delimiter = ",";
    size_t pos = 0;
    std::string x, y, angle, rest;
    if ((pos = s.find(delimiter)) != std::string::npos) {
        x = s.substr(0, pos);
        rest = s.substr(pos + 1, s.size());
    }
    else { 
        throw std::runtime_error("The stream does not conform to the input pattern for a Transform2D.");
    }

    if ((pos = rest.find(delimiter)) != std::string::npos) {
        y = rest.substr(0, pos);
        angle = rest.substr(pos + 1, rest.size());
    }
    else { 
        throw std::runtime_error("The stream does not conform to the input pattern for a Transform2D.");
    }

    // Remove whitespace
    x.erase(std::remove(x.begin(), x.end(), ' '), x.end());
    y.erase(std::remove(y.begin(), y.end(), ' '), y.end());
    angle.erase(std::remove(angle.begin(), angle.end(), ' '), angle.end());

    transform.set_translation(Vector2D(stod(x), stod(y)));
    transform.set_rotation(stod(angle));

    return is;
}

Twist2D::Twist2D() {
    theta = 0.0;
    x = 0.0;
    y = 0.0;
}

bool Twist2D::operator == (Twist2D &rhs) {
    return almost_equal(theta, rhs.theta) && almost_equal(x, rhs.x) && almost_equal(y, rhs.y);
}

Twist2D::Twist2D(float x, float y, float theta) {
    this->x = x;
    this->y = y;
    this->theta = theta;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & twist) {
    
    os << "[" << twist.theta << ", " << twist.x << ", " << twist.y << "]";
    return os;
}

std::istream & operator>>(std::istream & is, Twist2D & twist) {

    std::string s;
    std::getline(is, s);

    if (s.size() < 9) {
        throw std::runtime_error("The input stream is too short to contain a Twist2D.");
    }

    bool has_opening_bracket = s[0] == '[';
    bool has_closing_bracket = s[s.size() - 1] == ']';

    if (has_opening_bracket != has_closing_bracket) {
        throw std::runtime_error("The stream does not conform to the input pattern for a Twist2D.");
    }
    else if (has_opening_bracket && has_closing_bracket) {
        s = s.substr(1, s.size() - 2);
    }

    std::string delimiter = ",";
    size_t pos = 0;
    std::string x, y, theta, rest;
    if ((pos = s.find(delimiter)) != std::string::npos) {
        theta = s.substr(0, pos);
        rest = s.substr(pos + 1, s.size());
    }
    else { 
        throw std::runtime_error("The stream does not conform to the input pattern for a Twist2D.");
    }

    if ((pos = rest.find(delimiter)) != std::string::npos) {
        x = rest.substr(0, pos);
        y = rest.substr(pos + 1, rest.size());
    }
    else { 
        throw std::runtime_error("The stream does not conform to the input pattern for a Twist2D.");
    }

    // Remove whitespace
    x.erase(std::remove(x.begin(), x.end(), ' '), x.end());
    y.erase(std::remove(y.begin(), y.end(), ' '), y.end());
    theta.erase(std::remove(theta.begin(), theta.end(), ' '), theta.end());

    twist.x = stod(x), 
    twist.y = stod(y);
    twist.theta = stod(theta);

    return is;
}

Eigen::MatrixXd Transform2D::adjoint() const {

    Eigen::MatrixXd adjoint_matrix = Eigen::MatrixXd::Identity(3, 3);
    Vector2D position = translation();
    double theta = rotation();
    adjoint_matrix(2, 0) = -position.x;
    adjoint_matrix(1, 0) = position.y;
    
    adjoint_matrix(1, 1) = cos(theta);
    adjoint_matrix(1, 2) = -sin(theta);

    adjoint_matrix(2, 1) = sin(theta);
    adjoint_matrix(2, 2) = cos(theta);

    return adjoint_matrix;
}


Twist2D Transform2D::operator()(const Twist2D& twist) {

    Eigen::MatrixXd twist_vector = Eigen::MatrixXd(3, 1);
    twist_vector(0, 0) = twist.theta;
    twist_vector(1, 0) = twist.x;
    twist_vector(2, 0) = twist.y;

    Eigen::MatrixXd transformed_matrix = adjoint() * twist_vector;

    Twist2D transformed_twist;
    transformed_twist.theta = transformed_matrix(0, 0);
    transformed_twist.x = transformed_matrix(1, 0);
    transformed_twist.y = transformed_matrix(2, 0);

    return transformed_twist;
}

}

/*
int main() {

    using namespace turtlelib;

    Vector2D v;
    v.x = 1;
    v.y = 2;
    std::cout << v << std::endl;

    //std::cin >> v;
    //std::cout << v << std::endl;

    //Transform2D transform = Transform2D();
    //std::cin >> transform;
    //std::cout << transform << std::endl;
    //std::cout << transform.matrix << std::endl;

    Twist2D twist = Twist2D();
    std::cin >> twist;
    std::cout << twist<< std::endl;

    return 0;
}
*/
