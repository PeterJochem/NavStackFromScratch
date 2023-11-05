#include "rigid2d.hpp"

namespace turtlelib
{

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


}

int main() {

    using namespace turtlelib;

    Vector2D v;
    v.x = 1;
    v.y = 2;
    std::cout << v << std::endl;

    std::cin >> v;
    std::cout << v << std::endl;


    return 0;
}