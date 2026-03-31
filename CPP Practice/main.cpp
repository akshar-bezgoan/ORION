#include <iostream>
#include <cmath>

using namespace std;

int main() {
    const double pi = 3.14;
    double radius;
    cout << "Enter the radius of a circle: ";
    cin >> radius;
    double area = pi * pow(radius, 2);
    cout << area;
    
    return 0;
}