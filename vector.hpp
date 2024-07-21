#include <iostream>
#include <vector>

using namespace std;

class Vector{

    public:

        Vector(int size);

        void set_vector(const vector<float> entries);

        void set_size(int size);

        void init(int size);

        vector<float> get_vector();

        Vector operator+(const Vector &other);

        Vector operator-(const Vector &other);

        float operator*(const Vector &other); // dot product

        Vector operator=(const float &single); // need to figure out how I could combine this operator with the one below it.
        // the issue is that somehow the vector and the Vector assignment are being confused because C++ treats {0.0} same as assigning a Vector.

        Vector operator=(const vector<float> &other); //storing new vector

        Vector operator=(const Vector &other); //storing Vector

        friend std::ostream& operator<<(std::ostream& out, const Vector &vector);

    public:

        Vector() : 
                    entries_({0.0}),
                    size_(0){};

        vector<float> entries_;
        int size_;

};