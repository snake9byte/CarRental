#include <iostream>
#include <vector>
#include "vector.hpp"

using namespace std;

// namespace Math{

    class Matrix{

        public:

            Matrix(int rows, int columns);

            void set_size(int rows, int columns);

            void set_matrix(vector<float> entries);

            void init(int rows, int columns);

            vector<float> get_matrix();

            Matrix operator+(const Matrix &other);

            Matrix operator-(const Matrix &other);

            Matrix operator*(const Matrix &other);

            Vector operator*(const Vector &ohter); // matrix * vector

            Matrix inverse(const Matrix &other);

            Matrix transpose(); //keep in mind that traspose DOES NOT update the matrix. If you would like to update it store it on a new matrix.

            Matrix inverse();

            Matrix operator = (const vector<float> entries); // storing the entries of a given matrix as a vector format.

            Matrix operator = (const Matrix matrix); //storing the matrix

            friend std::ostream& operator<<(std::ostream& out, const Matrix &matrix);

        public:

            Matrix() : 
                      rows_(0)
                    , columns_(0)
                    , entries_({0.0}) {  }
            //{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
            int             rows_;
            int             columns_;
            vector<float>   entries_;

    };
        
// };

