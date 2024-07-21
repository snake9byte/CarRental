#include <iostream>

#include "vector.hpp"
#include <math.h>
// #include <cmath>

using namespace std;
// using namespace Math;
// using Matrix;

// namespace Math{

    Vector::Vector(int size){  

      set_size(size);
      init(size);

    };

    void Vector::set_vector(const vector<float> entries)
    {
      for(int entry = 0; entry < size_; entry++)
      {
        entries_[entry] = entries[entry];
      }
    }

    void Vector::set_size(int size)
    {
      size_ = size;
      init(size);

    }

    void Vector::init(int size)
    {
      for(int i = 0; i < size; i++)
      {
        entries_.push_back(0.0);
      }
    }

    vector<float> Vector::get_vector()
    {
      return entries_;
    };

    Vector Vector::operator+(const Vector &other){ //why cannot use the namespace Matrix:: here?

      Vector result(size_);
      vector<float> result_v;

      for(int i = 0; i < size_; i++)
      {
        result_v.push_back(entries_[i] + other.entries_[i]);
      }

      result.set_vector(result_v);

      return result;

    };

    Vector Vector::operator - (const Vector &other){

      Vector result(size_);

      vector<float> result_v;

      for(int i = 0; i < size_; i++)
      {
        result_v.push_back(entries_[i] - other.entries_[i]);
      }

      result.set_vector(result_v);

      return result;
    }

    float Vector::operator * (const Vector &other){

      Vector result(size_);
      float dot_product = 0;

      //vectors havethe same size
      if (other.size_ == size_)
      {
        int dimention = result.size_;
        float dot_product = 0;

        std::cout << dimention;

        for(int entry = 0; entry < dimention; entry++){

          dot_product += entries_[entry]*other.entries_[entry];

        }
      }

      return dot_product;
    }

    Vector Vector::operator = (const vector<float> &entries)
    {
      set_vector(entries);

      return *this;
    }

    Vector Vector::operator = (const float &single)
    {
      entries_ = {single};

      return *this;
    }

    Vector Vector::operator=(const Vector &vector)
    {
      set_size(vector.size_);
      set_vector(vector.entries_);

      return *this;
    } 

    std::ostream& operator<<(std::ostream& out, const Vector &vector)
    {
      for(int i = 0; i < vector.size_; i++)
      {
        if (i == vector.size_ - 1){
          out << vector.entries_[i] << '\n'; // printing last column at row i
        }
        else{
          out << vector.entries_[i] << ' '; // printing column j at row i
        }
      }
      // TODO How about convert the abobe int oa for loop? Have a look at concatination with chars and strings.

      return out;
    }