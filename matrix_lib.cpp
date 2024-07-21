#include <iostream>

#include "matrix_lib.hpp"
#include <math.h>
#include <cmath>

using namespace std;
// using namespace Math;
// using Matrix;

// namespace Math{

  // wheel encoders 20000RPM

    Matrix m;

    Matrix::Matrix(int rows, int columns){  

      set_size(rows, columns);

      init(rows,columns);

    };

    void Matrix::set_matrix(vector<float> entries){

      for(int entry = 0; entry < rows_*columns_; entry++)
      {
        entries_[entry] = entries[entry];

      }

    }

    void Matrix::set_size(int rows, int columns){
      
      rows_ = rows; 
      columns_ = columns;

      init(rows_,columns_);

    }

    void Matrix::init(int rows, int columns)
    {
      for(int entry = 0; entry < rows*columns; entry++)
      {
        entries_.push_back(0.0);
      }
    }

    vector<float> Matrix::get_matrix()
    {  
      return entries_;
    };

    Matrix Matrix::transpose(){

      Matrix transpose(columns_,rows_);

      vector<float> transposed;
      for(int row = 0; row < columns_*rows_; row++){
        transposed.push_back(0.0);
      }

      for(int row = 0; row < rows_; row++)
      {
        for(int clm = 0; clm < columns_; clm++)
        {
          int entry = clm + row*columns_;
          transposed[entry] = entries_[entry + (clm - row)*(rows_-1)];
        }
      }

      transpose.set_matrix(transposed);

      return transpose;
    }

    Matrix Matrix::inverse(){

      bool invalid = false;

      Matrix result(columns_, rows_);

      if (columns_ == rows_){
        
        int n = rows_; // indicating the matrix is

        Matrix guassian(n, n); // square matrix
        guassian.entries_ = entries_;

        vector<float> inverse;

        for(int row = 0; row < n; row++){
          for(int clm = 0; clm < columns_; clm++){
            if (clm == row){
              inverse.push_back(1);
            }
            else{
              inverse.push_back(0);
            }
          }
        }

        for(int row = 0; row < n; row++)
        {
          // getting the pivot. only gets the diagnonals
          float diag_focus = guassian.entries_[row + n*row];

          // swapping rows if diagnonal is zero.
          if( diag_focus == 0){
            for(int swap_row = row; swap_row < n; swap_row++){
              if(guassian.entries_[row + n*swap_row] != 0){
                diag_focus = guassian.entries_[row + n*swap_row];
                //switching row
                for(int clm = 0; clm < n; clm++){
                  float g_swap = guassian.entries_[clm + n*row];
                  guassian.entries_[clm + n*row] = guassian.entries_[clm + n*swap_row];
                  guassian.entries_[clm + n*swap_row] = g_swap;

                  float i_swap = inverse[clm + n*row];
                  inverse[clm + n*row] = inverse[clm + n*swap_row];
                  inverse[clm + n*swap_row] = i_swap;
                }
              }
              if(swap_row == n-1){

              }
            }
          }

          if (invalid == true){
            break;
          }

          // row = row/diag_focus
          // redusing the focus pivot to 1 eg [3,1,4] becomes [1,1/3,4/3]
          for (int clm = 0; clm < n; clm++)
          {
            guassian.entries_[clm + n*row] = guassian.entries_[clm + n*row]/diag_focus;
            inverse[clm + n*row] = inverse[clm + n*row]/diag_focus;
          }

          // reduce_row = reduce_row - x*row
          // reducing all entries of the column to which we got diag_focus to 0
          for (int reduce_row = 0; reduce_row < n; reduce_row++)
          {
            if (reduce_row != row)
            {
              // diagnonal pivot entry - reducing all entries of the column to 0.
              float first_entry = guassian.entries_[row + n*reduce_row];

              for(int clm = 0; clm < n; clm++)
              {
                guassian.entries_[clm + n*reduce_row] = guassian.entries_[clm + n*reduce_row] - guassian.entries_[clm + n*row]*first_entry;
                inverse[clm + n*reduce_row] = inverse[clm + n*reduce_row] - inverse[clm + n*row]*first_entry;
              }
            }
          }
        }

        if (!invalid){
          result.set_matrix(inverse);
        }
      }

      return result;

    }

    Matrix Matrix::operator+(const Matrix &other){ //why cannot use the namespace Matrix:: here?

      Matrix result(rows_,columns_);
      int n = rows_ * columns_;

      vector<float> result_v;

      for(int i = 0; i < n; i++){
        result_v.push_back(entries_[i] + other.entries_[i]);
      }

      result.set_matrix(result_v);

      return result;

    };

    Matrix Matrix::operator - (const Matrix &other){

      Matrix result(rows_,columns_);

      int n = rows_ * columns_;
      vector<float> result_v;

      for(int i = 0; i < n; i++){
        result_v.push_back(entries_[i] - other.entries_[i]);
      }

      result.set_matrix(result_v);

      return result;
    }

    Matrix Matrix::operator * (const Matrix &other){

      Matrix result(rows_,other.columns_);

      if (columns_ == other.rows_){
        
        vector<float> result_v;
        int n = rows_*other.columns_;
        int row = 0;
        int col = 0;

        while(row < rows_)
        {
          while(col < other.columns_)
          {
            float entry_result = 0.0;
            for (int j = 0; j < columns_; j++)
            {
              entry_result = entry_result + entries_[row * columns_ + j]*other.entries_[col + other.columns_*j];            
            }
            result_v.push_back(entry_result);
            col++;
          }
          col = 0;
          row++;
        }
        result.set_matrix(result_v);
      }

      return result;
    }
    

    Vector Matrix::operator*(const Vector &other)
    {

      Vector result(rows_);

      if (columns_ == other.size_){

        vector<float> result_v;

        int row = 0;
        int col = 0;

        while(row < rows_)
        {
          float entry_result = 0.0;
          for (int j = 0; j < columns_; j++)
          {
            entry_result = entry_result + entries_[row * columns_ + j]*other.entries_[col + j];            
          }
          result_v.push_back(entry_result);
          row++;
        }

        result.set_vector(result_v);
      }

      return result;

    }

    Matrix Matrix::operator = (const vector<float> entries)
    {
      set_matrix(entries);

      return *this;
    }

    Matrix Matrix::operator = (const Matrix matrix)
    {


      set_size(matrix.rows_, matrix.columns_);
      set_matrix(matrix.entries_);

      return *this;
    }

    std::ostream& operator<<(std::ostream& out, const Matrix &matrix)
    {
      
      for(int i = 0; i < matrix.rows_; i++)
      {
        for(int j = 0; j < matrix.columns_; j++)
        {
          if (j == matrix.columns_ - 1){
            out << matrix.entries_[j + i * matrix.columns_] << '\n'; // printing last column at row i
          }
          else{
            out << matrix.entries_[j + i * matrix.columns_] << ' '; // printing column j at row i
          }
        }
      }

      // TODO How about convert the abobe int oa for loop? Have a look at concatination with chars and strings.

      return out;
    }

// }