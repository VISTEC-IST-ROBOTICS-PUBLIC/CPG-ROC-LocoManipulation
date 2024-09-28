//
// Created by CVH on 30/03/2019
// Documentation available at https://github.com/CVH95/Jacksonville.git
//

#ifndef MATRIX_H
#define MATRIX_H

#include <math.h>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

template <typename T> class Matrix
{
    public:

        // Constructor
        Matrix(int rows, int columns, const T& init);
        
        // Copy constructor
        Matrix(const Matrix<T>& A);

        // Destructor
        virtual ~Matrix();

        // PUBLIC METHODS
        
        int getRows() const; // Get Matrix Size.
        int getColumns() const; // Get Matrix Size.
        void printMatrix(); // Print complete matrix.
        T& getElement(int row, int col); // Get individual element.
        void updateElement(int row, int col, T value); // Update individual element.
        
        
        // OPERATORS

        T& operator()(const int& row, const int& col);
        const T& operator()(const unsigned& row, const unsigned& col) const;
        Matrix<T>& operator=(const Matrix<T>& A);
        Matrix<T> operator+(const Matrix<T>& A);
        Matrix<T> operator-(const Matrix<T>& A);
        Matrix<T> operator*(const Matrix<T>& A);

        // ALGEBRA
        
        // Multiplications
        vector<T> matrixByColumnVector(vector<T> vec); // Matrix by Column Vector multiplication (result = vector).
        vector<T> rowVectorByMatrix(vector<T> vec); // Row Vector by Matrix Multiplication (result = vector).
        Matrix<T> columnVectorByRowVector(vector<T> a, vector<T> b); // Column Vector by Row Vector multiplication (result = Matrix).
        
        // Numerical
        void getCoFactors(vector<vector<T> > array, vector<vector<T> >& temp, int size, int r, int c); // Calculate matrix cofactors for computing the determinant.
        T iterateDet(vector<vector<T> > array, int size); 
        T getDeterminant(int size); // Determinant of a square matrix.
        Matrix<T> scalarTimesMatrix(T scalar);// Scalar-matrix multiplication.
        Matrix<T> scalarPlusMatrix(T scalar); // Scalar-matrix sum.
        Matrix<T> scalarMinusMatrix(T scalar); // Scalar-matrix substraction.
        vector<T> diagonalVector(); // Vector with the diagonal elements of the matrix.       

        // Transformations
        Matrix<T> identity(); // Identity matrix.
        Matrix<T> transpose(); // Transpose of the original matrix.
        Matrix<T> adjoint(); // Adjoint of a square matrix.
        Matrix<T> inverse(); //Matrix<T> inverse(); // Inverse of a square matrix.
        

    private:

        int ROWS;
        int COLS;
        vector<vector<T> > mat;
};

#include "Matrix.cpp"

#endif //MATRIX_H