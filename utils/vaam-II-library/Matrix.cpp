//
// Created by CVH on 30/03/2019
//

#ifndef MATRIX_CPP
#define MATRIX_CPP

#include "Matrix.h"


// CONSTRUCTOR
template<typename T>
Matrix<T>::Matrix(int rows, int columns, const T& init)
//Matrix::Matrix(int rows, int columns, const float init)
{
    ROWS = rows;
    COLS = columns;
    mat.resize(ROWS);
    for(int i=0; i<mat.size(); i++)
    {
        mat[i].resize(COLS, init);
    }
}


// COPY CONSTRUCTOR                                                                                                                                                           
template<typename T>
Matrix<T>::Matrix(const Matrix<T>& A) 
{
    mat = A.mat;
    ROWS = A.getRows();
    COLS = A.getColumns();
}


// DESTRUCTOR
template<typename T>
Matrix<T>::~Matrix()
{
    //cout << "Matrix succesfully destroyed." << endl;
}


//----------------------------------------------------------------------------------------------------


// PUBLIC METHODS


// Get number of rows
template<typename T>
int Matrix<T>::getRows() const
//int Matrix::getRows()
{
    return this->ROWS;
}
        
// Get number of columns
template<typename T>
int Matrix<T>::getColumns() const
//int Matrix::getColumns()
{
    return this->COLS;
}


// Print whole matrix
template<typename T>
void Matrix<T>::printMatrix()
//void Matrix::printMatrix()
{   
    int r = getRows();
    int c = getColumns();
    int limit = c-1;
    cout << "Matrix[" << r << "][" << c << "]:" << endl << endl; 
    for(int i=0; i<r; i++)
    {   
        for(int j=0; j<c; j++)
        {
            if(j == limit)
            {
                cout << this->mat[i][j];
            }
            else
            {
                cout << this->mat[i][j] << ",";
            }
        }
        cout << endl;
    }
    cout << endl;
}


// Get individual element
template<typename T>
T& Matrix<T>::getElement(int row, int col)
{
    return this->mat[row][col];
}


// Update individual value 
template<typename T>
void Matrix<T>::updateElement(int row, int col, T value)
{
    mat[row][col] = value;
}


//----------------------------------------------------------------------------------------------------


// OPERATORS


// Access individual elements
template<typename T>
T& Matrix<T>::operator()(const int& row, const int& col)
{
    return this->mat[row][col];
}


// Access individual elements const version
template<typename T>
const T& Matrix<T>::operator()(const unsigned& row, const unsigned& col) const
{
    return this->mat[row][col];
}


// Assignment Operator                                                                                                                                                        
template<typename T>
Matrix<T>& Matrix<T>::operator=(const Matrix<T>& A) 
{
    if (&A == this)
        return *this;

    int new_rows = A.getRows();
    int new_cols = A.getColumns();

    mat.resize(new_rows);
    for (int i=0; i<mat.size(); i++)
    {
        mat[i].resize(new_cols);
    }

    for (int i=0; i<new_rows; i++)
    {
        for (int j=0; j<new_cols; j++)
        {
            mat[i][j] = A(i, j);
        }
    }

    ROWS = new_rows;
    COLS = new_cols;

    return *this;
}


// Sum of two matrices                                                                                                                                                   
template<typename T>
Matrix<T> Matrix<T>::operator+(const Matrix<T>& A) 
{
    Matrix result(ROWS, COLS, 0);

    for (int i=0; i<ROWS; i++) 
    {
        for (int j=0; j<COLS; j++)
        {
            //T val = A.getElement(i, j);
            T newVal = this->mat[i][j] + A(i, j);
            result.updateElement(i, j, newVal);
        }
    }
    return result;
}


// Subtraction of this matrix and another                                                                                                                                     
template<typename T>
Matrix<T> Matrix<T>::operator-(const Matrix<T>& A)
{
    Matrix result(ROWS, COLS, 0);

    for (int i=0; i<ROWS; i++) 
    {
        for (int j=0; j<COLS; j++)
        {
            //T val = A.getElement(i, j);
            T newVal = this->mat[i][j] - A(i, j);
            result.updateElement(i, j, newVal);
        }
    }
    return result;
}


// Multiplication of this matrix and another                                                                                                                                     
template<typename T>
Matrix<T> Matrix<T>::operator*(const Matrix<T>& A)
{
    Matrix result(ROWS, COLS, 0);

    for (int i=0; i<ROWS; i++) 
    {
        for (int j=0; j<COLS; j++)
        {
            for(int k=0; k<ROWS; k++)
            {
                result(i, j) += this->mat[i][k] * A(k, j);
            }
        }
    }
    return result;
}


//----------------------------------------------------------------------------------------------------


// ALGEBRA


// Multiplication of a matrix A (m x n) by a vector v (n x 1) 
template<typename T>
vector<T> Matrix<T>::matrixByColumnVector(vector<T> vec)
{
    // Declare result vector
    vector<T> result;

    int vecRows = vec.size();
    int c = this->COLS;
    int r = this->ROWS;
    result.resize(r, 0);
    if (vecRows == c)
    {
        for(int i=0; i<r; i++)
        {
            for(int j=0; j<c; j++)
            {
                result[i] += this->mat[i][j] * vec[j];
            }
        }
        return result;
    }
    else
    {
        cout << "ERROR: Invalid vector size!" << endl;
        result.resize(r, 0);
        return result;
    }
}


// Multiplication of a row vector (1 x m) by a matrix (m x n)
template<typename T>
vector<T> Matrix<T>::rowVectorByMatrix(vector<T> vec)
{
    // Declare result vector
    vector<T> result;

    int vecCols = vec.size();
    int c = this->COLS;
    int r = this->ROWS;
    result.resize(c, 0);
    if (vecCols == r)
    {
        for(int i=0; i<c; i++)
        {
            for(int j=0; j<r; j++)
            {
                result[i] += this->mat[j][i] * vec[j];
            }
        }
        return result;
    }
    else
    {
        cout << "ERROR: Invalid vector size!" << endl;
        result.resize(c, 0);
        return result;
    }
}


// Multiplication of Column vector (m x 1) by a row vector (1 x m) 
template<typename T>
Matrix<T> Matrix<T>::columnVectorByRowVector(vector<T> a, vector<T> b)
{   
    Matrix<T> result(a.size(), b.size(), 0);
    if(a.size() == b.size())
    {
        // Initialize Matrix
        int r = a.size();
        int c = b.size();

        // Calculate elements:
        for(int i=0; i<r; i++)
        {
            for(int j=0; j<c; j++)
            {
                result.updateElement(i, j, a[i]*b[j]);
            }
        } 
        return result;
    }
    else
    {
        // Initialize Matrix
        int r = a.size();
        int c = b.size();
        cout << "ERROR: Invalid vector size!" << endl;
        return result;
    }
}


// Calculate matrix co-factors
template<typename T>
void Matrix<T>::getCoFactors(vector<vector<T> > array, vector<vector<T> >& temp, int size, int r, int c)
{
    int current_row = 0;
    int current_col = 0;  

    for(int i=0; i<size; i++)
    {
        for(int j=0; j<size; j++)
        {
            if(i != r && j != c)
            {
                temp[current_row][current_col] = array[i][j];
                //cout << "temp (" << i << "," << j << ") = " << temp[current_row][current_col] << endl;
                current_col++;
                if(current_col == size - 1)
                {
                    current_col = 0;
                    current_row++;
                }
            }
        }
    }
}


// Performs the iteration over the submatrices in the main matrix to calculate the determinant
template<typename T>
T Matrix<T>::iterateDet(vector<vector<T> > array, int size)
{
    T determinant = 0;
    if(size == 2)
    {
        return array[0][0]*array[1][1] - array[1][0]*array[0][1];
    }

    // Initialize 
    vector<vector<T> > temp;
    temp.resize(size);
    for(int i=0; i<size; i++)
    {
        temp[i].resize(size, 0);
    }
    T sign = 1;

    for(int h=0; h<size; h++)
    {
        this->getCoFactors(array, temp, size, 0, h);
        determinant += sign * array[0][h] * this->iterateDet(temp, size-1);
        //cout << "Partial det = " << determinant << endl;

        sign = -sign;
    }
    return determinant;
}


// Calculate the determinant of the matrix
template<typename T>
T Matrix<T>::getDeterminant(int size)
{
    if(ROWS != COLS)
    {
        cout << "ERROR: Matrix is not square!" << endl;
        return 0;
    }
    
    vector<vector<T> > array;
    array.resize(size);
    for(int i=0; i<size; i++)
    {
        array[i].resize(size, 0);
    }
    
    // Copy matrix to 2D array
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            array[i][j] = this->mat[i][j];
            //cout << "array (" << i << "," << j << ") = " << array[i][j] << endl;
        }
    }

    T det = this->iterateDet(array, size);
    return det;
}


// Multiplying a Matrix by a scalar
template<typename T>
Matrix<T> Matrix<T>::scalarTimesMatrix(T scalar)
{
    Matrix result(ROWS, COLS, 0);
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            T newVal = scalar * this->mat[i][j];
            result.updateElement(i, j, newVal);
        }
    }
    return result;
}


// Adding a scalar to a matrix
template<typename T>
Matrix<T> Matrix<T>::scalarPlusMatrix(T scalar)
{
    Matrix result(ROWS, COLS, 0);
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            T newVal = scalar + this->mat[i][j];
            result.updateElement(i, j, newVal);
        }
    }
    return result;
}


// Substracting a scalar to a matrix
template<typename T>
Matrix<T> Matrix<T>::scalarMinusMatrix(T scalar)
{
    Matrix result(ROWS, COLS, 0);
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            T newVal = this->mat[i][j] - scalar;
            result.updateElement(i, j, newVal);
        }
    }
    return result;
}


// Obtaining a vector with the diagonal elements
template<typename T>
vector<T> Matrix<T>::diagonalVector()
{
    vector<T> d;
    if(ROWS != COLS)
    {
        cout << "ERROR: Matrix is not square!" << endl;
        return d;
    }
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            if(i == j)
            {
                T val = this->getElement(i, j);
                d.push_back(val);
            }
        }
    }
    return d;
}


// Identity matrix
template<typename T>
Matrix<T> Matrix<T>::identity()
{
    Matrix I(ROWS, COLS, 0);
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            I.updateElement(i, j, 1);
        }
    }
    return I;
}


// Transpose
template<typename T>
Matrix<T> Matrix<T>::transpose()
{
    Matrix t(COLS, ROWS, 0);
    for(int i=0; i<COLS; i++)
    {
        for(int j=0; j<ROWS; j++)
        {
            t.updateElement(i, j, this->mat[j][i]);
        }
    }
    return t;
}


// Adjoint
template<typename T>
Matrix<T> Matrix<T>::adjoint()
{
    // Initialize result matrix
    Matrix adjnt(ROWS, COLS, 0);
    int size = ROWS;
    T sign = 1;

    if(ROWS != COLS)
    {
        cout << "ERROR: Matrix is not square!" << endl;
        return adjnt;
    }

    // Arrays to store cofactors
    vector<vector<T> > array;
    array.resize(size);
    for(int i=0; i<size; i++)
    {
        array[i].resize(size, 0);
    }
    for(int g=0; g<ROWS; g++)
    {
        for(int j=0; j<COLS; j++)
        {
            array[g][j] = this->mat[g][j];
            //cout << "array (" << i << "," << j << ") = " << array[i][j] << endl;
        }
    }
    vector<vector<T> > temp;
    temp.resize(size);
    for(int l=0; l<size; l++)
    {
        temp[l].resize(size, 0);
    }

    // Calculate adjoint matrix
    for(int h=0; h<ROWS; h++)
    {
        for(int k=0; k<COLS; k++)
        {
            // Calculate cofactors
            this->getCoFactors(array, temp, size, h, k);

            // Update sign: Positive if row+col = even; Negative otherwise
            sign = ((h+k)%2==0)? 1: -1;

            T partial_det = sign * this->iterateDet(temp, size-1);
            adjnt.updateElement(k, h, partial_det);
        }
    }
    return adjnt;
}


// Inverse
template<typename T>
Matrix<T> Matrix<T>::inverse()
{
    Matrix inv(ROWS, COLS, 0);
    int size = ROWS;
    
    if(ROWS != COLS)
    {
        cout << "ERROR: Matrix is not square!" << endl;
        return inv;
    }
    
    T det = this->getDeterminant(size);
    
    // Check for singular matrix
    if(det == 0)
    {
        cout << "ERROR: Singular matrix, not possible to compute its determinant" << endl;
        return inv;
    }
    
    // Calculate adjoint
    Matrix adj(ROWS, COLS, 0);
    adj = this->adjoint();

    // Calculate inverse as: inv(A) = adj(A)/det(A)
    for(int i=0; i<ROWS; i++)
    {
        for(int j=0; j<COLS; j++)
        {
            T adj_val = adj.getElement(i, j);
            T newVal = adj_val/det;
            inv.updateElement(i, j, newVal);
        }
    }
    return inv;
}


//----------------------------------------------------------------------------------------------------


#endif //MATRIX_CPP