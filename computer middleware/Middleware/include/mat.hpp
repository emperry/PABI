#include <string>
#include <math.h>
#include <iostream>
#include <iomanip> 
#include <functional>
#include <regex>
#include <forward_list>

#ifndef MAT_H
#define MAT_H

// Matrix data type and matrix transformation string parsing

// Transformation matrix
class TransMatrix {
    public:
        // Initialized as an identity matrix
        TransMatrix();
        // 4x4 matrix, this should be a row major
        // array with 16 values
        TransMatrix(double *mat);
        TransMatrix(const TransMatrix &mat);
        TransMatrix &operator=(const TransMatrix &mat);
        // This matrix is T1, arg is T2, output is T3
        // T3 = T1 * T2
        TransMatrix mult(TransMatrix &mat);
        // Multiply this matrix by a vector [x, y, z, 1]^T
        // Takes a double array with 4 values
        void multVec(double *v);
        // Gets the current translation of this matrix
        // returns a pointer to an array [x, y, z, 1]
        double *getTranslate();
        // Inverts the transformation matrix
        void invert();
        void print();
        ~TransMatrix();
    protected:
        double innerDet(double *m);
        double *m1;
        // true if allocated with memalign
        bool aligned = false;
};

// Transformation matrix with a variable substitute
// Can also be instantiated without a substitute
// to be used in a situation where a wrapper would be favorable
class SymTransMatrix {
    public:
        SymTransMatrix(std::string sym, 
            // function to generate the matrix
            // given the arguments
            std::function<TransMatrix(double*)> gen):
            sym(sym), mat(NULL), gen(gen) {};
        SymTransMatrix(TransMatrix mat) {
            wrapper = true;
            sym = "";
            this->mat = new TransMatrix(mat);
        }
        // Substitutes arguments into the underlying matrix
        // and returns the result. Arguments are substituted in order.
        // The args array should be at least the size of the
        // arguments read for the operation.
        TransMatrix substitute(double *args);
        std::string getSymName() {
            return sym;
        }
        ~SymTransMatrix() {
            if(mat != NULL) {
                delete mat;
            }
        };
    private:
        std::string sym;
        TransMatrix *mat;
        bool wrapper = false;
        std::function<TransMatrix(double*)> gen;
};

// Matrix string in the form
// T for translate in 3D, T(1, 2, 3) 
// RX, RY, RZ for rotate
// T(X,Y,Z), RX(theta), RY(theta), RZ(theta)
TransMatrix transformStringToMat(std::string matStr);

// Matrix string translation with variables
std::vector<SymTransMatrix*> symStringToMat(std::string matStr);


// Rotation matrices
TransMatrix matRotX(double theta);
TransMatrix matRotY(double theta);
TransMatrix matRotZ(double theta);

// Translation matrix
TransMatrix matTrans(double x, double y, double z);

// Scaling
TransMatrix matScale(double x, double y, double z);

// Identity Matrix
TransMatrix matIdentity();

void matrixTest();
void matrixMulSpeed();

#endif