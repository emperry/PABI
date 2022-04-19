#include "../include/mat.hpp"
#ifdef __AVX__
#include <immintrin.h>
#include <malloc.h>
#elif  __ARM_NEON
#include <arm_neon.h>
#endif
#include <chrono>
#include <random>

#define DEG_TO_RAD(T) T * (M_PI/180.0)

#define RC_TO_IDX(ROW,COL) (4 * ROW) + COL
#define MAT_MUL_RC(M1,ROW,M2,COL) \
    M1[RC_TO_IDX(ROW,0)]*M2[RC_TO_IDX(0,COL)] + \
    M1[RC_TO_IDX(ROW,1)]*M2[RC_TO_IDX(1,COL)] + \
    M1[RC_TO_IDX(ROW,2)]*M2[RC_TO_IDX(2,COL)] + \
    M1[RC_TO_IDX(ROW,3)]*M2[RC_TO_IDX(3,COL)]
#define VEC_MUL_RC(M1,ROW,V) \
    M1[RC_TO_IDX(ROW,0)]*V[0] + \
    M1[RC_TO_IDX(ROW,1)]*V[1] + \
    M1[RC_TO_IDX(ROW,2)]*V[2] + \
    M1[RC_TO_IDX(ROW,3)]*V[3]
// for inverse
#define INV_MUL(M1,ROW) \
    (-M1[RC_TO_IDX(ROW,0)])*M1[RC_TO_IDX(0,3)] + \
    (-M1[RC_TO_IDX(ROW,1)])*M1[RC_TO_IDX(1,3)] + \
    (-M1[RC_TO_IDX(ROW,2)])*M1[RC_TO_IDX(2,3)]

TransMatrix::TransMatrix(double *mat): m1(mat) {};

TransMatrix::TransMatrix() {
    m1 = new double[16] {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
}

TransMatrix::TransMatrix(const TransMatrix &mat) {
    m1 = new double[16];
    for(int i = 0; i < 16; i++) {
        m1[i] = mat.m1[i];
    }
}

TransMatrix &TransMatrix::operator=(const TransMatrix &mat) {
    for(int i = 0; i < 16; i++) {
        m1[i] = mat.m1[i];
    }
    return *this;
}

TransMatrix TransMatrix::mult(TransMatrix &mat) {
    double *m2 = mat.m1;
    double *m3;
    #ifdef __AVX__
    m3 = (double *)memalign(32, sizeof(double)*16);
    auto col = new double[16] {
        m2[RC_TO_IDX(0,0)], m2[RC_TO_IDX(1,0)], m2[RC_TO_IDX(2,0)], 
        m2[RC_TO_IDX(3,0)], m2[RC_TO_IDX(0,1)], m2[RC_TO_IDX(1,1)],
        m2[RC_TO_IDX(2,1)], m2[RC_TO_IDX(3,1)], m2[RC_TO_IDX(0,2)],
        m2[RC_TO_IDX(1,2)], m2[RC_TO_IDX(2,2)], m2[RC_TO_IDX(3,2)],
        m2[RC_TO_IDX(0,3)], m2[RC_TO_IDX(1,3)], m2[RC_TO_IDX(2,3)],
        m2[RC_TO_IDX(3,3)]};
    __m256d col0 = _mm256_loadu_pd(&col[0]);
    __m256d col1 = _mm256_loadu_pd(&col[4]);
    __m256d col2 = _mm256_loadu_pd(&col[8]);
    __m256d col3 = _mm256_loadu_pd(&col[12]);
    delete[] col;
    for(int i = 0; i < 4; i++) {
        // 4 dot products at once w/ AVX instructions
        // https://stackoverflow.com/questions/10454150/intel-avx-256-bits-version-of-dot-product-for-double-precision-floating-point-v
        __m256d row = _mm256_loadu_pd(&m1[RC_TO_IDX(i,0)]);
        __m256d xy0 = _mm256_mul_pd(row, col0);
        __m256d xy1 = _mm256_mul_pd(row, col1);
        __m256d xy2 = _mm256_mul_pd(row, col2);
        __m256d xy3 = _mm256_mul_pd(row, col3);

        // low to high: xy00+xy01 xy10+xy11 xy02+xy03 xy12+xy13
        __m256d temp01 = _mm256_hadd_pd( xy0, xy1 );   

        // low to high: xy20+xy21 xy30+xy31 xy22+xy23 xy32+xy33
        __m256d temp23 = _mm256_hadd_pd( xy2, xy3 );

        // low to high: xy02+xy03 xy12+xy13 xy20+xy21 xy30+xy31
        __m256d swapped = _mm256_permute2f128_pd( temp01, temp23, 0x21 );

        // low to high: xy00+xy01 xy10+xy11 xy22+xy23 xy32+xy33
        __m256d blended = _mm256_blend_pd(temp01, temp23, 0b1100);

        __m256d dotproduct = _mm256_add_pd( swapped, blended );
        _mm256_stream_pd(&m3[RC_TO_IDX(i,0)], dotproduct);
    }
    TransMatrix m(m3);
    m.aligned = true;
    return m;
    #elif __ARM_NEON
    m3 = new double[16];
    auto col = new double[16] {
        m2[RC_TO_IDX(0,0)], m2[RC_TO_IDX(1,0)], m2[RC_TO_IDX(2,0)], 
        m2[RC_TO_IDX(3,0)], m2[RC_TO_IDX(0,1)], m2[RC_TO_IDX(1,1)],
        m2[RC_TO_IDX(2,1)], m2[RC_TO_IDX(3,1)], m2[RC_TO_IDX(0,2)],
        m2[RC_TO_IDX(1,2)], m2[RC_TO_IDX(2,2)], m2[RC_TO_IDX(3,2)],
        m2[RC_TO_IDX(0,3)], m2[RC_TO_IDX(1,3)], m2[RC_TO_IDX(2,3)],
        m2[RC_TO_IDX(3,3)]};
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            float64x2_t halfCol = vld1q_f64(&col[j*4]);
            float64x2_t halfRow = vld1q_f64(&m1[RC_TO_IDX(i,0)]);
            float64x2_t acc = vmulq_f64(halfRow, halfCol);
            halfCol = vld1q_f64(&col[j*4 + 2]);
            halfRow = vld1q_f64(&m1[RC_TO_IDX(i,2)]);
            acc = vfmaq_f64(acc, halfRow, halfCol);
            float64_t dot = vgetq_lane_f64(acc, 0) + vgetq_lane_f64(acc, 1);
            m3[RC_TO_IDX(i,j)] = dot;
	    }
    }
    delete[] col;
    return TransMatrix(m3);
    #else
    m3 = new double[16];
    // Fallback non vectorized implementation
    m3[RC_TO_IDX(0,0)] = MAT_MUL_RC(m1,0,m2,0);
    m3[RC_TO_IDX(0,1)] = MAT_MUL_RC(m1,0,m2,1);
    m3[RC_TO_IDX(0,2)] = MAT_MUL_RC(m1,0,m2,2);
    m3[RC_TO_IDX(0,3)] = MAT_MUL_RC(m1,0,m2,3);

    m3[RC_TO_IDX(1,0)] = MAT_MUL_RC(m1,1,m2,0);
    m3[RC_TO_IDX(1,1)] = MAT_MUL_RC(m1,1,m2,1);
    m3[RC_TO_IDX(1,2)] = MAT_MUL_RC(m1,1,m2,2);
    m3[RC_TO_IDX(1,3)] = MAT_MUL_RC(m1,1,m2,3);

    m3[RC_TO_IDX(2,0)] = MAT_MUL_RC(m1,2,m2,0);
    m3[RC_TO_IDX(2,1)] = MAT_MUL_RC(m1,2,m2,1);
    m3[RC_TO_IDX(2,2)] = MAT_MUL_RC(m1,2,m2,2);
    m3[RC_TO_IDX(2,3)] = MAT_MUL_RC(m1,2,m2,3);

    // Uneccessary for transformation matrices
    /*
    m3[RC_TO_IDX(3,0)] = MAT_MUL_RC(m1,3,m2,0);
    m3[RC_TO_IDX(3,1)] = MAT_MUL_RC(m1,3,m2,1);
    m3[RC_TO_IDX(3,2)] = MAT_MUL_RC(m1,3,m2,2);
    m3[RC_TO_IDX(3,3)] = MAT_MUL_RC(m1,3,m2,3);
    */

    // All transformation matrices have 0, 0, 0, 1
    // for the last row
    m3[RC_TO_IDX(3,0)] = 0;
    m3[RC_TO_IDX(3,1)] = 0;
    m3[RC_TO_IDX(3,2)] = 0;
    m3[RC_TO_IDX(3,3)] = 1;
    return TransMatrix(m3);
    #endif
}

void TransMatrix::multVec(double *v) {
    v[0] = VEC_MUL_RC(m1,0,v);
    v[1] = VEC_MUL_RC(m1,1,v);
    v[2] = VEC_MUL_RC(m1,2,v);
    // v[3] is always 1
}

void TransMatrix::invert() {
    // M^-1 = (1/det(M)) * adj(M) of the inner 3x3 matrix
    // Adjugate of the inner 3x3
    m1[RC_TO_IDX(0,1)] *= -1;
    m1[RC_TO_IDX(1,0)] *= -1;
    m1[RC_TO_IDX(1,2)] *= -1;
    m1[RC_TO_IDX(2,1)] *= -1;
    // multiply the inner 3x3 by 
    // the inverse of the determinant of that 3x3
    auto d = innerDet(m1);
    for(int i = 0; i < 9; i++) {
        m1[i] *= (1/d);
    }
    // multiply the negative inner 3x3 by the right vector
    double x = INV_MUL(m1,0);
    double y = INV_MUL(m1,1);
    double z = INV_MUL(m1,2);
    m1[RC_TO_IDX(0,3)] = x;
    m1[RC_TO_IDX(1,3)] = y;
    m1[RC_TO_IDX(2,3)] = z;
}

double TransMatrix::innerDet(double *m) {
    // 3x3 determinant
    double d = (m[0] * (m[RC_TO_IDX(1,1)]*m[RC_TO_IDX(2,2)] -
                        m[RC_TO_IDX(1,2)]*m[RC_TO_IDX(2,1)])) -
               (m[1] * (m[RC_TO_IDX(1,0)]*m[RC_TO_IDX(2,2)] -
                        m[RC_TO_IDX(2,0)]*m[RC_TO_IDX(1,2)])) +
               (m[2] * (m[RC_TO_IDX(1,0)]*m[RC_TO_IDX(2,1)] -
                        m[RC_TO_IDX(2,0)]*m[RC_TO_IDX(1,1)]));
    return abs(d);
}

void TransMatrix::print() {
    std::cout << std::setprecision(3);
    std::cout.setf(std::ios::fixed);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cout << m1[RC_TO_IDX(i,j)] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

double *TransMatrix::getTranslate() {
    return new double[4] {
        m1[RC_TO_IDX(0,3)],
        m1[RC_TO_IDX(1,3)],
        m1[RC_TO_IDX(2,3)],
        m1[RC_TO_IDX(3,3)]
    };
}

TransMatrix::~TransMatrix() {
    if(aligned) {
        free(m1);
    } else {
        delete[] m1;
    }
}

TransMatrix SymTransMatrix::substitute(double *args) {
    if(wrapper) {
        return TransMatrix(*mat);
    }
    return gen(args);
}

// get arguments needed from operation
int getArgsNeeded(std::string op) {
    if(op[0] == 'R' && op.length() == 2) {
        return 1;
    } else if(op.compare("S") == 0) {
        return 3;
    } else if(op.compare("T") == 0) {
        return 3;
    } else {
        throw std::invalid_argument("invalid operation " + op);
    }
}


// Matches numbers inside the token
std::regex numArgs("[+-]?(\\d*\\.)?\\d+");
// Matches variables inside tokens
// this only works when the op is cut off, (X,Y,Z) not T(X,Y,Z)
std::regex symArgs("(-?[[a-zA-Z]\\[]?[a-zA-Z]\\]?|[a-zA-Z]+\\w+)");
// Splits the matrix string into tokens
std::regex tokenize("[a-zA-Z]+[(][^)]+[)]");


std::deque<std::string> tokenizeMatString(std::string matStr) {
    std::deque<std::string> tokens;
    // Tokenize the string using a regex
    auto begin = std::sregex_iterator(matStr.begin(),
        matStr.end(), tokenize);
    auto end = std::sregex_iterator();
    for(auto i = begin; i != end; i++) {
        auto token = (*i).str();
        tokens.push_back(token);
    }
    return tokens;
}

std::string getMatTokenOp(std::string token) {
    // Get the operation, like RX, or T from a token
    int argEnd = 0;
    for(uint i = 0; i < token.length(); i++) {
        if(token[i] == '(') {
            argEnd = i;
            break;
        }
    }
    return token.substr(0, argEnd);
}

void getNumericArgs(std::string token, double *curArgs, int argsNeeded) {
    // Get the numeric arguments using a regex
    auto begin = std::sregex_iterator(token.begin(),
        token.end(), numArgs);
    auto end = std::sregex_iterator();
    int argCount = 0;
    for(auto i = begin; i != end; i++) {
        auto arg = (*i).str();
        curArgs[argCount] = std::strtod(arg.c_str(), NULL);
        argCount++;
        if(argCount > argsNeeded) {
            throw std::invalid_argument("too many arguments for " + token);
        }
    }
    if(argCount < argsNeeded) {
        throw std::invalid_argument("too few arguments for " + token);
    }
}

TransMatrix getMat(std::string op, double *curArgs) {
    // get transformation matrix from arguments found and the operation
    TransMatrix curMat;
    if(op[0] == 'R' && op.length() == 2) {
        if(op[1] == 'X') {
            curMat = matRotX(curArgs[0]);
        } else if(op[1] == 'Y') {
            curMat = matRotY(curArgs[0]);
        } else if(op[1] == 'Z') {
            curMat = matRotZ(curArgs[0]);
        } else {
            throw std::invalid_argument("invalid axis of rotation " + op[1]);
        }
    } else if(op.compare("S") == 0) {
        curMat = matScale(curArgs[0], curArgs[1], curArgs[2]);
    } else if(op.compare("T") == 0) {
        curMat = matTrans(curArgs[0], curArgs[1], curArgs[2]);
    } else {
        throw std::invalid_argument("invalid operation " + op); 
    }
    return curMat;
}

std::vector<SymTransMatrix*> symStringToMat(std::string matStr) {
    std::vector<SymTransMatrix*> mats;
    auto tokens = tokenizeMatString(matStr);
    std::string token = "";
    while(tokens.size() > 0) {
        token = tokens.front();
        tokens.pop_front();
        auto op = getMatTokenOp(token);
        auto argsNeeded = getArgsNeeded(op);
        // get symbolic or numeric arguments using a regex
        auto headless = token.substr(op.length());
        auto begin = std::sregex_iterator(headless.begin(),
            headless.end(), symArgs);
        auto end = std::sregex_iterator();
        if((*begin).size() > 0) {
            // symbolic arguments
            for(auto i = begin; i != end; i++) {
                auto symToken = (*i).str();
                if(symToken.find('[') != std::string::npos) {
                    // with function
                    throw std::logic_error("functions in matrix strings not implemented");
                } else {
                    // without function
                    mats.push_back(new SymTransMatrix(symToken,
                        [op](double *args) {
                            return getMat(op, args);
                        }));
                }
            }
        } else {
            // numeric arguments
            auto curArgs = new double[3];
            getNumericArgs(token, curArgs, argsNeeded);
            auto curMat = getMat(op, curArgs);
            delete[] curArgs;
            mats.push_back(new SymTransMatrix(curMat));
        }
    }
    std::vector<SymTransMatrix*> condMats;
    // condense the matrix by multiplying non variable matrices together
    SymTransMatrix *curM = NULL;
    for(auto m : mats) {
        // if the current matrix can't be substituted into
        // then push back our current condensed matrix
        // and then push this one, skip the rest
        if(m->getSymName().compare("") != 0) {
            if(curM != NULL) {
                condMats.push_back(curM);
                curM = NULL;
            }
            condMats.push_back(m);
            continue;
        }
        // if our current condensing matrix
        // is empty, then fill it with this one
        if(curM == NULL) {
            curM = m;
        } else {        
            // get the matrices out of the SymTransMatrix, multiply, then
            // and wrap it in a new SymTransMatrix
            auto subs = m->substitute(NULL);
            auto newM = new SymTransMatrix(curM->substitute(NULL).mult(subs));
            delete m;
            delete curM;
            curM = newM;
        }
    }
    if(curM != NULL) {
        condMats.push_back(curM);
    }
    return condMats;
}

TransMatrix transformStringToMat(std::string matStr) {
    auto mat = matIdentity();
    auto tokens = tokenizeMatString(matStr);
    std::string token = "";
    while(tokens.size() > 0) {
        token = tokens.front();
        tokens.pop_front();
        auto op = getMatTokenOp(token);
        int argsNeeded = getArgsNeeded(op);
        auto curArgs = new double[3];
        getNumericArgs(token, curArgs, argsNeeded);
        auto curMat = getMat(op, curArgs);
        delete[] curArgs;
        // multiply by the current matrix and store in current matrix
        mat = mat.mult(curMat);
    }
    return mat;
}

TransMatrix matRotX(double theta) {
    auto *m = new double[16] {
        1, 0,           0,          0,
        0, cos(theta), -sin(theta), 0,
        0, sin(theta),  cos(theta), 0,
        0, 0,           0,          1
    };
    return TransMatrix(m);
}


TransMatrix matRotY(double theta) {
    auto *m = new double[16] {
        cos(theta), 0, sin(theta), 0,
        0,          1,  0,          0,
        -sin(theta), 0,  cos(theta), 0,
        0,          0,  0,          1
    };
    return TransMatrix(m);
}

TransMatrix matRotZ(double theta) {
    auto *m = new double[16] {
        cos(theta), -sin(theta), 0, 0,
        sin(theta),  cos(theta), 0, 0,
        0,           0,          1, 0,
        0,           0,          0, 1
    };
    return TransMatrix(m);
}

TransMatrix matTrans(double x, double y, double z) {
    auto *m = new double[16] {
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z,
        0, 0, 0, 1
    };
    return TransMatrix(m);
}

TransMatrix matScale(double x, double y, double z) {
    auto *m = new double[16] {
        x, 0, 0, 0,
        0, y, 0, 0,
        0, 0, z, 0,
        0, 0, 0, 1
    };
    return TransMatrix(m);
}

TransMatrix matIdentity() {
    return TransMatrix(new double[16] {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    });
}

void matrixMulSpeed() {
    std::default_random_engine g;
    std::uniform_real_distribution<double> d(0.0,1.0);
    auto before = std::chrono::steady_clock::now();
    for(int i = 0; i < 1000; i++) {
        auto m1 = new double[16] {
            d(g), d(g), d(g), d(g),
            d(g), d(g), d(g), d(g),
            d(g), d(g), d(g), d(g),
            0, 0, 0, 1
        };
        auto m2 = new double[16] {
            d(g), d(g), d(g), d(g),
            d(g), d(g), d(g), d(g),
            d(g), d(g), d(g), d(g),
            0, 0, 0, 1
        };
        auto mat1 = TransMatrix(m1);
        auto mat2 = TransMatrix(m2);
        auto mat3 = mat1.mult(mat2);
    }
    auto after = std::chrono::steady_clock::now();
    std::cout << "1000 matrix multiplications performed in: " <<
        (after - before).count() << 
        " ns" << std::endl;
}

// Testing function
void matrixTest() {
    // test RC_TO_IDX
    std::cout << "RC_TO_IDX, Expected: Actual" << std::endl 
    << "0: " << RC_TO_IDX(0,0) << std::endl
    << "7: " << RC_TO_IDX(1,3) << std::endl
    << "10: " << RC_TO_IDX(2,2) << std::endl
    << "12: " << RC_TO_IDX(3,0) << std::endl 
    << "2: " << RC_TO_IDX(0,2) << std::endl
    << "3: " << RC_TO_IDX(0,3) << std::endl << std::endl;

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            std::cout << RC_TO_IDX(i, j) << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // 2 identity matrices
    double *m1 = new double[16] {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    double *m2 = new double[16] {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    TransMatrix mat1(m1);
    TransMatrix mat2(m2);
    TransMatrix mat3 = mat1.mult(mat2);
    std::cout << "Should print I" << std::endl;
    mat3.print();
    

    // Adding 2 translations
    mat1 = matTrans(1, 2, 3);
    mat2 = matTrans(3, 2, 1);
    mat3 = mat1.mult(mat2);
    std::cout << "T(3,2,1)T(1,2,3) = T(4,4,4)" << std::endl;
    mat1.print();
    mat2.print();
    mat3.print();

    std::cout << "Multiply by [0, 0, 0, 1]" << std::endl;
    // Move [0, 0, 0, 1] by T(4,4,4)
    auto v = new double[4] {0, 0, 0, 1};
    mat3.multVec(v);
    for(int i = 0; i < 4; i++) {
        std::cout << v[i] << " ";
    }
    std::cout << std::endl;
    delete[] v;

    // counterclockwise and clockwise rotation to make identity
    mat1 = matRotX(DEG_TO_RAD(45));
    mat2 = matRotX(DEG_TO_RAD(-45));
    mat3 = mat1.mult(mat2);
    std::cout << "Rx(45)Rx(-45) = I" << std::endl;
    mat1.print();
    mat2.print();
    mat3.print();

    mat1 = matRotX(DEG_TO_RAD(30));
    mat2 = matRotX(DEG_TO_RAD(-30));
    mat3 = mat1.mult(mat2);
    std::cout << "Rx(30)Rx(-30) = I" << std::endl;
    mat1.print();
    mat2.print();
    mat3.print();

    mat1 = matTrans(1, 2, 3);
    mat2 = matRotX(DEG_TO_RAD(30));
    mat3 = mat1.mult(mat2);
    std::cout << "T(1,2,3)Rx(30) = Both" << std::endl;
    mat1.print();
    mat2.print();
    mat3.print();

    mat1 = matTrans(1, 2, 3);
    mat2 = matRotX(DEG_TO_RAD(30));
    mat3 = matTrans(3,2,1);
    auto mat4 = mat1.mult(mat2).mult(mat3);
    std::cout << "T(1,2,3)Rx(30)T(3,2,1) = All 3" << std::endl;
    mat4.print();
    std::cout << "Expected:" << std::endl;
    TransMatrix(new double[16] {
        1, 0, 0, 4,
        0, 0.866, -0.5, 3.232,
        0, 0.5, 0.866, 4.866,
        0, 0, 0, 1
    }).print();

    std::cout << "Invert the previous matrix" << std::endl;
    mat4.invert();
    mat4.print();
    std::cout << "Invert back" << std::endl;
    mat4.invert();
    mat4.print();
    std::cout << "Expected:" << std::endl;
    TransMatrix(new double[16] {
        1, 0, 0, -4,
        0, 0.866, 0.5, -5.232,
        0, -0.5, 0.866, -2.598,
        0, 0, 0, 1
    }).print();

    std::cout << "T(1,2,3)" << std::endl;
    transformStringToMat("T(1,2,3)").print();

    std::cout << "T(1,2,3)T(3,2,1)" << std::endl;
    transformStringToMat("T(1,2,3)T(3,2,1)").print();

    std::cout << "T(1, 2,3)T(3,2, 1)" << std::endl;
    transformStringToMat("T(1, 2,3)T(3,2, 1)").print();

    std::cout << "T(1, 2, 3)T(3, 2, 1)" << std::endl;
    transformStringToMat("T(1, 2, 3)T(3, 2, 1)").print();

    std::cout << "T(1, 2, 3)T(3, 2, 1)RX(30)RX(-30)" << std::endl;
    transformStringToMat("T(1, 2, 3)T(3, 2, 1)RX(30)RX(-30)").print();

    std::cout << "RX(45)RZ(45)RZ(-45)RX(-45)" << std::endl;
    transformStringToMat("RX(45)RZ(45)RZ(-45)RX(-45)").print();

    std::cout << "RX(45)RZ(45)RY(-32.86)RY(32.86)RZ(-45)RX(-45)" << std::endl;
    transformStringToMat("RX(45)RZ(45)RY(-32.86)RY(32.86)RZ(-45)RX(-45)").print();

    std::cout << "RX(14.5)RX(14.50000000)RX(0.5)RX(0.50)" << std::endl;
    transformStringToMat("RX(14.5)RX(14.50000000)RX(0.5)RX(0.50)").print();

    std::cout << "T(1,2,3)RX(t1)" << std::endl;
    auto mats = symStringToMat("T(1,2,3)RX(t1)");
    for(auto mat : mats) {
        auto a = new double[3];
        a[0] = DEG_TO_RAD(45);
        mat->substitute(a).print();
        delete[] a;
    }
}
