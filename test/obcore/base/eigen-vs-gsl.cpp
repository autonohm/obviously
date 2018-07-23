#include "obcore/base/Time.h"
#include "obcore/base/Timer.h"

#include <unistd.h>
#include <iostream>

#include <Eigen/Core>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

namespace {
const unsigned int DIM = 4000000;
}

int main(int argc, char** argv)
{
    std::cout << "will test lib eigen with float and row major..." << std::endl;

    obvious::Timer timer;

    {
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> mat(DIM, 3);
        mat = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Random(DIM, 3);

        Eigen::Matrix3f trans(Eigen::Matrix3f::Random());
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> result(DIM, 3);
        timer.start();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test lib eigen with double and row major..." << std::endl;

    {
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> mat(DIM, 3);
        mat = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>::Random(DIM, 3);

        Eigen::Matrix3d trans(Eigen::Matrix3d::Random());
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> result(DIM, 3);
        timer.start();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test lib eigen with float and col major..." << std::endl;

    {
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::ColMajor> mat(DIM, 3);
        mat = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::ColMajor>::Random(DIM, 3);

        Eigen::Matrix3f trans(Eigen::Matrix3f::Random());
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::ColMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test lib eigen with double and col major..." << std::endl;

    {
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor> mat(DIM, 3);
        mat = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor>::Random(DIM, 3);

        Eigen::Matrix3d trans(Eigen::Matrix3d::Random());
        Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::ColMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test lib eigen with int and row major..." << std::endl;

    {
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> mat(DIM, 3);
        mat = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::ColMajor>::Random(DIM, 3);

        Eigen::Matrix<int, 3, 3> trans(Eigen::Matrix<int, 3, 3>::Random(3, 3));
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test lib eigen with int and col major..." << std::endl;

    {
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::ColMajor> mat(DIM, 3);
        mat = Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::ColMajor>::Random(DIM, 3);

        Eigen::Matrix<int, 3, 3> trans(Eigen::Matrix<int, 3, 3>::Random(3, 3));
        Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::ColMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test gsl with double..." << std::endl;

    {
        gsl_matrix* mat = gsl_matrix_alloc(DIM, 3);
        gsl_matrix* trans = gsl_matrix_alloc(3, 3);
        gsl_matrix* result = gsl_matrix_alloc(DIM, 3);
        gsl_matrix_set_all(mat, 1234.4567f);
        gsl_matrix_set_all(trans, 4321.32f);
        timer.reset();
        gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat, trans, 0.0, result);
        std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;
        gsl_matrix_free(mat);
        gsl_matrix_free(trans);
        gsl_matrix_free(result);
    }


    std::cout << "will test gsl with float..." << std::endl;

    {
        gsl_matrix_float* mat = gsl_matrix_float_alloc(DIM, 3);
        gsl_matrix_float* trans = gsl_matrix_float_alloc(3, 3);
        gsl_matrix_float* result = gsl_matrix_float_alloc(DIM, 3);
        gsl_matrix_float_set_all(mat, 1234.4567f);
        gsl_matrix_float_set_all(trans, 4321.32f);
        timer.reset();
        gsl_blas_sgemm(CblasNoTrans, CblasNoTrans, 1.0, mat, trans, 0.0, result);
        std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;
        gsl_matrix_float_free(mat);
        gsl_matrix_float_free(trans);
        gsl_matrix_float_free(result);
    }


    std::cout << "will test eigen mapping an external array..." << std::endl;

    {
        float* data = new float[DIM * 3];
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> > mat(data, DIM, 3);
        mat = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Random(DIM, 3);
        Eigen::Matrix3f trans(Eigen::Matrix3f::Random());
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;


    std::cout << "will test eigen mapping an external array with stride..." << std::endl;

    {
        struct Coord {
            float x;
            float y;
            float z;
        };

        struct Color {
            unsigned char r;
            unsigned char g;
            unsigned char b;
        };

        struct Point {
            Coord coord;
            Color color;
        };

        union Memory {
            Point point;
            float data[4];
        };

        std::cout << "size of Memory = " << sizeof(Memory) << std::endl;

        Memory* mem = new Memory[DIM];
        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>, 0, Eigen::InnerStride<1> > mat(mem[0].data, DIM, 3);
        mat = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Random(DIM, 3);
        Eigen::Matrix3f trans(Eigen::Matrix3f::Random());
        Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> result(DIM, 3);
        timer.reset();
        result = mat * trans;
    }

    std::cout << "elapsed time = " << timer.elapsed() << std::endl << std::endl;
}
