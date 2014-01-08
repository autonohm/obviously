#include <iostream>

#include "gtest/gtest.h"

#include "obcore/math/linalg/linalg.h"

using namespace obvious;

double data[] = {17.0, 13.0, 2.0, 9.0, 44.0, 1.0, 116.0, 43.0, 2.0};

TEST(matrix_test_constructor, matrix_test)
{
  Matrix M(3, 3, data);

  EXPECT_EQ(M(0,0), data[0]);
  EXPECT_EQ(M(0,1), data[1]);
  EXPECT_EQ(M(0,2), data[2]);
  EXPECT_EQ(M(1,0), data[3]);
  EXPECT_EQ(M(1,1), data[4]);
  EXPECT_EQ(M(1,2), data[5]);
  EXPECT_EQ(M(2,0), data[6]);
  EXPECT_EQ(M(2,1), data[7]);
  EXPECT_EQ(M(2,2), data[8]);
}

TEST(matrix_test_copy_constructor, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2(M, 0, 0, 2, 2);
  EXPECT_EQ(M2(0,0), M(0,0));
  EXPECT_EQ(M2(0,1), M(0,1));
  EXPECT_EQ(M2(1,0), M(1,0));
  EXPECT_EQ(M2(1,1), M(1,1));

  Matrix M3(M, 1, 1, 2, 2);
  EXPECT_EQ(M3(0,0), M(1,1));
  EXPECT_EQ(M3(0,1), M(1,2));
  EXPECT_EQ(M3(1,0), M(2,1));
  EXPECT_EQ(M3(1,1), M(2,2));
}

TEST(matrix_test_assignment_operator, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;

  for(int r=0; r<3; r++)
    for(int c=0; c<3; c++)
      EXPECT_EQ(M(r,c), M2(r,c));
}

TEST(matrix_test_add_scalar, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;
  M2 += 5.0;

  for(int r=0; r<3; r++)
    for(int c=0; c<3; c++)
      EXPECT_EQ(M2(r,c), M(r,c)+5.0);
}

TEST(matrix_test_add_scalar_to_column, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;
  M2.addToColumn(0, 5.0);

  for(int r=0; r<3; r++)
    EXPECT_EQ(M2(r,0), M(r,0)+5.0);

  for(int r=0; r<3; r++)
    for(int c=1; c<3; c++)
      EXPECT_EQ(M2(r,c), M(r,c));
}

TEST(matrix_test_add_scalar_to_row, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;
  M2.addToRow(0, 5.0);

  for(int c=0; c<3; c++)
    EXPECT_EQ(M2(0,c), M(0,c)+5.0);

  for(int r=1; r<3; r++)
    for(int c=0; c<3; c++)
      EXPECT_EQ(M2(r,c), M(r,c));
}

TEST(matrix_test_multiply_equal_operator, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;
  M2 *= M;

  EXPECT_EQ(M2(0,0), data[0]*data[0] + data[1]*data[3] + data[2]*data[6]);
  EXPECT_EQ(M2(0,1), data[0]*data[1] + data[1]*data[4] + data[2]*data[7]);
  EXPECT_EQ(M2(0,2), data[0]*data[2] + data[1]*data[5] + data[2]*data[8]);
  EXPECT_EQ(M2(1,0), data[3]*data[0] + data[4]*data[3] + data[5]*data[6]);
  EXPECT_EQ(M2(1,1), data[3]*data[1] + data[4]*data[4] + data[5]*data[7]);
  EXPECT_EQ(M2(1,2), data[3]*data[2] + data[4]*data[5] + data[5]*data[8]);
  EXPECT_EQ(M2(2,0), data[6]*data[0] + data[7]*data[3] + data[8]*data[6]);
  EXPECT_EQ(M2(2,1), data[6]*data[1] + data[7]*data[4] + data[8]*data[7]);
  EXPECT_EQ(M2(2,2), data[6]*data[2] + data[7]*data[5] + data[8]*data[8]);
}

TEST(matrix_test_multiply_right, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M;
  Matrix M3 = M;

  M2 *= M;
  M3.multiplyRight(M, false, false);

  for(int r=0; r<3; r++)
    for(int c=0; c<3; c++)
      EXPECT_EQ(M2(r,c), M3(r,c));
}

TEST(matrix_test_set_data, matrix_test)
{
  double data2[] = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0};
  Matrix M(3, 3, data);
  M.setData(data2);

  EXPECT_EQ(M(0,0), data2[0]);
  EXPECT_EQ(M(0,1), data2[1]);
  EXPECT_EQ(M(0,2), data2[2]);
  EXPECT_EQ(M(1,0), data2[3]);
  EXPECT_EQ(M(1,1), data2[4]);
  EXPECT_EQ(M(1,2), data2[5]);
  EXPECT_EQ(M(2,0), data2[6]);
  EXPECT_EQ(M(2,1), data2[7]);
  EXPECT_EQ(M(2,2), data2[8]);
}

TEST(matrix_test_get_data, matrix_test)
{
  double data2[] = {10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0};
  double data3[9];

  Matrix M(3, 3, data);
  M.setData(data2);
  M.getData(data3);

  EXPECT_EQ(data2[0], data3[0]);
  EXPECT_EQ(data2[1], data3[1]);
  EXPECT_EQ(data2[2], data3[2]);
  EXPECT_EQ(data2[3], data3[3]);
  EXPECT_EQ(data2[4], data3[4]);
  EXPECT_EQ(data2[5], data3[5]);
  EXPECT_EQ(data2[6], data3[6]);
  EXPECT_EQ(data2[7], data3[7]);
  EXPECT_EQ(data2[8], data3[8]);
}

TEST(matrix_test_properties, matrix_test)
{
  Matrix M(3, 3, data);

  EXPECT_EQ(M.getRows(), 3);
  EXPECT_EQ(M.getCols(), 3);
}

TEST(matrix_test_identity, matrix_test)
{
  Matrix M(3, 3, data);
  M.setIdentity();

  EXPECT_EQ(M(0,0), 1.0);
  EXPECT_EQ(M(0,1), 0.0);
  EXPECT_EQ(M(0,2), 0.0);
  EXPECT_EQ(M(1,0), 0.0);
  EXPECT_EQ(M(1,1), 1.0);
  EXPECT_EQ(M(1,2), 0.0);
  EXPECT_EQ(M(2,0), 0.0);
  EXPECT_EQ(M(2,1), 0.0);
  EXPECT_EQ(M(2,2), 1.0);
}

TEST(matrix_test_zero, matrix_test)
{
  Matrix M(3, 3, data);
  M.setZero();

  EXPECT_EQ(M(0,0), 0.0);
  EXPECT_EQ(M(0,1), 0.0);
  EXPECT_EQ(M(0,2), 0.0);
  EXPECT_EQ(M(1,0), 0.0);
  EXPECT_EQ(M(1,1), 0.0);
  EXPECT_EQ(M(1,2), 0.0);
  EXPECT_EQ(M(2,0), 0.0);
  EXPECT_EQ(M(2,1), 0.0);
  EXPECT_EQ(M(2,2), 0.0);
}

TEST(matrix_test_transpose, matrix_test)
{
  Matrix M(3, 3, data);
  M.transpose();

  EXPECT_EQ(M(0,0), data[0]);
  EXPECT_EQ(M(0,1), data[3]);
  EXPECT_EQ(M(0,2), data[6]);
  EXPECT_EQ(M(1,0), data[1]);
  EXPECT_EQ(M(1,1), data[4]);
  EXPECT_EQ(M(1,2), data[7]);
  EXPECT_EQ(M(2,0), data[2]);
  EXPECT_EQ(M(2,1), data[5]);
  EXPECT_EQ(M(2,2), data[8]);
}

TEST(matrix_test_get_transpose, matrix_test)
{
  Matrix M(3, 3, data);

  Matrix M2 = M.getTranspose();

  for(int r=0; r<3; r++)
    for(int c=0; c<3; c++)
      EXPECT_EQ(M(r,c), M2(c,r));
}

TEST(matrix_test_trace, matrix_test)
{
  Matrix M(3, 3, data);

  EXPECT_EQ(M.trace(), data[0]+data[4]+data[8]);
}

TEST(matrix_test_solve, matrix_test)
{
  double data2[] = {1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 1.0, 2.0, 3.0};
  double b[] = {6.0, 11.0, 14.0};
  double x[3];

  Matrix M(3, 3, data2);
  M.solve(b, x);
  EXPECT_EQ(x[0], 1.0);
  EXPECT_EQ(x[1], 2.0);
  EXPECT_EQ(x[2], 3.0);
}

TEST(matrix_test_multiply_generic, matrix_test)
{
  double data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  Matrix M(3, 3, data2);
  Matrix M2(3, 3, data2);

  Matrix Mres = Matrix::multiply(M, M2, false, false);
  EXPECT_EQ(Mres(0, 0), 30.0);
  EXPECT_EQ(Mres(0, 1), 36.0);
  EXPECT_EQ(Mres(0, 2), 42.0);
  EXPECT_EQ(Mres(1, 0), 66.0);
  EXPECT_EQ(Mres(1, 1), 81.0);
  EXPECT_EQ(Mres(1, 2), 96.0);
  EXPECT_EQ(Mres(2, 0), 102.0);
  EXPECT_EQ(Mres(2, 1), 126.0);
  EXPECT_EQ(Mres(2, 2), 150.0);

  Matrix Mres2 = Matrix::multiply(M, M2, true, false);
  EXPECT_EQ(Mres2(0, 0), 66.0);
  EXPECT_EQ(Mres2(0, 1), 78.0);
  EXPECT_EQ(Mres2(0, 2), 90.0);
  EXPECT_EQ(Mres2(1, 0), 78.0);
  EXPECT_EQ(Mres2(1, 1), 93.0);
  EXPECT_EQ(Mres2(1, 2), 108.0);
  EXPECT_EQ(Mres2(2, 0), 90.0);
  EXPECT_EQ(Mres2(2, 1), 108.0);
  EXPECT_EQ(Mres2(2, 2), 126.0);

  Matrix Mres3 = Matrix::multiply(M, M2, false, true);
  EXPECT_EQ(Mres3(0, 0), 14.0);
  EXPECT_EQ(Mres3(0, 1), 32.0);
  EXPECT_EQ(Mres3(0, 2), 50.0);
  EXPECT_EQ(Mres3(1, 0), 32.0);
  EXPECT_EQ(Mres3(1, 1), 77.0);
  EXPECT_EQ(Mres3(1, 2), 122.0);
  EXPECT_EQ(Mres3(2, 0), 50.0);
  EXPECT_EQ(Mres3(2, 1), 122.0);
  EXPECT_EQ(Mres3(2, 2), 194.0);

  Matrix Mres4 = Matrix::multiply(M, M2, true, true);
  EXPECT_EQ(Mres4(0, 0), 30.0);
  EXPECT_EQ(Mres4(0, 1), 66.0);
  EXPECT_EQ(Mres4(0, 2), 102.0);
  EXPECT_EQ(Mres4(1, 0), 36.0);
  EXPECT_EQ(Mres4(1, 1), 81.0);
  EXPECT_EQ(Mres4(1, 2), 126.0);
  EXPECT_EQ(Mres4(2, 0), 42.0);
  EXPECT_EQ(Mres4(2, 1), 96.0);
  EXPECT_EQ(Mres4(2, 2), 150.0);
}

TEST(matrix_test_vector_multiply, matrix_test)
{
  double data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  double data3[] = {1.0, 2.0, 3.0};
  Matrix M(3, 3, data2);
  Vector V(3, data3);
  Vector Vres = Matrix::multiply(M, V, false);

  EXPECT_EQ(Vres(0), 14.0);
  EXPECT_EQ(Vres(1), 32.0);
  EXPECT_EQ(Vres(2), 50.0);

  Vector Vres2 = Matrix::multiply(M, V, true);

  EXPECT_EQ(Vres2(0), 30.0);
  EXPECT_EQ(Vres2(1), 36.0);
  EXPECT_EQ(Vres2(2), 42.0);
}

TEST(matrix_test_matrix_transform, matrix_test)
{
  double data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  double tdata[] = {0.0, -1.0, 0.0, 0.5,
                    1.0, 0.0, 0.0, 0.6,
                    0.0, 0.0, 1.0, 0.7};

  Matrix coords(3, 3, data2);
  Matrix T(3, 4, tdata);

  coords.transform(T);

  EXPECT_EQ(coords(0,0), -1.5);
  EXPECT_EQ(coords(0,1), 1.6);
  EXPECT_EQ(coords(0,2), 3.7);
  EXPECT_EQ(coords(1,0), -4.5);
  EXPECT_EQ(coords(1,1), 4.6);
  EXPECT_EQ(coords(1,2), 6.7);
  EXPECT_EQ(coords(2,0), -7.5);
  EXPECT_EQ(coords(2,1), 7.6);
  EXPECT_EQ(coords(2,2), 9.7);
}

TEST(matrix_test_matrix_multiply_array, matrix_test)
{
  double data2[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  double rdata[] = {0.0, -1.0, 0.0,
                    1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0};

  Matrix R(3, 3, rdata);

  Matrix::multiply(R, data2, 3, 3);

  EXPECT_EQ(data2[0], -2.0);
  EXPECT_EQ(data2[1], 1.0);
  EXPECT_EQ(data2[2], 3.0);
  EXPECT_EQ(data2[3], -5.0);
  EXPECT_EQ(data2[4], 4.0);
  EXPECT_EQ(data2[5], 6.0);
  EXPECT_EQ(data2[6], -8.0);
  EXPECT_EQ(data2[7], 7.0);
  EXPECT_EQ(data2[8], 9.0);
}
