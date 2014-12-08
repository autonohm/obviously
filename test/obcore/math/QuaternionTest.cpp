#include <iostream>

#include "gtest/gtest.h"

#include "obcore/math/Quaternion.h"
#include "obcore/math/mathbase.h"
#include "obcore/math/linalg/MatrixFactory.h"
#include <math.h>

using namespace obvious;

TEST(quaternion_test_constructor, quaternion_test)
{
  Quaternion q;

  EXPECT_EQ(q.w(), 1.0);
  EXPECT_EQ(q.x(), 0.0);
  EXPECT_EQ(q.y(), 0.0);
  EXPECT_EQ(q.z(), 0.0);
}

TEST(quaternion_test_constructor2, quaternion_test)
{
  double len    = 1.0/sqrt(3.0);
  double cosine = cos(deg2rad(45.0));
  double sine   = sin(deg2rad(45.0));

  Quaternion q(cosine, sine*len, sine*len, sine*len);

  EXPECT_EQ(q.w(), cosine);
  EXPECT_EQ(q.x(), sine*len);
  EXPECT_EQ(q.y(), sine*len);
  EXPECT_EQ(q.z(), sine*len);
}

TEST(quaternion_test_constructor3, quaternion_test)
{
  double phi = deg2rad(45.0);
  Matrix R = MatrixFactory::RotationMatrix22(phi);

  Quaternion q(R);

  EXPECT_EQ(q.w(), cos(phi/2.0));
  EXPECT_EQ(q.x(), 0.0);
  EXPECT_EQ(q.y(), 0.0);
  EXPECT_EQ(q.z(), sin(phi/2.0));
}

TEST(quaternion_test_constructor4, quaternion_test)
{
  double phi = deg2rad(180.0);
  Matrix R = MatrixFactory::RotationMatrix22(phi);

  Quaternion q(R);

  EXPECT_EQ(q.w(), cos(phi/2.0));
  EXPECT_EQ(q.x(), 0.0);
  EXPECT_EQ(q.y(), 0.0);
  EXPECT_EQ(q.z(), sin(phi/2.0));
}

TEST(quaternion_test_constructor5, quaternion_test)
{
  double phi   = deg2rad(75.0);
  Matrix R = MatrixFactory::RotationMatrix33AboutZ(phi);

  Quaternion q(R);

  EXPECT_EQ(q.w(), cos(phi/2.0));
  EXPECT_EQ(q.x(), 0.0);
  EXPECT_EQ(q.y(), 0.0);
  EXPECT_EQ(q.z(), sin(phi/2.0));
}

TEST(quaternion_test_constructor6, quaternion_test)
{
  double psi   = deg2rad(45.0);
  double theta = deg2rad(60.0);
  double phi   = deg2rad(75.0);
  Matrix Rx = MatrixFactory::RotationMatrix33AboutX(psi);
  Matrix Ry = MatrixFactory::RotationMatrix33AboutY(theta);
  Matrix Rz = MatrixFactory::RotationMatrix33AboutZ(phi);
  Matrix R = Rx * Ry * Rz;
  Quaternion q(R);
  Matrix R2 = q.convertToMatrix();

  EXPECT_DOUBLE_EQ(R(0,0), R2(0,0));
  EXPECT_DOUBLE_EQ(R(0,1), R2(0,1));
  EXPECT_DOUBLE_EQ(R(0,2), R2(0,2));

  EXPECT_DOUBLE_EQ(R(1,0), R2(1,0));
  EXPECT_DOUBLE_EQ(R(1,1), R2(1,1));
  EXPECT_DOUBLE_EQ(R(1,2), R2(1,2));

  EXPECT_DOUBLE_EQ(R(2,0), R2(2,0));
  EXPECT_DOUBLE_EQ(R(2,1), R2(2,1));
  EXPECT_DOUBLE_EQ(R(2,2), R2(2,2));
}
