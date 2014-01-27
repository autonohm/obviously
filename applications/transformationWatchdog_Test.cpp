
#include "obcore/math/linalg/linalg.h"
#include "obcore/math/TransformationWatchdog.h"

int main(void)
{
	double dataT1[16] = { 1.0, 0.0, 0.0, 0.0,
												0.0, 1.0, 0.0, 0.0,
												0.0, 0.0, 1.0, 0.0,
												0.0, 0.0, 0.0, 1.0};
	obvious::Matrix T1(4,4, dataT1);

	double dataT2[16] = { 1.0, 0.0, 0.0, 0.2,
												0.0, 1.0, 0.0, 0.0,
												0.0, 0.0, 1.0, 0.0,
												0.0, 0.0, 0.0, 1.0};
	obvious::Matrix T2(4,4, dataT2);
  double dataT3[16] = {0.939692620785908,	0.342020143325669,	0.0, 0.0,
											-0.342020143325669,	0.939692620785908,	0.0, 0.0,
											0.0,	0.0, 	1.0, 0.0,
											0.0,	0.0, 	0.0, 1.0};
	obvious::Matrix T3(4,4, dataT3);

	obvious::TransformationWatchdog watchdog;
	watchdog.setRotationThreshold(0.34);
	watchdog.setTranslationThreshold(0.05);
	watchdog.setInitTransformation(T1);

  std::cout << "First check with initial matrix (expected 0)" << std::endl;
	std::cout << watchdog.checkWatchdog(T1) << std::endl;

  std::cout << "First check with translation matrix (expected 1)" << std::endl;
	std::cout << watchdog.checkWatchdog(T2) << std::endl;

  std::cout << "First check with rotation matrix (expected 1)" << std::endl;
	std::cout << watchdog.checkWatchdog(T3) << std::endl;
}
