#include "obcore/base/PointCloud.h"
#include "obcore/base/Point.h"
#include "obdevice/CloudFactory.h"
#include "obgraphic/CloudWidget.h"
#include "obcore/base/Timer.h"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    obvious::CloudWidget viewA;
    obvious::CloudWidget viewB;
    viewA.show();
    viewB.show();

    obvious::PointCloud<obvious::Point> cloud;
//    obvious::CloudFactory::generateRandomCloud(cloud, 1000);
    obvious::CloudFactory::loadCloud(cloud, "/home/knueppl/git/libra3d/build/12:55:10.386.pcd");
    viewA.setCloud(cloud);

    obvious::Timer timer;
    timer.start();

    cloud.rotate(90.0f * M_PI / 180.0f, 0.0f, 0.0f);

    std::cout << "time = " << timer.elapsed() << std::endl;
    viewB.setCloud(cloud);

    return app.exec();
}
