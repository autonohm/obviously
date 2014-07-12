#include "obcore/base/PointCloud.h"
#include "obcore/base/Point.h"
#include "obdevice/CloudFactory.h"
#include "obgraphic/CloudWidget.h"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    obvious::CloudWidget view;
    view.show();

    obvious::PointCloud<obvious::Point> cloud;
    obvious::CloudFactory::generateRandomCloud(cloud, 1000);
    view.setCloud(cloud);

    return app.exec();
}
