#include <QApplication>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtMath>
using namespace QtCharts;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 1. 准备数据
    QLineSeries *series = new QLineSeries();
    for (double x = 0; x <= 10; x += 0.5)
        series->append(x, qSin(x));

    // 2. 图表
    QChart *chart = new QChart();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("Qt5 Charts 折线图");

    // 3. 显示
    QChartView view(chart);
    view.resize(800, 600);
    view.show();
    return a.exec();
}