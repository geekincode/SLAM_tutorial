#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QDebug>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

QT_CHARTS_USE_NAMESPACE

/**
 * @brief 从文件读取数据点
 * @param filename 文件名
 * @param x_data x坐标数据
 * @param y_data y坐标数据
 * @return 是否读取成功
 */
bool readDataFromFile(const QString& filename, std::vector<double>& x_data, std::vector<double>& y_data) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Cannot open file for reading:" << filename;
        return false;
    }

    QTextStream in(&file);
    int lineCount = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(" ", Qt::SkipEmptyParts);
        if (parts.size() >= 2) {
            bool xOk, yOk;
            double x = parts[0].toDouble(&xOk);
            double y = parts[1].toDouble(&yOk);
            if (xOk && yOk) {
                x_data.push_back(x);
                y_data.push_back(y);
                lineCount++;
            }
        }
    }
    file.close();
    std::cout << "Read " << lineCount << " data points from " << filename.toStdString() << std::endl;
    return lineCount > 0;
}

/**
 * @brief 根据参数计算指数函数值 y = exp(a*x^2 + b*x + c)
 * @param x x坐标
 * @param a 参数a
 * @param b 参数b
 * @param c 参数c
 * @return 函数值
 */
double calculateFunction(double x, double a, double b, double c) {
    return std::exp(a * x * x + b * x + c);
}

/**
 * @brief 创建并显示曲线拟合结果的可视化图表
 * @param x_data 原始数据x坐标
 * @param y_data 原始数据y坐标
 * @param a 拟合参数a
 * @param b 拟合参数b
 * @param c 拟合参数c
 */
void visualizeCurveFitting(const std::vector<double>& x_data, 
                          const std::vector<double>& y_data,
                          double a, double b, double c) {
    std::cout << "Creating visualization window..." << std::endl;
    
    // 检查数据是否为空
    if (x_data.empty() || y_data.empty()) {
        std::cerr << "Error: No data to visualize!" << std::endl;
        return;
    }
    
    // 创建主窗口
    QMainWindow *window = new QMainWindow();
    window->setWindowTitle("g2o Curve Fitting Visualization");
    window->resize(800, 600);
    
    // 创建图表
    QChart *chart = new QChart();
    
    // 使用标准C++流来格式化标题
    std::ostringstream titleStream;
    titleStream << "Curve Fitting Results: y = exp(" 
                << std::fixed << std::setprecision(3) << a << "x² + " 
                << std::fixed << std::setprecision(3) << b << "x + " 
                << std::fixed << std::setprecision(3) << c << ")";
    QString title = QString::fromStdString(titleStream.str());
    
    chart->setTitle(title);
    chart->setAnimationOptions(QChart::SeriesAnimations);
    
    // 创建散点序列显示原始数据
    QScatterSeries *scatterSeries = new QScatterSeries();
    scatterSeries->setName("Original Data Points");
    scatterSeries->setMarkerSize(8.0);
    
    // 添加原始数据点
    for (size_t i = 0; i < x_data.size(); ++i) {
        scatterSeries->append(x_data[i], y_data[i]);
    }
    
    std::cout << "Added " << x_data.size() << " data points to scatter series" << std::endl;
    
    // 创建线序列显示拟合曲线
    QLineSeries *lineSeries = new QLineSeries();
    lineSeries->setName("Fitted Curve");
    
    // 生成拟合曲线点
    const int numPoints = 300;
    double minX = *std::min_element(x_data.begin(), x_data.end());
    double maxX = *std::max_element(x_data.begin(), x_data.end());
    double step = (maxX - minX) / (numPoints - 1);
    
    for (int i = 0; i < numPoints; ++i) {
        double x = minX + i * step;
        double y = calculateFunction(x, a, b, c);
        lineSeries->append(x, y);
    }
    
    std::cout << "Generated " << numPoints << " points for fitted curve" << std::endl;
    
    // 将序列添加到图表
    chart->addSeries(scatterSeries);
    chart->addSeries(lineSeries);
    
    // 创建坐标轴
    QValueAxis *axisX = new QValueAxis();
    axisX->setTitleText("X");
    chart->addAxis(axisX, Qt::AlignBottom);
    
    QValueAxis *axisY = new QValueAxis();
    axisY->setTitleText("Y");
    chart->addAxis(axisY, Qt::AlignLeft);
    
    // 将序列与坐标轴关联
    scatterSeries->attachAxis(axisX);
    scatterSeries->attachAxis(axisY);
    lineSeries->attachAxis(axisX);
    lineSeries->attachAxis(axisY);
    
    // 设置图表为视图并显示
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    
    window->setCentralWidget(chartView);
    window->show();
    std::cout << "Visualization window should now be visible" << std::endl;
}

/**
 * @brief 显示使用帮助
 */
void showUsage() {
    std::cout << "Usage: visualization <data_file> <a> <b> <c>" << std::endl;
    std::cout << "Example: visualization data.txt 1.0 2.0 1.0" << std::endl;
    std::cout << "Or run without arguments to use default values" << std::endl;
}

int main(int argc, char *argv[]) {
    std::cout << "Starting visualization application..." << std::endl;
    
    // 检查命令行参数
    std::vector<double> x_data, y_data;
    double a = 1.0, b = 2.0, c = 1.0;
    
    if (argc == 1) {
        // 没有参数，使用默认示例数据
        std::cout << "Using default example data..." << std::endl;
        // 生成一些示例数据
        for (int i = 0; i <= 100; ++i) {
            double x = i / 100.0;
            x_data.push_back(x);
            // 添加一些噪声的示例数据，基于 y = exp(1.0*x^2 + 2.0*x + 1.0)
            y_data.push_back(std::exp(1.0 * x * x + 2.0 * x + 1.0) + (std::rand() % 100) / 100.0);
        }
        std::cout << "Generated " << x_data.size() << " sample data points" << std::endl;
    } else if (argc == 5) {
        // 有参数，读取文件和参数
        QString filename = argv[1];
        a = std::stod(argv[2]);
        b = std::stod(argv[3]);
        c = std::stod(argv[4]);
        
        std::cout << "Reading data from file: " << filename.toStdString() << std::endl;
        std::cout << "Using parameters: a=" << a << ", b=" << b << ", c=" << c << std::endl;
        
        if (!readDataFromFile(filename, x_data, y_data)) {
            std::cerr << "Failed to read data from file: " << filename.toStdString() << std::endl;
            return -1;
        }
    } else {
        showUsage();
        return -1;
    }
    
    // 创建Qt应用程序
    std::cout << "Creating QApplication..." << std::endl;
    QApplication app(argc, argv);
    
    // 显示可视化结果
    std::cout << "Showing visualization with parameters: a=" << a 
              << ", b=" << b << ", c=" << c << std::endl;
    visualizeCurveFitting(x_data, y_data, a, b, c);
    
    std::cout << "Starting event loop..." << std::endl;
    return app.exec();
}