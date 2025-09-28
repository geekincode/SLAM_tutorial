#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sba/types_sba.h>  // 新增包含SBA类型头文件
#include <chrono>

using namespace std;
using namespace cv;

void find_feature_matches(const Mat& img_1, const Mat& img_2,
                          vector<KeyPoint>& keypoints_1,
                          vector<KeyPoint>& keypoints_2,
                          vector<DMatch>& matches);

Point2d pixel2cam(const Point2d& p, const Mat& K);

void bundleAdjustment(const vector<Point3f> points_3d,
                      const vector<Point2f> points_2d,
                      const Mat& K,
                      Mat& R, Mat& t);

int main(int argc, char** argv) {
    if (argc != 5) {
        cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    
    // 修改为正确使用IMREAD标志
    Mat img_1 = imread(argv[1], IMREAD_COLOR);
    Mat img_2 = imread(argv[2], IMREAD_COLOR);
    Mat d1 = imread(argv[3], IMREAD_UNCHANGED);

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
    cout << "一共找到了" << matches.size() << "组匹配点" << endl;

    // 建立3D点
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    
    for (DMatch m : matches) {
        ushort d = d1.ptr<unsigned short>(
            int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
            
        if (d == 0) continue;
        
        float dd = d / 5000.0;
        Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
        pts_3d.push_back(Point3f(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(keypoints_2[m.trainIdx].pt);
    }

    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    Mat r, t;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false);
    Mat R;
    Rodrigues(r, R);

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;
    cout << "calling bundle adjustment" << endl;

    bundleAdjustment(pts_3d, pts_2d, K, R, t);
    return 0;
}

void find_feature_matches(const Mat& img_1, const Mat& img_2,
                          vector<KeyPoint>& keypoints_1,
                          vector<KeyPoint>& keypoints_2,
                          vector<DMatch>& matches) {
    // 使用现代OpenCV API
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    Mat descriptors_1, descriptors_2;
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);

    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);

    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        min_dist = min(min_dist, dist);
        max_dist = max(max_dist, dist);
    }

    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam(const Point2d& p, const Mat& K) {
    return Point2d(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

void bundleAdjustment(const vector<Point3f> points_3d,
                      const vector<Point2f> points_2d,
                      const Mat& K,
                      Mat& R, Mat& t) {
    // 使用unique_ptr管理内存
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;
    using LinearSolverType = g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>;
    
    auto linearSolver = new LinearSolverType();
    auto solver_ptr = new BlockSolverType(std::unique_ptr<LinearSolverType>(linearSolver));
    auto solver = new g2o::OptimizationAlgorithmLevenberg(std::unique_ptr<BlockSolverType>(solver_ptr));
    
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // 相机位姿顶点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;
    R_mat << 
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
    
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
        R_mat,
        Eigen::Vector3d(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0))
    ));
    optimizer.addVertex(pose);

    // 路标点顶点
    for (size_t i = 0; i < points_3d.size(); ++i) {
        g2o::VertexPointXYZ* point = new g2o::VertexPointXYZ();
        point->setId(i + 1);
        point->setEstimate(Eigen::Vector3d(
            points_3d[i].x, points_3d[i].y, points_3d[i].z
        ));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    // 相机参数
    g2o::CameraParameters* camera = new g2o::CameraParameters(
        K.at<double>(0, 0), 
        Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)),
        0
    );
    camera->setId(0);
    optimizer.addParameter(camera);

    // 边定义
    for (size_t i = 0; i < points_2d.size(); ++i) {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();  // 确保 edge 被正确定义
        edge->setId(i+1); 
        edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(i + 1)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(points_2d[i].x, points_2d[i].y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    // 优化
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}