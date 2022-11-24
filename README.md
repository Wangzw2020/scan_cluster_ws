# scan_cluster_ws

points_cutter 用前后左右进行点云范围选取，将scan msg转为vector<point>

DBSCAN 输入为二维 vector<point> 输出聚类结果位置与半径

使用：
roslaunch scan_cluster scan_cluster.launch
