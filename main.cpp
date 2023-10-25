#include <iostream>
#include "inc/teb_config.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include<boost/smart_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
//#include "/home/juchunyu/20231013/tebNoRos/teb_local_planner/src/matplotlibcpp.h"
#include "/home/juchunyu/20231013/tebNoRos/teb_local_planner/matplotlib-cpp/matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;//可视化

using namespace teb_local_planner;
using namespace std;

int main()
{
    // 参数配置
    TebConfig config;
    PoseSE2 start(-2,1,0);
    PoseSE2 end(10,1,0);

    double r = 0.4;
    
    //Obstacles
    std::vector<ObstaclePtr> obst_vector;
    //obst_vector.emplace_back(boost::make_shared<PointObstacle>(5,0.1));　//add Point障礙物
    //obst_vector.emplace_back(boost::make_shared<LineObstacle>(5.8,-5,5.8,5));　//add Point障礙物

    //add Line Obstacle
    vector<double> lineObs_x = {5,5};
    vector<double> lineObs_y = {1.0,1.2};
    obst_vector.emplace_back(boost::make_shared<LineObstacle>(lineObs_x[0],lineObs_y[0],lineObs_x[1],lineObs_y[1]));  

    //add PolyObstacle
    vector<double> PolyOb_x = {1,0,1,2};
    vector<double> PolyOb_y = {-1.4,-1.3,-1.5,-1.2};
    PolygonObstacle* polyobst2 = new PolygonObstacle;
    
    polyobst2->pushBackVertex(PolyOb_x[0], PolyOb_y[0]);
    polyobst2->pushBackVertex(PolyOb_x[1], PolyOb_y[1]);
    polyobst2->pushBackVertex(PolyOb_x[2], PolyOb_y[2]);
    polyobst2->pushBackVertex(PolyOb_x[3], PolyOb_y[3]);
    polyobst2->finalizePolygon();
    obst_vector.emplace_back(polyobst2);
    

    vector<double> PolyOb_x_1 = {0,1,2,3};
    vector<double> PolyOb_y_1 = {1.1,1.2,1.3,1.5};
    PolygonObstacle* polyobst = new PolygonObstacle;

    polyobst->pushBackVertex(PolyOb_x_1[0], PolyOb_y_1[0]);
    polyobst->pushBackVertex(PolyOb_x_1[1], PolyOb_y_1[1]);
    polyobst->pushBackVertex(PolyOb_x_1[2], PolyOb_y_1[2]);
    polyobst->pushBackVertex(PolyOb_x_1[3], PolyOb_y_1[3]);
    polyobst->finalizePolygon();
    obst_vector.emplace_back(polyobst);



    ViaPointContainer via_points;
    vector<double> via_points_x;
    vector<double> via_points_y;
   

    //add wayPoints
    for(int i = -2;i < 11;i++){
        via_points.push_back( Eigen::Vector2d( i, 1 ) );
        via_points_x.push_back(i);
        via_points_y.push_back(1);
        cout<<"viaPoints("<<i<<','<<1<<")"<<endl;
    }
    // Setup robot shape model
    RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(r);
    auto visual = TebVisualizationPtr(new TebVisualization(config));
    auto planner = new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points);
    //cv::Mat show_map = cv::Mat::zeros(cv::Size(500,500),CV_8UC3);

    // param
    int start_theta = 0;
    int end_theta = 0;
    float v_x = 0;
    float v_y = 0;
    float w   = 0;
    int look_ahead_poses = 1;
    vector<double> path_x;
    vector<double> path_y;

    vector<double> circle_x;
    vector<double> circle_y;

   // while (true)
   // {
           // start.theta() = start_theta * 0.1;
           // end.theta() = end_theta * 0.1;
            start.theta() = start_theta;
            end.theta()   = end_theta;
            planner->plan(start,end);
            std::vector<Eigen::Vector3f> path;
            planner->getFullTrajectory(path);
            planner->getVelocityCommand(v_x,v_y,w,look_ahead_poses);
            cout<<"速度指令v_ｘ:"<<v_x<<" "<<"速度指令ｖ_y:"<<v_y<<" " <<"w指令"<<w<<endl;
            cout<<"路徑長度："<<path.size()<<endl;
            for(int i = 0;i < path.size() - 1;i ++)
            {
                path_x.push_back(path.at(i)[0]);
                path_y.push_back(path.at(i)[1]);
                cout<<"("<<path.at(i)[0]<<","<<path.at(i)[1]<<")"<<endl;
            }
            
            plt::clf();
            
            vector<double> path_x_show_step_x;
            vector<double> path_y_show_step_y;
            for(int i = 0;i < path_x.size();i++){

                 
                plt::clf();
                std::map<std::string, std::string> keywords1;
                keywords1.insert(std::pair<std::string, std::string>("label", "TEB_Plan_Traj") );
                keywords1.insert(std::pair<std::string, std::string>("linewidth", "2.5") );
                //plt::plot(path_x,path_y,keywords1);
                path_x_show_step_x.push_back(path_x[i]);
                path_y_show_step_y.push_back(path_y[i]);
                plt::plot(path_x_show_step_x,path_y_show_step_y,keywords1);
                
                std::map<std::string, std::string> keywords;
                keywords.insert(std::pair<std::string, std::string>("label", "global_Plan_traj") );
                keywords.insert(std::pair<std::string, std::string>("linewidth", "1.8") );
                plt::plot(via_points_x,via_points_y,keywords);


                std::map<std::string, std::string> keywords2;
                keywords2.insert(std::pair<std::string, std::string>("label", "PolyObstacle") );
                keywords2.insert(std::pair<std::string, std::string>("linewidth", "2.5") );
                plt::plot(PolyOb_x,PolyOb_y);
                plt::fill(PolyOb_x,PolyOb_y,keywords2);

                plt::plot(PolyOb_x_1,PolyOb_y_1);
                plt::fill(PolyOb_x_1,PolyOb_y_1,keywords2);

                std::map<std::string, std::string> keywords3;
                keywords3.insert(std::pair<std::string, std::string>("label", "LIneObstacle") );
                keywords3.insert(std::pair<std::string, std::string>("linewidth", "3") );
                
                plt::plot(lineObs_x,lineObs_y,keywords3);
                //plt::scatter(ob_x,ob_y,50);


                //Plot Robot
                double pi = 3.14;
                double theta = 0;
                while(theta < 2*pi){
                //double x_temp = start.x ＋r*cos(theta);
                    double x_temp = path_x[i] + r * cos(theta);
                    double y_temp = path_y[i] + r * sin(theta);
                    circle_x.push_back(x_temp);
                    circle_y.push_back(y_temp);
                    theta = theta + 0.01;
                }

                std::map<std::string, std::string> keywords4;
                keywords4.insert(std::pair<std::string, std::string>("label", "RobotModel") );
                keywords4.insert(std::pair<std::string, std::string>("linewidth", "3") );
                
                plt::plot(circle_x,circle_y,keywords4);
                circle_x.clear();
                circle_y.clear();

                plt::xlabel("x");
                plt::ylabel("y");

                plt::xlim(-1, 13);
                plt::xlim(-5, 13);
                plt::title("Teb Plan Traj");
                plt::legend();
                //plt::show();
                plt::pause(0.01);
                if(i == path_x.size()-1){
                    plt::show();
                }
            }
            

    //}
    return 0;
}
