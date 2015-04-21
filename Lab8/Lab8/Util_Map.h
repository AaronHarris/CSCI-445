#ifndef Test_Util_Map2_h
#define Test_Util_Map2_h

#include <iostream>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "Particle2.h"
#include "Util2.h"
#include "math.h"


void parseFile(
               std::vector<cv::Point> &edges,
               std::vector<cv::Point> &rocket,
               std::vector<cv::Point> &dock,
               cv::Point &arcReactorLoc
               ) {
    std::ifstream myfile;
  
    myfile.open ("map_origin.txt");
        
    if (myfile.is_open()){
        while (!myfile.eof()){
            cv::Point p1;
            cv::Point p2;
            int type;// 0 is wall,1 is rocket, 2 is docking
            if (myfile >> p1.x >> p1.y >> p2.x >> p2.y >> type ) {
                p1.x = p1.x*25;//convert ft to cm
                p1.y = p1.y*25;//convert ft to cm
                p2.x = p2.x*25;//convert ft to cm
                p2.y = p2.y*25;//convert ft to cm
                if(type == 0) {
                    //wall
                    edges.push_back(p1);
                    edges.push_back(p2);
                }
                else if(type == 1){
                    rocket.push_back(p1);
                    rocket.push_back(p2);
                }
                else if(type ==2){
                    dock.push_back(p1);
                    dock.push_back(p2);
                    
                }else if(type == 3){
                    arcReactorLoc.x = p1.x;
                    arcReactorLoc.y = p1.y;
                    
                }
            }
        }
    }
    myfile.close();
}

void drawMap(cv::Mat& img,
             std::vector<cv::Point> &edges,
             std::vector<cv::Point> &rocket,
             std::vector<cv::Point> &docks,
             cv::Point &arcReactorLoc
             ) {
    img = cv::Scalar::all(0);

    parseFile(edges,rocket,docks,arcReactorLoc);
    
    int thickness = 6; //6
    int lineType = 8; //8
    
    //Draw Wall
    for(int i = 0;i < edges.size()-1; i+=2){
        cv::Point p1 = edges[i];
        cv::Point p2 = edges[i+1];
        line (img, p1, p2, cv::Scalar(255, 0, 0), thickness, lineType);
    }
}

void drawParticles(cv::Mat& img, std::vector<Particle>& particles){
                    for(vector<Particle>::iterator i = particles.begin(); i!= particles.end(); i++){
                        cv::circle(img, i->getPosition(), 3, cv::Scalar(0,255,255),i->getWeight(), 8, 0);
                    }
}

void drawOrientation(cv::Mat& image3,int orientation,int sonar_relative_orientation,cv::Point& robot){
        cv::Point a;
        a.x=robot.x+12*cos((orientation+sonar_relative_orientation)* M_PI/180.0);
        a.y=robot.y+12*sin((orientation+sonar_relative_orientation)* M_PI/180.0);
        line(image3,robot,a, cv::Scalar(255,0,255),6,8);

        a.x=robot.x+12*cos(orientation* M_PI/180.0);
        a.y=robot.y+12*sin(orientation* M_PI/180.0);
        line(image3,robot,a, cv::Scalar(0,255,255),6,8);
}

   
#endif
