#ifndef Test_Util2_h
#define Test_Util2_h

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "Particle2.h"

double getProbability(double u, double s, double x){
    double coefficient = 1.0/sqrt(2.0 * M_PI * s*s);    //s = Standard Deviation, s*s = Variance
    return coefficient * exp(-(x-u)*(x-u)/(2.0 * s*s));
}

double distToEdge(cv::Point pt1, cv::Point pt2) {
    double x = pt1.x - pt2.x;
    double y = pt1.y - pt2.y;
    return sqrt(pow(x, 2) + pow(y, 2));
}

void updateProbability(std::vector<Particle> &particles, std::vector<cv::Point> &particle_closest_edge_point, double distance) {
    float total_probabilities = 0.0;
    float new_weight = 0.0;
    float old_probabilities = 0.0;
    float new_probability = 0.0;
    double map_distance = 0.0;
    double sonar_variance = 30.0;
    
    // update all the particle probabilities
    for (int i=0; i<particles.size(); i++){
        cv::Point pos = particles[i].getPosition();
        
        // use heading to calculate the map distance from particle to wall.
        //map_distance =  distToEdge(direction,cv::Point(pos));
        map_distance = distToEdge(particles[i].getPosition(), particle_closest_edge_point[i]);
        
        // printf("Sonar: %f, Map_Distance: %f, direction: %d, Point(%d,%d), Weight: %f\n", distance, map_distance, direction, pos.i, pos.j, particles[i].getWeight());
        // Compute new probability using measured distance , map distance and sonar variance
        new_probability =  getProbability(distance, sonar_variance, map_distance); //distance by sonar report, sonar variance, given loaction
        
        // update each probability and compute total probabilities
        old_probabilities = particles[i].getWeight(); //P(robot@location)
        new_weight = old_probabilities * new_probability; //P(Sensor Reading| Robot @ Location) * P(robot@location)
        printf("particle %d; location x: %d y: %d, closest_edge.x: %d, closest_edge.y: %d, d2e: %f, old_probabilities: %f, new_probability: %f, new_weight; %f\n",i, pos.x,pos.y, particle_closest_edge_point[i].x,particle_closest_edge_point[i].y, map_distance, old_probabilities,new_probability, new_weight);
        particles[i].setWeight(new_weight);
        total_probabilities += new_weight; //Ex: 0.25 + 0.25 + 0.3 = 0.8, so N = 1/0.8, here total_probabilities = 1/N
        //cout << "total_probabilities = " << total_probabilities << endl;
    }
    
    // Normalize all probabilities
    for (int i=0; i<particles.size(); i++){
        //normalized probability = probability / total probabilities
        particles[i].setWeight(particles[i].getWeight()/total_probabilities); //0.25/0.8 + 0.25/0.8 + 0.3/0.8 = 1
    }
}

std::vector<Particle> resampleParticles(std::vector<Particle>& oldParticles) {
    
    std::vector<Particle> newParticles;
    
    //Calculate a Cumulative Distribution Function for our particle weights
    std::vector<double> CDF;
    CDF.resize(oldParticles.size());
    CDF[0] = ((Particle)oldParticles[0]).getWeight();
    
    for(int i=1; i<CDF.size(); i++)
        CDF[i] = CDF[i-1] + oldParticles[i].getWeight();
    //Loop through our particles as if spinning a roulette wheel.
    //The random u that we start with determines the initial offset
    //and each particle will have a probability of getting landed on and
    //saved proportional to its posterior probability. If a particle has a very large
    //posterior, then it may get landed on many times. If a particle has a very low
    //posterior, then it may not get landed on at all. By incrementing by
    // 1/(numParticles) we ensure that we don't change the number of particles in our
    // returned set.
    
    int i = 0;
    double u = randomDouble()* 1.0/double(oldParticles.size());
    //double u = 1.0/double(oldParticles.size());
    
    //Resampling: pick 15 particles with replacement from the original particles with probabilities proportional to their weight:
    // 
    for(int j=0; j < oldParticles.size(); j++){
        while(u > CDF[i]) //if i particle is very small, we don't want to let it in newparticle, so i++
            i++;
        
        Particle p = oldParticles[i]; //leave possible particles
        p.setWeight(1.0/double(oldParticles.size()));
        newParticles.push_back(p);
        
        u += 1.0/double(oldParticles.size());
    }
    
    return newParticles;
}

#endif
