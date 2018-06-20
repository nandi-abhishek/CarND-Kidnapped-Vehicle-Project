/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	is_initialized = true;

	default_random_engine gen;
	// These lines create a normal (Gaussian) distribution for x,y and theta
	normal_distribution<double> dist_x(0, std[0]);
	normal_distribution<double> dist_y(0, std[1]);
	normal_distribution<double> dist_theta(0, std[2]);
	
	for (int i = 0 ; i < num_particles; i++) {
		Particle p;
		p.id = i;
		p.x = x;
		p.y = y;
		p.theta = theta;
		p.weight = 1;
		weights.push_back(p.weight);

		p.x += dist_x(gen);
		p.y += dist_y(gen);
		p.theta += dist_theta(gen);
		particles.push_back(p);
	}	
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;
	// These lines create a normal (Gaussian) distribution for x,y and theta
	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0, std_pos[2]);
	
	for (auto &particle : particles) {
        //avoid division by zero
    	if (fabs(yaw_rate) > 0.00001) {
        	particle.x += velocity/yaw_rate * ( sin (particle.theta + yaw_rate * delta_t) - sin(particle.theta));
        	particle.y += velocity/yaw_rate * ( cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t) );
			particle.theta += yaw_rate * delta_t;
    	}
    	else {
			particle.x += velocity * delta_t * cos(particle.theta);
        	particle.y += velocity * delta_t * sin(particle.theta);
    	}
		particle.x += dist_x(gen);
		particle.y += dist_y(gen);
		particle.theta += dist_theta(gen);
		//particle.theta = fmod(particle.theta, (2* M_PI));
	}


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (auto &obs : observations) {
		double min_dist = numeric_limits<double>::max(); ;
		int min_id = -1;
		for (auto &landmark : predicted) {
			double cdist = dist(obs.x, obs.y, landmark.x, landmark.y);
			if (cdist < min_dist) {
				min_dist = cdist;
				min_id = landmark.id;
			}
		}
		obs.id = min_id;
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	
	double gauss_norm = (1/(2 * M_PI * std_landmark[0] * std_landmark[1]));
	double exp_x_denom = 2 * pow(std_landmark[0], 2);
	double exp_y_denom = 2 * pow(std_landmark[1], 2);
	std::vector<double> updated_weights;	
  	for (auto &particle : particles) {
	//   1. TRANSFORM
		std::vector<LandmarkObs> trans_observations;
    	for (auto &obs : observations) {
			double x = obs.x * cos(particle.theta) - obs.y * sin(particle.theta) + particle.x;
			double y = obs.x * sin(particle.theta) + obs.y * cos(particle.theta) + particle.y;
			LandmarkObs trans_obs;
			trans_obs.x = x;
			trans_obs.y = y;
			trans_observations.push_back(trans_obs);
		}	 
	//   2. ASSOCIATE
		std::vector<LandmarkObs> predicted;
		for (auto &map_obj : map_landmarks.landmark_list) {
			LandmarkObs obs;
			obs.x = map_obj.x_f;
			obs.y = map_obj.y_f;
			obs.id = map_obj.id_i;
			if (fabs(obs.x - particle.x) <= sensor_range && fabs(obs.y - particle.y) <= sensor_range) {
				predicted.push_back(obs);
			}
		}		
		dataAssociation(predicted, trans_observations);
		std::vector<double> sense_x;
		std::vector<double> sense_y;
		std::vector<int> associations;
		for (auto &obs : trans_observations) {	
			sense_x.push_back(obs.x);
			sense_y.push_back(obs.y);
			associations.push_back(obs.id);
		}	
		SetAssociations(particle, associations, sense_x, sense_y);
	//   3. UPDATE WEIGHTS
	    double weight = 1;
	    for (auto & obs : trans_observations) {
			double x = obs.x;
			double y = obs.y;
			for (auto &predict_obs : predicted) {
        		if (predict_obs.id == obs.id) {
					double mu_x = predict_obs.x;
					double mu_y = predict_obs.y;
					double exponent= (pow((x - mu_x), 2))/ exp_x_denom + (pow((y - mu_y), 2))/exp_y_denom;
					weight *= gauss_norm * exp(-exponent);	
					break;
        		}
        	}
		}	
		particle.weight = weight;
		updated_weights.push_back(weight);
	}
	weights = updated_weights;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<Particle> new_particles(num_particles);	
#if 0
	uniform_int_distribution<int> uniintdist(0, num_particles-1);
    auto index = uniintdist(gen);
    double beta = 0.0;
    double mw = *max_element(weights.begin(), weights.end());
	uniform_real_distribution<double> unirealdist(0.0, 2 * mw);
    for (int i = 0; i < num_particles; i++) {
        beta += unirealdist(gen); // * 2.0;
        while (beta > weights[index]) {
        	beta -= weights[index];
        	index = (index + 1) % num_particles;
		}		
        new_particles[i] = particles[index];
	}
#endif		
	// Use discrete distribution to return particles by weight
  	random_device rd;
  	default_random_engine gen(rd());
  	for (int i = 0; i < num_particles; ++i) {
    	discrete_distribution<int> index(weights.begin(), weights.end());
    	new_particles[i] = particles[index(gen)];
  	}		
	particles = new_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
