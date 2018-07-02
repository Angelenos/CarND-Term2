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

using namespace std;

ParticleFilter::ParticleFilter(int num) {
	// Constructor for Particle Filter when number of particles specified
	num_particles = num;
	particles.resize(num_particles);
	// Assign corresponding id to all particles
	for (int i = 0; i < num_particles; i++) {
		particles[i].id = i;
	}
	// Set is_initialized to false
	is_initialized = false;
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	default_random_engine gen;
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);

	for (auto part_it = particles.begin(); part_it != particles.end(); part_it++) {
		part_it->x = dist_x(gen);
		part_it->y = dist_y(gen);
		part_it->theta = dist_theta(gen);
		part_it->weight = 1.0;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	default_random_engine gen;

	for (auto part_it = particles.begin(); part_it != particles.end(); part_it++) {
		double x_f = part_it->x + (velocity / yaw_rate * (sin(part_it->theta + (yaw_rate * delta_t)) - sin(part_it->theta)));
		double y_f = part_it->y + (velocity / yaw_rate * (cos(part_it->theta) - cos(part_it->theta + (yaw_rate * delta_t))));
		double theta_f = part_it->theta + (delta_t * yaw_rate);

		normal_distribution<double> pos_x(x_f, std_pos[0]);
		normal_distribution<double> pos_y(y_f, std_pos[1]);
		normal_distribution<double> pos_theta(theta_f, std_pos[2]);

		part_it->x = pos_x(gen);
		part_it->y = pos_y(gen);
		part_it->theta = pos_theta(gen);
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
	for (auto obs_it = observations.begin(); obs_it != observations.end(); obs_it++) {
		double x_p = obs_it->x;
		double y_p = obs_it->y;
		double dist_min = dist(predicted[0].x, predicted[0].y, x_p, y_p);
		int ind_min = 0;
		for (auto pred_it = predicted.begin(); pred_it != predicted.end(); pred_it++) {
			double distance = dist(pred_it->x, pred_it->y, x_p, y_p);
			if (distance < dist_min) {
				dist_min = distance;
				ind_min = pred_it - predicted.begin();
			}
		}
		// Here we use the field id in struct LandmarkObs to store the index of the associated land marks 
		// within predicted vector
		obs_it->id = ind_min;  
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
	double w_tot = 0.0; // Sum of weights for all particles before normalization
	double sigma_x = std_landmark[0];
	double sigma_y = std_landmark[1];
	// Loop over all particles
	for (auto part_it = particles.begin(); part_it != particles.end(); part_it++) {
		double x_p = part_it->x;
		double y_p = part_it->y;
		double theta = part_it->theta;
		// Create vector of predicted landmarkobs containing all landmarkobs within the range of sensor
		vector<LandmarkObs> predicted;
		for (auto map_it = map_landmarks.landmark_list.begin(); map_it != map_landmarks.landmark_list.end(); map_it++) {
			if (dist(x_p, y_p, map_it->x_f, map_it->y_f) <= sensor_range) {
				LandmarkObs obs_inrange;
				obs_inrange.x = map_it->x_f;
				obs_inrange.y = map_it->y_f;
				obs_inrange.id = map_it->id_i;
				predicted.push_back(obs_inrange);
			}
		}
		// Convert the observation to map coordinates
		vector<LandmarkObs> observations_map(observations);
		for (auto obs_it = observations.begin(); obs_it != observations.end(); obs_it++) {
			double x_c = obs_it->x;
			double y_c = obs_it->y;
			auto obs_map_it = observations_map.begin() + (obs_it - observations.begin());
			obs_map_it->x = x_p + (cos(theta) * x_c) - (sin(theta) * y_c);
			obs_map_it->y = y_p + (sin(theta) * x_c) + (cos(theta) * y_c);
			obs_map_it->id = obs_it->id;
		}
		// Perform data association
		if (!observations.empty()) {
			dataAssociation(predicted, observations_map);
			// Calculate weights
			part_it->weight = 1.0;
			for (auto obs_it = observations_map.begin(); obs_it != observations_map.end(); obs_it++) {
				double x_c = obs_it->x;
				double y_c = obs_it->y;
				double mu_x = predicted[obs_it->id].x;
				double mu_y = predicted[obs_it->id].y;
				double exponent = ((x_c - mu_x) * (x_c - mu_x) / (2 * sigma_x * sigma_x)) + ((y_c - mu_y) * (y_c - mu_y) / (2 * sigma_y * sigma_y));
				part_it->weight *= exp(-exponent) / 2 * M_PI * sigma_x * sigma_y;
			}
		}
		// Update the total weights for all particles
		w_tot += part_it->weight;
	}
	// Normalize the weight
	for (auto part_it = particles.begin(); part_it != particles.end(); part_it++) {
		part_it->weight /= w_tot;
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	vector<double> weights;
	for (auto part_it = particles.begin(); part_it != particles.end(); part_it++) {
		weights.push_back(part_it->weight);
	}
	discrete_distribution<int> resample_wheel(weights.begin(), weights.end());
	vector<Particle> resample(particles);

	for (int i = 0; i < num_particles; i++) {
		int index = resample_wheel(gen);
		resample[i] = particles[index];
		resample[i].id = i;
	}

	particles = resample;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
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
