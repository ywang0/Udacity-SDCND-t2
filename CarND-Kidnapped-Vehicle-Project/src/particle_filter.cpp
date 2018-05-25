#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <random>
#include <map>

#include "particle_filter.h"

using namespace std;

#define EPS 1e-6
#define MAX_DOUBLE numeric_limits<double>::max()
#define MIN_DOUBLE numeric_limits<double>::min()

void ParticleFilter::init(double x, double y, double theta, double std[]) {

  num_particles = 10;

  random_device rd;
  default_random_engine gen(rd());

  for (unsigned int i = 0; i < num_particles; ++i) {
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    Particle p;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  random_device rd;
  default_random_engine gen(rd());

  for (Particle &p : particles) {
    double new_x, new_y, new_theta;
    double theta = p.theta;
    double v_over_yawrate = velocity / yaw_rate;

    if (fabs(yaw_rate) < EPS) {
      new_theta = theta;
      new_x = p.x + velocity * delta_t * cos(theta);
      new_y = p.y + velocity * delta_t * sin(theta);
    }
    else {
      new_theta = theta + yaw_rate * delta_t;
      new_x = p.x + v_over_yawrate * (sin(new_theta) - sin(theta));
      new_y = p.y + v_over_yawrate * (cos(theta) - cos(new_theta));
    }

    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_theta, std_pos[2]);

    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
	//   implement this method and use it as a helper during the updateWeights phase.

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

  // define constants used in calculating Multivariate Guassian distribution
  double denom = 1 / (2*M_PI * std_landmark[0] * std_landmark[1]);
  double varx_sqr_2 = 2. *  std_landmark[0] * std_landmark[0];
  double vary_sqr_2 = 2. *  std_landmark[1] * std_landmark[1];

  // https://discussions.udacity.com/t/c-help-with-dataassociation-method/291220/5
  for (Particle &p : particles) {
    double posterior = 1.0;

    // transform observations in particle frame to map frame
    for (LandmarkObs observ : observations) {
      LandmarkObs observ_t;
      double cos_theta = cos(p.theta);
      double sin_theta = sin(p.theta);

      observ_t.x = observ.x * cos_theta - observ.y * sin_theta + p.x;
      observ_t.y = observ.x * sin_theta + observ.y * cos_theta + p.y;

      // Find nearest landmark for each observation and update particle's weight:
      // (here we allow each landmark associates with more than one observation)
      // 1. for each observation, find nearest landmark and calculate Multivariate Gaussian
      // 2. if no landmark within sensor range, set probability for the observation to be a minimum number

      vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
      double min_dist = MAX_DOUBLE;
      int lm_idx = -1;

      for (unsigned int i = 0; i < landmarks.size(); ++i) {
        double dst = dist(landmarks[i].x_f, landmarks[i].y_f, observ_t.x, observ_t.y);
        if (dst <= sensor_range && dst < min_dist) {
          lm_idx = i;
          min_dist = dst;
        }
      }

      if (lm_idx >= 0) {  // a valid landmark is found
        // calculate multivairate Gaussian distribution of the observation given the associated landmark
        double x_diff = observ_t.x - landmarks[lm_idx].x_f;
        double y_diff = observ_t.y - landmarks[lm_idx].y_f;
        double prob = denom * exp(-(x_diff*x_diff / varx_sqr_2 + y_diff*y_diff / vary_sqr_2));
        // update posterior
        posterior *= prob;
      }
      else {
        double prob = MIN_DOUBLE;
        posterior *= prob;
      }
    }

    p.weight = posterior;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.

  vector<double> weights;

  for (Particle p : particles) {
    weights.push_back(p.weight);
  }

  vector<Particle> new_particles;

  // http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::random_device rd;
  std::mt19937 gen(rd());
  std::discrete_distribution<> d(weights.begin(), weights.end());
  std::map<int, int> m;

  for(int i = 0; i < num_particles; ++i) {
    Particle particle_res = particles[d(gen)];
    new_particles.push_back(particle_res);
  }

  particles = new_particles;

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

    return particle;
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
