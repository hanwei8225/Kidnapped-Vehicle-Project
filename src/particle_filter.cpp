/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::default_random_engine;
using std::normal_distribution;
using std::string;
using std::uniform_real_distribution;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 1000; // TODO: Set the number of particles

  //gaussion
  static default_random_engine e;

  normal_distribution<double> gx(0, std[0]);
  normal_distribution<double> gy(0, std[1]);
  normal_distribution<double> gt(0, std[2]);

  for (int i = 0; i < num_particles; i++)
  {
    Particle p;
    p.id = i;
    p.x = x;
    p.y = y;
    p.theta = theta;

    p.weight = 1.0;

    //add noise
    p.x += gx(e);
    p.y += gy(e);
    p.theta += gt(e);
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */
  //gaussion
  static default_random_engine e;
  normal_distribution<double> gx(0, std_pos[0]);
  normal_distribution<double> gy(0, std_pos[1]);
  normal_distribution<double> gt(0, std_pos[2]);

  for (int i = 0; i < num_particles; i++)
  {
    Particle p = particles[i];

    if (fabs(yaw_rate) < 0.0001)
    {
      p.x = p.x + velocity * delta_t * cos(p.theta);
      p.y = p.y + velocity * delta_t * sin(p.theta);
    }
    else
    {
      p.x = p.x + velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y = p.y + velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
    }
    p.theta = p.theta + delta_t * yaw_rate;

    //add noise
    p.x += gx(e);
    p.y += gy(e);
    p.theta += gt(e);

    particles[i] = p;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  //每一个地图上的参考点坐标，都从测量点中找到与之最接近的一个，将两个对齐

  vector<LandmarkObs> new_observations;

  for (int i = 0; i < predicted.size(); i++)
  {
    double min_len = dist(predicted[i].x, predicted[i].y, observations[0].x, observations[0].y);
    int index = 0;
    for (int j = 0; j < observations.size(); j++)
    {
      double len = dist(predicted[i].x, predicted[i].y, observations[j].x, observations[j].y);
      if (len < min_len)
      {
        min_len = len;
        index = j;
      }
    }
    new_observations[i] = observations[index];
  }
  observations = new_observations;
}

void ParticleFilter::updateWeights(double sensor_range,
                                   double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];
  for (int i = 0; i < num_particles; i++)
  {
    //对于每一个粒子
    Particle p = particles[i];

    //只考虑在传感器测量范围内的地图参考点作为点集，为地图坐标,计算方形比计算圆形快很多
    vector<LandmarkObs> predictions;
    for (int k = 0; k < map_landmarks.landmark_list.size(); k++)
    {
      Map::single_landmark_s m = map_landmarks.landmark_list[k];
      if ((fabs(p.x - m.x_f) < sensor_range) && (fabs(p.y - m.y_f) < sensor_range))
      {
        LandmarkObs pre;
        pre.id = m.id_i;
        pre.x = m.x_f;
        pre.y = m.y_f;
        predictions.push_back(pre);
      }
    }

    //转换每一个测量到的参考点的坐标，从汽车坐标系转换为地图坐标系
    vector<LandmarkObs> observations_map;
    for (int j = 0; j < observations.size(); j++)
    {
      LandmarkObs obs = observations[j];
      double x_map = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
      double y_map = p.y + (sin(p.theta) * obs.x) - (cos(p.theta) * obs.y);
      observations_map.push_back(LandmarkObs{obs.id,x_map,y_map});

    }

    //匹配测试数据属于哪个地标
    dataAssociation(predictions,observations_map);

    //计算weight
    for (int j = 0; j < predictions.size(); j++)
    {
      LandmarkObs pred = predictions[j];
      LandmarkObs obs = observations_map[j];
      double pxy = 0.5 * M_PI*(sig_x*sig_y)*exp(-((((obs.x -pred.x) * (obs.x -pred.x)) / (2 * (sig_x * sig_x)))
                                              + (((obs.y -pred.y) * (obs.y -pred.y)) / (2 * (sig_y * sig_y)))));
      particles[i].weight *= pxy;
    }
    




    // double pxy = 0.5 * M_PI*(sig_x*sig_y)*exp(-((((obs.x -p.x) * (obs.x -p.x)) / (2 * (sig_x * sig_x)))
    //                                               + (((obs.y -p.y) * (obs.y -p.y)) / (2 * (sig_y * sig_y)))));
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  //获取所有粒子的权重，放入向量
  vector<double> weights;
  for (int i = 0; i < num_particles; i++)
  {
    weights.push_back(particles[i].weight);
  }
  //根据权重向量生成随机的索引
  default_random_engine e;
  std::discrete_distribution<int> index (weights.begin(),weights.end());

  //重采样粒子
  vector<Particle> new_particles;
  for (int i = 0; i < num_particles; i++)
  {
    new_particles[i] = particles[index(e)];
  }
  particles = new_particles;

}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}