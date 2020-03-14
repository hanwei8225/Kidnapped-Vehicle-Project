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
  num_particles = 100; // TODO: Set the number of particles

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

  //每一个地图上的参考点坐标，都从测量点中找到与之最接近的一个，将两个对齐,使用索引对齐
  //******************待发现错误，暂时跑不通
  // vector<LandmarkObs> new_observations;

  // for (unsigned int i = 0; i < predicted.size(); i++)
  // {
  //   double min_len = std::numeric_limits<double>::max();
  //   //  double min_len = 0;
  //   int index = -1;
  //   for (int j = 0; j < observations.size(); j++)
  //   {
  //     double len = dist(predicted[i].x, predicted[i].y, observations[j].x, observations[j].y);
  //     if (len < min_len)
  //     {
  //       min_len = len;
  //       index = j;
  //     }
  //   }
  //   new_observations.push_back( observations[index]);
  // }

  // std::cout << "observations:" << observations.size() << std::endl;
  // std::cout << "new_observations:" << new_observations.size() << std::endl;
  // observations = new_observations;

  // 每一个地图上的参考点坐标，都从测量点中找到与之最接近的一个，将两个对齐,  使用ID对其
  for (unsigned int i = 0; i < observations.size(); i++)
  {
    //将距离最大化
    double min_len = std::numeric_limits<double>::max();
    int id = -1;
    // 找到最接近的点的id
    for (int j = 0; j < particles.size(); j++)
    {
      double len = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      // std::cout << "len:" << len << std::endl;
      if (len < min_len)
      {
        min_len = len;
        id = predicted[j].id;
      }
    }
    observations[i].id = id;
  }
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
      if ((fabs(p.x - m.x_f) <= sensor_range) && (fabs(p.y - m.y_f) <= sensor_range))
      {

        predictions.push_back(LandmarkObs{m.id_i, m.x_f, m.y_f});
      }
    }

    //转换每一个测量到的参考点的坐标，从汽车坐标系转换为地图坐标系
    vector<LandmarkObs> observations_map;
    for (unsigned int j = 0; j < observations.size(); j++)
    {
      double x_map = p.x + cos(p.theta) * observations[j].x - sin(p.theta) * observations[j].y;
      double y_map = p.y + sin(p.theta) * observations[j].x + cos(p.theta) * observations[j].y;
      observations_map.push_back(LandmarkObs{observations[j].id, x_map, y_map});
    }

    // std::cout << "observations_map_pre:" << observations_map.size() << std::endl;
    // for (size_t k = 0; k < observations_map.size(); k++)
    // {
    //   std::cout << "observations_map_pre: x=" <<observations_map[k].x << "  y= "<<observations_map[k].y << "  id= "<<observations_map[k].id<< std::endl;
    // }

    //匹配测试数据属于哪个地标
    dataAssociation(predictions, observations_map);
    // std::cout << "predictions:" << predictions.size() << std::endl;

    // for (size_t k = 0; k < predictions.size(); k++)
    // {
    //   std::cout << "predictions: x=" << predictions[k].x << "  y= "<<predictions[k].y << "  id= "<<predictions[k].id<< std::endl;
    // }

    // std::cout << "observations_map:" << observations_map.size() << std::endl;
    // for (size_t k = 0; k < observations_map.size(); k++)
    // {
    //   std::cout << "observations_map: x=" <<observations_map[k].x << "  y= "<<observations_map[k].y << "  id= "<<observations_map[k].id<< std::endl;
    // }

    //将权重重新初始化，不积累，否则最终将归零
    particles[i].weight = 1.0;

    //计算weight

    for (int j = 0; j < observations_map.size(); j++)
    {

      /*
    这种方法是使用ID对齐，所以需要进行找ID的行为，需要每次进行一次循环，对应的数据处理是使用ID对齐
    */
      LandmarkObs obs = observations_map[j];
      LandmarkObs pred;
      for (unsigned k = 0; k < predictions.size(); k++)
      {
        if (predictions[k].id == obs.id)
        {
          pred = predictions[k];
        }
      }

      /*
    这种方法是使用索引对齐，不需要进行循环
    */
      //  LandmarkObs pred = predictions[j];
      //  LandmarkObs obs = observations_map[j];

      double pxy = (0.5 / (M_PI * sig_x * sig_y)) * exp(-(pow(pred.x - obs.x, 2) / (2 * pow(sig_x, 2)) + pow(pred.y - obs.y, 2) / (2 * pow(sig_y, 2))));
      // std::cout << "pxy=" << pxy << std::endl;
      particles[i].weight *= pxy;
    }
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
  std::discrete_distribution<int> index(weights.begin(), weights.end());

  //重采样粒子
  vector<Particle> new_particles;
  for (int i = 0; i < num_particles; i++)
  {
    new_particles.push_back(particles[index(e)]);
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