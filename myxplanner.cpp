
#include "myxplanner.h"

template <typename T>
T RandT(T _min, T _max)
{
  T temp;
  if (_min > _max)
  {
    temp = _max;
    _max = _min;
    _min = temp;
  }
  return rand() / (double)RAND_MAX * (_max - _min) + _min;
}

bool isin(geometry_msgs::Pose point, float x[2], float y[2], float z[2])
{
  bool flag = 0;
  if (point.position.x < x[1] && point.position.x > x[0])
  {
    if (point.position.y < y[1] && point.position.y > y[0])
    {
      if (point.position.z < z[1] && point.position.z > z[0])
        flag = 1;
    }
  }
  return flag;
}

void solve(geometry_msgs::Pose start, geometry_msgs::Pose goal, std::vector<geometry_msgs::Pose> waypoints)
{
  float wsbound_x[2] = {0, 0.9};
  float wsbound_y[2] = {0, 0.9};
  float wsbound_z[2] = {0, 1.3};

  float table1_x[2] = {0.5, 0.9};
  float table1_y[2] = {0, 0.2};
  float table1_z[2] = {0, 0.4};

  float table2_x[2] = {0, 0.2};
  float table2_y[2] = {0.5, 0.9};
  float table2_z[2] = {0, 0.4};

  float collision_x[2] = {0.3, 0.5};
  float collision_y[2] = {0.3, 0.5};
  float collision_z[2] = {0.7, 0.9};

  float start_state[3];
  start_state[0] = start.position.x;
  start_state[1] = start.position.y;
  start_state[2] = start.position.z;

  float goal_state[3];
  goal_state[0] = goal.position.x;
  goal_state[1] = goal.position.y;
  goal_state[2] = goal.position.z;

  while (1)
  {
    float goal_bias = 0.05;
    float randnum = RandT(0, 1);
    bool flag = 1;
    if (randnum > 0.05)
    {
      while (flag)
      {
        geometry_msgs::Pose point;
        point.position.x = RandT(wsbound_x[0], wsbound_x[1]);
        point.position.y = RandT(wsbound_y[0], wsbound_y[1]);
        point.position.z = RandT(wsbound_z[0], wsbound_z[1]);

        if (isin(point, wsbound_x, wsbound_y, wsbound_z) || isin(point, table1_x, table1_y, table1_z) || isin(point, table2_x, table2_y, table2_z) || isin(point, collision_x, collision_y, collision_z))
          flag = 1;
        else
          flag = 0;
      }
    }
    else
    {
      geometry_msgs::Pose point;
      
    }
    
  }
}