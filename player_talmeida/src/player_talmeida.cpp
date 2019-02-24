#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <vector>

using namespace std; // ja na e preciso usar o std
using namespace ros;

float randomizePosition()
{
  srand(153234 * time(NULL)); // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

float randomizePosition2()
{
  srand(72323 * time(NULL)); // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace talmeida_ns // namespace talmeida_ns
{
class Team
{
public:
  string team_name;
  vector<string> player_names; // lista
  ros::NodeHandle n;
  Team(string team_name_in)
  {
    team_name = team_name_in;
    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team" << team_name << " has players: " << endl;
    for (size_t i = 0; i < player_names.size(); i++)
    {
      cout << player_names[i] << endl;
    }
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t ii = 0; ii < player_names.size(); ii++)
    {
      if (player_names[ii] == player_name)
      {
        return true;
      }
    }

    return false;
  }
};

class Player
{
public:
  // properties:
  string player_name;

  // string team_name;

  // constructor
  Player(string player_name_in)
  {
    player_name = player_name_in;
  }

  void setTeamName(string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else
    {
      cout << "Cannot set team name to " << team_name_in << endl;
    }
  }
  // overload:
  void setTeamName(int team_index)
  {
    if (team_index == 0)
    {
      setTeamName("red");
    }
    else if (team_index == 1)
    {
      setTeamName("green");
    }
    else if (team_index == 2)
    {
      setTeamName("blue");
    }
  }

  string getTeamName()
  {
    return team_name;
  };

private:
  string team_name; // assim nao da para mudar
};

class MyPlayer : public Player // herda tudo da class player
{
public:
  // ponteiros para classes especiais teams
  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_green;

  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vis_pub;

  MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) // construtor da class
  {
    setTeamName(team_name_in);
    team_red = (boost::shared_ptr<Team>)new Team("red"); // mallock
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

    ros::NodeHandle n;
    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("bocas", 0);

    if (team_red->playerBelongsToTeam(player_name_in))
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }
    else if (team_green->playerBelongsToTeam(player_name_in))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }

    else if (team_blue->playerBelongsToTeam(player_name_in))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "something wrong with team parametrization!!" << endl;
    }

    // define initial position
    float sx = randomizePosition();
    float sy = randomizePosition2();
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(sx, sy, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, M_PI);
    T1.setRotation(q);

    // define global movement
    tf::Transform Tglobal = T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
    ros::Duration(0.1).sleep();
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));
    printInfo();
  }

  void printInfo()
  {
    // cout << "debugger" << endl;
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name);
    ROS_INFO_STREAM("I am hunting " << team_preys->team_name << " and fleeing from " << team_hunters->team_name);
  }
  // Quando eu publico esta callback é chamada:
  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new msg");
    // publicar uma transformacao:

    // STEP 1: find out where I am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("/world", player_name, ros::Time(0), T0); // ros::time(0)-extrapolacao
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    // STEP 2: define where I want to move
    vector<float> distance_to_preys;
    vector<float> angle_to_preys;
    vector<float> distance_to_hunters;
    vector<float> angle_to_hunters;

//For each prey find the closest.Then follow it
#if 1
    for (size_t i = 0; i < team_preys->player_names.size(); i++)
    {
      ROS_WARN_STREAM("team_preys = " << team_preys->player_names[i]);

      std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_preys->player_names[i]);
      std::tuple<float, float> t2 = getDistanceAndAngleToPlayer(team_hunters->player_names[i]);
      distance_to_preys.push_back(std::get<0>(t));
      angle_to_preys.push_back(std::get<1>(t));
      distance_to_hunters.push_back(std::get<0>(t2));
      angle_to_hunters.push_back(std::get<1>(t2));
    }

    int idx_closest_prey = 0;
    int idx_closest_hunter = 0;
    float distance_closest_prey = 1000;
    float distance_closest_hunter = 1000;
    for (size_t i = 0; i < distance_to_preys.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_prey)
      {
        idx_closest_prey = i;
        distance_closest_prey = distance_to_preys[i];
      }
    }

    for (size_t ii = 0; ii < distance_to_hunters.size(); ii++)
    {
      if (distance_to_hunters[ii] < distance_closest_hunter)
      {
        idx_closest_hunter = ii;
        distance_closest_hunter = distance_to_hunters[ii];
      }
    }
#endif
#if 0

    for (size_t i = 0; i < msg->green_alive.size(); i++)
    {
      std::tuple<float, float> t = getDistanceAndAngleToPlayer(msg->green_alive[i]);
      std::tuple<float, float> t2 = getDistanceAndAngleToPlayer(msg->red_alive[i]);
      distance_to_preys.push_back(std::get<0>(t));
      angle_to_preys.push_back(std::get<1>(t));
      distance_to_hunters.push_back(std::get<0>(t2));
      angle_to_hunters.push_back(std::get<1>(t2));

      if (isnan(distance_to_preys[i]))
      {
      }
      else
      {
        distance_closest_prey = distance_to_preys[i];
        idx_closest_prey = i;
      }
    }
#endif
    std::tuple<float, float> t_world = getDistanceAndAngleToPlayer("world");
    float distance_to_world = (std::get<0>(t_world));
    float angle_to_world = (std::get<1>(t_world));
    float dx = 10;
    float angle;

    if ((distance_to_world) > 7.8)
    {
      angle = angle_to_world + M_PI / 2;
    }
    else
    {
      if (distance_closest_hunter < 2)
      {
        angle = -angle_to_hunters[idx_closest_hunter];
      }
      else
      {
        angle = angle_to_preys[idx_closest_prey];
      }
    }

    // STEP 2.5: check values
    float dx_max = msg->cheetah;
    dx > dx_max ? dx = dx_max : dx = dx; // operadores ternarios
    double amax = M_PI / 30;
    fabs(angle) > fabs(amax) ? angle = amax * angle / fabs(angle) : angle = angle;

    // STEP 3: define local movment
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, angle);
    T1.setRotation(q);

    // STEP 4: define global movment
    tf::Transform Tglobal = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    visualization_msgs::Marker marker;
    marker.header.frame_id = player_name;
    marker.header.stamp = ros::Time();
    marker.ns = player_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.y = 0.4;
    marker.scale.z = 0.3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if (team_preys->player_names[idx_closest_prey] == "ttavares")
    {
      marker.text = "vou-te comer " + team_preys->player_names[idx_closest_prey];
    }
    else if (team_preys->player_names[idx_closest_prey] == "acastro")
    {
      marker.text = "es um burro " + team_preys->player_names[idx_closest_prey];
    }
    else
    {
      marker.text = "ja foste " + team_preys->player_names[idx_closest_prey];
    }

    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub->publish(marker);
  }

  tuple<float, float> getDistanceToArenaCenter(string other_player)
  {
    return getDistanceAndAngleToPlayer("world");
  }

  tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
  {
    tf::StampedTransform T;
    try
    {
      listener.lookupTransform(player_name, other_player, ros::Time(0), T);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      return {1000, 0.0};
    }
    float distance = sqrt(T.getOrigin().y() * T.getOrigin().y() + T.getOrigin().x() * T.getOrigin().x());
    float angle = atan2(T.getOrigin().y(), T.getOrigin().x());
    return {distance, angle};
  }

private:
};
} // namespace talmeida_ns

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "talmeida");
  ros::NodeHandle n;

  talmeida_ns::MyPlayer player("talmeida", "red");
  // Player player("talmeida");
  // player.setTeamName(0); // ou "red"
  // cout << "Hello world from " << player.player_name << " of team "
  //<< player.getTeamName() << endl;
  // talmeida_ns::Team team_red("red");
  // team_red.player_names.push_back("talmeida"); // acrescenat elementos a
  // equipa
  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &talmeida_ns::MyPlayer::makeAPlayCallback, &player);

  player.printInfo();
  ros::Rate r(20);
  while (ros::ok())
  {
    // team_red.printInfo();

    // cout << "talmeida belongs to team?"
    //     << team_red.playerBelongsToTeam("talmeida") << endl;
    ros::spinOnce();
    r.sleep();
  }
  return 1;
}
