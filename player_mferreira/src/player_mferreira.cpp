#include <math.h>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace boost;
using namespace ros;

float randomizePosition()
{
  srand(6832 * time(NULL));  // set initial seed value to 5323
  return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

namespace mferreira_ns
{
class Team
{
public:
  string team_name;
  vector<string> player_names;
  ros::NodeHandle n;

  Team(string team_name_in)
  {
    team_name = team_name_in;
    // read the team players
    n.getParam("/team_" + team_name, player_names);
  }

  void printInfo()
  {
    cout << "Team " << team_name << " has players: " << endl;
    for (size_t i = 0; i < player_names.size(); i++)
    {
      cout << player_names[i] << endl;
    }
  }

  bool playerBelongsToTeam(string player_name)
  {
    for (size_t i = 0; i < player_names.size(); i++)
    {
      if (player_name == player_names[i])
      {
        return true;
      }
    }
    return false;
  }

private:
};

class Player
{
public:
  // properties
  string player_name;

  // std::string player_name;

  Player(std::string player_name_in)
  {
    player_name = player_name_in;
  }

  // Set team name, if given a correct team name (accessor)
  void setTeamName(std::string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" || team_name_in == "blue")
    {
      team_name = team_name_in;
    }
    else /* code */
    {
      std::cout << "Cannot set team name" << team_name_in << std::endl;
    }
  }

  void setTeamName(int team_index)
  {
    if (team_index == 0)
      setTeamName("red");
    else if (team_index == 1)
      setTeamName("green");
    else if (team_index == 2)
      setTeamName("blue");
    else
    {
      setTeamName("");
    }
  }

  std::string getTeamName()
  {
    return team_name;
  }

private:
  std::string team_name;
};

class MyPlayer : public Player
{
public:
  boost::shared_ptr<Team> team_red;
  boost::shared_ptr<Team> team_green;
  boost::shared_ptr<Team> team_blue;
  boost::shared_ptr<Team> team_hunters;
  boost::shared_ptr<Team> team_mine;
  boost::shared_ptr<Team> team_preys;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  boost::shared_ptr<ros::Publisher> vis_pub;
  // =

  MyPlayer(std::string player_name_in, std::string team_name_in) : Player(player_name_in)
  {
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");
    ros::NodeHandle n;
    vis_pub = (boost::shared_ptr<ros::Publisher>)new ros::Publisher;
    (*vis_pub) = n.advertise<visualization_msgs::Marker>("/bocas", 0);

    if (team_red->playerBelongsToTeam(player_name))
    {
      team_mine = team_red;
      team_preys = team_green;
      team_hunters = team_blue;
    }
    else if (team_green->playerBelongsToTeam(player_name))
    {
      team_mine = team_green;
      team_preys = team_blue;
      team_hunters = team_red;
    }
    else if (team_blue->playerBelongsToTeam(player_name))
    {
      team_mine = team_blue;
      team_preys = team_red;
      team_hunters = team_green;
    }
    else
    {
      cout << "Something wrong in team parameterization!" << endl;
    }

    setTeamName(team_mine->team_name);

    // define intial position
    float sx = randomizePosition();
    float sy = randomizePosition();
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
    ROS_INFO_STREAM("My name is " << player_name << " and my team is " << team_mine->team_name << endl);
    ROS_INFO_STREAM("I am hunting team " << team_preys->team_name << " and fleeing from team "
                                         << team_hunters->team_name << endl);
  }

  // float getDistanceToPlayer(string other_player)
  // {
  //   tf::StampedTransform T0;

  //   try
  //   {
  //     listener.lookupTransform("mferreira", other_player, ros::Time(0), T0);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s", ex.what());
  //     ros::Duration(0.1).sleep();
  //     return 1000;
  //   }

  //   return sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());
  // }

  std::tuple<float, float> getDistanceAndAngleToPlayer(string other_player)
  {
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform(player_name, other_player, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.01).sleep();
      return { 1000.0, 0.0 };
    }

    float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());
    float a = atan2(T0.getOrigin().y(), T0.getOrigin().x());
    //            return std::tuple<float, float>(d,a);
    return { d, a };
  }

  // std::tuple<float, float> getDistanceToLimits(string other_player)
  // {
  //   tf::StampedTransform T0;
  //   try
  //   {
  //     listener.lookupTransform(player_name, other_player, ros::Time(0), T0);
  //   }
  //   catch (tf::TransformException ex)
  //   {
  //     ROS_ERROR("%s", ex.what());
  //     ros::Duration(0.01).sleep();
  //     return { 1000.0, 0.0 };
  //   }

  //   float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());
  //   float a = atan2(T0.getOrigin().y(), T0.getOrigin().x());
  //   //            return std::tuple<float, float>(d,a);
  //   return { d, a };
  // }

  std::tuple<float, float> getDistanceAndAngleToOrigin()
  {
    return getDistanceAndAngleToPlayer("world");
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new message");

    // Step 1:Find out where I am
    tf::StampedTransform T0;
    try
    {
      listener.lookupTransform("/world", player_name, ros::Time(0), T0);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.1).sleep();
    }

    float distanceToOrigin = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y());

    // Step 2: Define how I want to move

    // team_preys = team_red;
    // team_hunters = team_green;

    // PREYS
    vector<float> distance_to_preys;
    vector<float> angle_to_preys;

    for (size_t i = 0; i < team_preys->player_names.size(); i++)
    {
      ROS_WARN_STREAM("team_preys = " << team_preys->player_names[i]);

      std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_preys->player_names[i]);
      distance_to_preys.push_back(std::get<0>(t));
      angle_to_preys.push_back(std::get<1>(t));
    }

    // HUNTERS
    vector<float> distance_to_hunters;
    vector<float> angle_to_hunters;

    for (size_t i = 0; i < team_hunters->player_names.size(); i++)
    {
      ROS_WARN_STREAM("team_hunters = " << team_hunters->player_names[i]);

      std::tuple<float, float> t = getDistanceAndAngleToPlayer(team_hunters->player_names[i]);
      distance_to_hunters.push_back(std::get<0>(t));
      angle_to_hunters.push_back(std::get<1>(t));
    }

    // compute closest hunter
    int idx_closest_hunter = 0;
    float distance_closest_hunter = 1000;
    for (size_t i = 0; i < distance_to_hunters.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_hunter)
      {
        idx_closest_hunter = i;
        distance_closest_hunter = distance_to_preys[i];
      }
    }

    // compute closest prey
    int idx_closest_prey = 0;
    float distance_closest_prey = 1000;
    for (size_t i = 0; i < distance_to_preys.size(); i++)
    {
      if (distance_to_preys[i] < distance_closest_prey)
      {
        idx_closest_prey = i;
        distance_closest_prey = distance_to_preys[i];
      }
    }

    float dx = 100;
    float a = angle_to_preys[idx_closest_prey];

    // Condicao para nao sair
    vector<float> distance_to_origin;
    vector<float> angle_to_origin;

    std::tuple<float, float> w = getDistanceAndAngleToOrigin();
    distance_to_origin.push_back(std::get<0>(w));
    angle_to_origin.push_back(std::get<1>(w));

    if ((distance_closest_hunter < distance_closest_prey * 0.5) && (distance_closest_hunter < 2))
    {
      // escape!!!!!!!!!;
      a = -angle_to_hunters[idx_closest_hunter];
    }
    else
    {
      // hunt!!!!!!!!!!;
      a = angle_to_preys[idx_closest_prey];
    }

    float minDist = 7;
    if (distance_to_origin[0] > minDist)
    {
      a = angle_to_hunters[idx_closest_hunter] + M_PI;
    }

    // Step 2.5: check values
    float dx_max = msg->turtle;
    dx > dx_max ? dx = dx_max : dx = dx;

    double amax = M_PI / 30;
    fabs(a) > fabs(amax) ? a = amax * a / fabs(a) : a = a;

    // Step 3:move
    tf::Transform T1;
    T1.setOrigin(tf::Vector3(dx, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, a);
    T1.setRotation(q);

    // br.sendTransform(tf::StampedTransform(T1, ros::Time::now(), "world",
    // player_name));

    // Step 4:
    tf::Transform Tglobal = T0 * T1;
    br.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", player_name));

    // Criar Marcador
    visualization_msgs::Marker marker;
    marker.header.frame_id = player_name;
    marker.header.stamp = ros::Time();
    marker.ns = player_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = 1;
    marker.pose.position.y = 0.5;
    // marker.pose.position.z = 1;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 1;
    // marker.scale.y = 0.1;
    marker.scale.z = 0.4;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.text = "Prey: " + team_preys->player_names[idx_closest_prey];
    // only if using a MESH_RESOURCE marker type:
    // marker.mesh_resource =
    // "package://pr2_description/meshes/base_v0/base.dae";
    vis_pub->publish(marker);
  }
};  // namespace mferreira_ns

}  // namespace mferreira_ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mferreira");
  ros::NodeHandle n;
  mferreira_ns::MyPlayer player("mferreira", "green");
  std::cout << "Hello world from " << player.player_name << " of team " << player.getTeamName() << std::endl;

  mferreira_ns::Team team_green("red");

  ros::Subscriber sub = n.subscribe("/make_a_play", 100, &mferreira_ns::MyPlayer::makeAPlayCallback, &player);

  player.printInfo();
  ros::Rate r(20);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}