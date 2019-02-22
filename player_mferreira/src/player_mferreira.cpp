#include <iostream>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>
#include <vector>

using namespace std;
using namespace boost;
using namespace ros;

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

  Player(std::string player_name_in) { player_name = player_name_in; }

  // Set team name, if given a correct team name (accessor)
  void setTeamName(std::string team_name_in)
  {
    if (team_name_in == "red" || team_name_in == "green" ||
        team_name_in == "blue")
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

  std::string getTeamName() { return team_name; }

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
  tf::Transform transform;

  MyPlayer(std::string player_name_in, std::string team_name_in)
      : Player(player_name_in)
  {
    team_red = (boost::shared_ptr<Team>)new Team("red");
    team_green = (boost::shared_ptr<Team>)new Team("green");
    team_blue = (boost::shared_ptr<Team>)new Team("blue");

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
  }

  void printInfo()
  {
    ROS_INFO_STREAM("My name is " << player_name << " and my team is "
                                  << team_mine->team_name << endl);
    ROS_INFO_STREAM("I am hunting team " << team_preys->team_name
                                         << " and fleeing from team "
                                         << team_hunters->team_name << endl);
  }

  void makeAPlayCallback(rws2019_msgs::MakeAPlayConstPtr msg)
  {
    ROS_INFO("received a new message");

    tf::Transform transform1;
    transform1.setOrigin(tf::Vector3(3.0, 4.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform1.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world",
                                          player_name));
  }
};

} // namespace mferreira_ns

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mferreira");
  ros::NodeHandle n;
  mferreira_ns::MyPlayer player("mferreira", "green");
  std::cout << "Hello world from " << player.player_name << " of team "
            << player.getTeamName() << std::endl;

  mferreira_ns::Team team_green("red");

  ros::Subscriber sub = n.subscribe(
      "/make_a_play", 100, &mferreira_ns::MyPlayer::makeAPlayCallback, &player);

  while (ros::ok())
  {
    ros::Duration(1).sleep();
    player.printInfo();
    ros::spinOnce();
  }

  return 0;
}