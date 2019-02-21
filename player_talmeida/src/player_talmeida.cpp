#include <boost/shared_ptr.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace std; // ja na e preciso usar o std
using namespace ros;

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

    // methods
    Player(string player_name_in) { player_name = player_name_in; }

    void setTeamName(string team_name_in)
    {
        if (team_name_in == "red" || team_name_in == "green" ||
            team_name_in == "blue")
        {
            team_name = team_name_in;
        }
        else
        {
            cout << "Cannot set team name" << team_name_in << endl;
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

    string getTeamName() { return team_name; };

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

    MyPlayer(string player_name_in, string team_name_in)
        : Player(player_name_in) // construtor da class
    {
        setTeamName(team_name_in);
        team_red = (boost::shared_ptr<Team>)new Team("red"); // mallock
        team_green = (boost::shared_ptr<Team>)new Team("green");
        team_blue = (boost::shared_ptr<Team>)new Team("blue");

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
        // team_red->printInfo();
        printInfo();
    }

    void printInfo()
    {
        // cout << "debugger" << endl;
        ROS_INFO_STREAM("My name is " << player_name << " and my team is "
                                      << team_mine->team_name);
        ROS_INFO_STREAM("I am hunting " << team_preys->team_name
                                        << " and fleeing from "
                                        << team_hunters->team_name);
    }
};

} // namespace talmeida_ns

int doSomething() { return 0; }

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

    while (ros::ok())
    {
        // team_red.printInfo();

        // cout << "talmeida belongs to team?"
        //     << team_red.playerBelongsToTeam("talmeida") << endl;
        ros::Duration(1).sleep();
    }
    return 0;
}
