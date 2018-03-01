// system libraries
#include <iostream>                                                                                     
#include <vector>
#include <boost/shared_ptr.hpp>
#include <sstream>

// include ros libraries
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// other libraries
#include <rws2018_libs/team.h>

// this way i don't have to write std::something all the time
using namespace std;

namespace rws_resteves
{

class Player
{
    public:

    Player(string argin_name)
    {
        name = argin_name;
    }

    //overloaded setter for team_name
    int setTeamName(int index)
    {
        if(index == 1)
        {
            return setTeamName("red");
        }
        else if(index == 2)
        {
            return setTeamName("green");
        }
        else if(index == 3)
        {
            return setTeamName("blue");
        }
        else
        {
            cout << "Error: invalid color index input" << endl;
            return 0;
        }
    }

    // Setter for team_name
    int setTeamName(string argin_team)
    {
        if(argin_team == "red" || argin_team == "green" || argin_team == "blue")
        {
            team_name = argin_team;
            return 1;
        }
        else
        {
            cout << "Error: invalid color input: " << argin_team << endl;
            return 0;
        }
    }

    string name;
    string getTeam(void){return team_name;}

    private:

    string team_name;

};

// class Team
// {
//     public: 
      
//        /**
//        * @brief Constructor
//        * @param team_name the team name
//        */
//       Team(string team_name)
//       {
//         name = team_name; 

//       }

//       /**
//        * @brief Prints the name of the team and the names of all its players
//        */
//       void printTeamInfo(void)
//       {
//         //Write code here ...
//       }

//       /**
//        * @brief Checks if a player belongs to the team
//        * @param player_name the name of the player to check
//        * @return true or false, yes or no
//        */
//       bool playerBelongsToTeam(string player_name)
//       {
//         //write code here ...
//       }

//       /**
//        * @brief The team name
//        */
//       string name;

//       /**
//        * @brief A list of the team's player names
//        */
//       vector<string> players;
// };

// class MyPlayer inherited class Player's "methods"
class MyPlayer : public Player
{
    public:

    boost::shared_ptr<Team> red_team;
    boost::shared_ptr<Team> green_team;
    boost::shared_ptr<Team> blue_team;

    MyPlayer(string name, string team) : Player(name)
    {
        
        setTeamName(team);
        printReport();
    }

    void move(void)
    {
        static tf::TransformBroadcaster br; // declares the broadcaster
        tf::Transform transform; // declares the transformation object
        transform.setOrigin( tf::Vector3(7, 7, 0.0) );
        tf::Quaternion q;
        q.setRPY(0, 0, M_PI/3);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "resteves"));
    }

    void printReport()
    {
        cout << "My name is " << name << " and my team is " << getTeam() << endl;
    }
};

} // end of namespace

// function overload
int somar(int a, int b){return a+b;}
double somar(double a, double b){return a+b;}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "resteves");

    ros::NodeHandle n;

    rws_resteves::MyPlayer my_player("resteves", "blue");
    
    // string test_param_value;
    // vector<string> test_red;

    //n.getParam("test_param", test_param_value);
    //n.getParam("team_red", test_red);

    // cout << "read test_param with value " << test_param_value << endl;

    // for (int x = 0; x != test_red.size(); ++x)
    // {
    //     cout << "Player " << x+1 << " name: " << test_red[x] << endl;
    //     //cout << test_red.at(x) << "- calling at member" << endl;
    // }
    
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        my_player.move();

        ros::spinOnce();
        loop_rate.sleep();
    }
}