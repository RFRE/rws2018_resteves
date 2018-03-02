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
                    setTeamName("none");
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
                    //cout << "Error: invalid color input: " << argin_team << endl;
                    ROS_ERROR("Invalid color input: %s", argin_team.c_str());
                    return 0;
                }
            }

            string name;

            // Getter of team_name
            string getTeamName(void){return team_name;}

        private:
            string team_name;
    };

    // class MyPlayer inherited class Player's "methods"
    class MyPlayer : public Player
    {
        public:
            boost::shared_ptr<Team> red_team;
            boost::shared_ptr<Team> green_team;
            boost::shared_ptr<Team> blue_team;
            tf::TransformBroadcaster br; //declare the broadcaster

            MyPlayer(string name, string team) : Player(name)
            {
               red_team = boost::shared_ptr<Team> (new Team("red"));
                green_team = boost::shared_ptr<Team> (new Team("green"));
                blue_team = boost::shared_ptr<Team> (new Team("blue"));

                setTeamName(team);
                printReport();
            }

            void move(void)
            {
                tf::Transform transform; //declare the transformation object
                transform.setOrigin( tf::Vector3(7, 7, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, M_PI/3);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "resteves"));
            }

            void printReport()
            {
                //cout << "My name is " << name << " and my team is " << getTeamName() << endl;
                ROS_INFO();
                ROS_INFO("Player %s is on the %s team.", name.c_str(), getTeamName().c_str());
            }
    };

} // end of namespace

// function overload example
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