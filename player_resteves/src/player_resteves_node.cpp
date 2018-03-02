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
#include <rws2018_msgs/MakeAPlay.h>

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

            string name;

            //overloaded setter for team_name
            int setTeamName(int index)
            {
                if(index == 1){return setTeamName("red");}
                else if(index == 2){return setTeamName("green");}
                else if(index == 3){return setTeamName("blue");}
                else{setTeamName("none");}
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
                    ROS_ERROR("cannot set team name to %s", argin_team.c_str());
                    ros::shutdown();
                }
            }

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
            boost::shared_ptr<Team> my_team;
            boost::shared_ptr<Team> my_preys;
            boost::shared_ptr<Team> my_hunters;

            tf::TransformBroadcaster br; //declare the broadcaster
            ros::NodeHandle n;
            boost::shared_ptr<ros::Subscriber> sub;
            tf::Transform T; //declare the transformation object (player's pose wrt world)


            MyPlayer(string name, string team) : Player(name)
            {
                red_team = boost::shared_ptr<Team> (new Team("red"));
                green_team = boost::shared_ptr<Team> (new Team("green"));
                blue_team = boost::shared_ptr<Team> (new Team("blue"));

                if (red_team->playerBelongsToTeam(name))
                {
                    my_team = red_team;
                    my_preys = green_team;
                    my_hunters = blue_team;
                    setTeamName("red");
                }
                else if (green_team->playerBelongsToTeam(name))
                {
                    my_team = green_team;
                    my_preys = blue_team;
                    my_hunters = red_team;
                    setTeamName("green");
                }
                else if (blue_team->playerBelongsToTeam(name))
                {
                    my_team = blue_team;
                    my_preys = red_team;
                    my_hunters = green_team;
                    setTeamName("blue");
                }

                sub = boost::shared_ptr<ros::Subscriber> (new ros::Subscriber());
                *sub = n.subscribe("/make_a_play", 100, &MyPlayer::move, this);

                warp(randomizePosition(), randomizePosition(), M_PI/2);

                printReport();
            }

            void warp(double x, double y, double alfa)
            {
                T.setOrigin( tf::Vector3(x, y, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, alfa);
                T.setRotation(q);
                br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "resteves"));
                ROS_INFO("Warping to x=%f y=%f a=%f", x,y,alfa);
            }

            void move(const rws2018_msgs::MakeAPlay::ConstPtr& msg)
            {
                double x = T.getOrigin().x();
                double y = T.getOrigin().y();
                double a = 0;

                T.setOrigin( tf::Vector3(x+=0.01, y, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, a);
                T.setRotation(q);

                br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "resteves"));
                ROS_INFO("Moving to ");
            }

            void printReport()
            {
                //cout << "My name is " << name << " and my team is " << getTeamName() << endl;
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
    
    //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

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
    
    // ros::Rate loop_rate(10);
    // while(ros::ok())
    // {
    //     my_player.move();

    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }

    ros::spin();
}