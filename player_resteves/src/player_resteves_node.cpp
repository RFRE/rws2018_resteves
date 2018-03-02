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
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

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
            ros::Publisher pub; 

            MyPlayer(string name, string team) : Player(name)
            {
                pub = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );

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

                struct timeval t1;
                gettimeofday(&t1, NULL);
                srand(t1.tv_usec);
                double start_x = ((double)rand()/(double)RAND_MAX) *10 -5;
                double start_y = ((double)rand()/(double)RAND_MAX) *10 -5;
                printf("start_x=%f start_y=%f\n", start_x, start_y);
                warp(start_x, start_y, M_PI/2);

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

                    //---------------------------------------
                    //--- AI PART 
                    //---------------------------------------
                    double displacement = 6; //computed using AI
                    double delta_alpha = M_PI/2;

                    showMarker();

                    //---------------------------------------
                    //--- CONSTRAINS PART 
                    //---------------------------------------
                    double displacement_max = msg->dog;
                    displacement > displacement_max ? displacement = displacement_max: displacement = displacement;

                    double delta_alpha_max = M_PI/30;
                    fabs(delta_alpha) > fabs(delta_alpha_max) ? delta_alpha = delta_alpha_max * delta_alpha / fabs(delta_alpha): delta_alpha = delta_alpha;

                    tf::Transform my_move_T; //declare the transformation object (player's pose wrt world)
                    my_move_T.setOrigin( tf::Vector3(displacement, 0.0, 0.0) );
                    tf::Quaternion q1;
                    q1.setRPY(0, 0, delta_alpha);
                    my_move_T.setRotation(q1);
            
                    T = T * my_move_T;
                    br.sendTransform(tf::StampedTransform(T, ros::Time::now(), "world", "resteves"));
                }

                void showMarker()
                {
                    //pub = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "resteves";
                    marker.header.stamp = ros::Time();
                    marker.ns = "my_namespace";
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = 0;
                    marker.pose.position.y = 0;
                    marker.pose.position.z = 0;
                    marker.text = "aaaaaaaah";
                    // marker.pose.orientation.x = 0.0;
                    // marker.pose.orientation.y = 0.0;
                    // marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    // marker.scale.x = .3;
                    // marker.scale.y = 0.1;
                    marker.scale.z = 0.4;
                    marker.color.a = 1.0; // Don't forget to set the alpha!
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                    // marker.lifetime = ros::Duration(2);
                    pub.publish(marker);
                }

                void printReport()
                {
                    ROS_INFO("My name is %s and my team is %s", name.c_str(), (getTeamName().c_str()) );
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