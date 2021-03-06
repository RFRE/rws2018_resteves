#include <iostream>   
#include <vector>                                                                                     

//Boost includes
#include <boost/shared_ptr.hpp>

//Ros includes
#include <ros/ros.h>
#include <rws2018_libs/team.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <rws2018_msgs/MakeAPlay.h>

#define DEFAULT_TIME 0.05

using namespace std;
using namespace ros;
using namespace tf;

namespace rws_resteves
{

  class Player
  {
    public:
      Player(string argin_name) {name = argin_name;}

      string name;

      //Overloaded setter for team_nam
      int setTeamName(int index = 2)
      {
        if (index == 0){ setTeamName("red");}
        else if (index == 1){ setTeamName("green");}
        else if (index == 2){ setTeamName("blue");}
        else { setTeamName("none");}
      }

      //Setter for team_nam
      int setTeamName(string argin_team)
      {
        if (argin_team=="red" || argin_team=="green" || argin_team=="blue")
        {
          team_name = argin_team; return 1;
        }
        else
        {
          ROS_ERROR("cannot set team name to %s", argin_team.c_str());
          ros::shutdown();
        }
      }

      //Getter of team_name
      string getTeamName(void)
      {
        return team_name;
      }

    private:
      string team_name;
  };


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
      boost::shared_ptr<ros::Publisher> pub;
      tf::TransformListener listener;

      MyPlayer(string argin_name, string argin_team/*disregard this one. overrided by params*/) : Player(argin_name)
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

      pub = boost::shared_ptr<ros::Publisher> (new ros::Publisher());
      *pub = n.advertise<visualization_msgs::Marker>( "/bocas", 0 );

      struct timeval t1;
      gettimeofday(&t1, NULL);
      srand(t1.tv_usec);
      double start_x = ((double)rand()/(double)RAND_MAX) *10 -5;
      double start_y = ((double)rand()/(double)RAND_MAX) *10 -5;
      printf("start_x=%f start_y=%f\n", start_x, start_y);

      ros::Duration(0.1).sleep();
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


      double getAngleToPLayer(string other_player, double time_to_wait=DEFAULT_TIME)
      {
        StampedTransform t; //The transform object
        //Time now = Time::now(); //get the time
        Time now = Time(0); //get the latest transform received

        try{
          listener.waitForTransform("resteves", other_player, now, Duration(time_to_wait));
          listener.lookupTransform("resteves", other_player, now, t);
        }
        catch (TransformException& ex){
          ROS_ERROR("%s",ex.what());
          return NAN;
        }

        return atan2(t.getOrigin().y(), t.getOrigin().x());
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
        double delta_alpha = getAngleToPLayer("moliveira");

        if (isnan(delta_alpha))
          delta_alpha = 0;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "resteves";
        marker.header.stamp = ros::Time();
        marker.ns = "resteves";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.4;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.5;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.text = "aaaaaaaaah";
        marker.lifetime = ros::Duration(2);
        pub->publish( marker );

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

      void printReport()
      {
        ROS_INFO("My name is %s and my team is %s", name.c_str(), (getTeamName().c_str()) );
      }

  };

}//end of namespace

int main(int argc, char** argv)
{

  ros::init(argc, argv, "resteves");
  ros::NodeHandle n;



  //Creating an instance of class Player
  rws_resteves::MyPlayer my_player("resteves", "doesnotmatter");

  ros::spin();
  //ros::Rate loop_rate(10);
  //while (ros::ok())
  //{
  //my_player.move();

  //ros::spinOnce();
  //loop_rate.sleep();
  //}

}

