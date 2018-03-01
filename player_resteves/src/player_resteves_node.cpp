#include <iostream>                                                                                     

class Player
{
    public:

    Player(std::string argin_name)
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
            std::cout << "Error: invalid color index input" << std::endl;
            return 0;
        }
    }

    // Setter for team_name
    int setTeamName(std::string argin_team)
    {
        if(argin_team == "red" || argin_team == "green" || argin_team == "blue")
        {
            team_name = argin_team;
            return 1;
        }
        else
        {
            std::cout << "Error: invalid color input: " << argin_team << std::endl;
            return 0;
        }
    }

    std::string name;
    std::string getTeam(void){return team_name;}

    private:

    std::string team_name;

};

class MyPlayer : public Player
{
    public:
    MyPlayer(std::string name, std::string team) : Player(name)
    {
        setTeamName(team);
    }
};

// function overload
int somar(int a, int b){return a+b;}
double somar(double a, double b){return a+b;}

int main()
{

    std::string player_name = "resteves";
    //Creating an instance of class Player
    Player player(player_name);
    player.setTeamName(2);
    

    std::cout << "Created an instance of class player with public name " << player.name << std::endl;
    std::cout << "Team is " << player.getTeam() << std::endl;

    MyPlayer my_player("resteves", "red");
    std::cout << "Created an instance of class player with public name " << my_player.name << std::endl;
    std::cout << "Team is " << my_player.getTeam() << std::endl;

}