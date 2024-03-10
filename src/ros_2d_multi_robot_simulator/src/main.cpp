#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

int main(int argc, char **argv)
{

    if (argc != 2)
    {
        std::cout << "The correct use of the programm is ./parser configuration_file" << std::endl;
        return 1;
    }

    std::string mapName;

    try
    {
        YAML::Node config = YAML::LoadFile(argv[1]); // Load the configuration file from the first input argument

        mapName = config["environment"]["map"].as<std::string>(); // Map name parsing

        std::cout << "Map name: " << mapName << std::endl;

        // Robots parsing
        for (YAML::const_iterator it = config["robots"].begin(); it != config["robots"].end(); ++it)
        {

            YAML::Node robot = *it;

            // Robot ids parsing
            std::string id = robot["id"].as<std::string>();             // ID robot parsing
            std::string frame_id = robot["frame_id"].as<std::string>(); // Frame ID parsing

            // Max velocities parsing
            double max_linear_velocity = robot["max_velocity"]["linear"].as<double>();   // Max linear velocity parsing
            double max_angular_velocity = robot["max_velocity"]["angular"].as<double>(); // Max angular velocity parsing

            std::cout << "Robot ID: " << id << std::endl;
            std::cout << "Frame ID: " << frame_id << std::endl;
            std::cout << "Max Linear Velocity: " << max_linear_velocity << " m/s" << std::endl;
            std::cout << "Max Angular Velocity: " << max_angular_velocity << " rad/s" << std::endl;

            // Devices parsing
            for (YAML::const_iterator it_dev = robot["devices"].begin(); it_dev != robot["devices"].end(); ++it_dev)
            {
                YAML::Node device = *it_dev;

                std::string device_type = device["type"].as<std::string>();         // Device type parsing
                std::string device_frame_id = device["frame_id"].as<std::string>(); // Device frame ID parsing
                std::string device_topic = device["topic"].as<std::string>();       // Device topic parsing
                int device_beams = device["beams"].as<int>();                       // Device beams parsing
                double min_range = device["range"]["min"].as<double>();             // Device min range parsing
                double max_range = device["range"]["max"].as<double>();             // Device max range parsing

                std::cout << "  Device Type: " << device_type << std::endl;
                std::cout << "  Device Frame ID: " << device_frame_id << std::endl;
                std::cout << "  Device Topic: " << device_topic << std::endl;
                std::cout << "  Device Beams: " << device_beams << std::endl;
                std::cout << "  Min Range: " << min_range << " meters" << std::endl;
                std::cout << "  Max Range: " << max_range << " meters" << std::endl;
            }
            std::cout << std::endl;
        }
    }
    catch (const YAML::BadFile &e)
    {
        // Can't open the file
        std::cerr << "Cannot open the configuration file." << std::endl;
    }
    catch (const YAML::ParserException &e)
    {
        // The file is not a valid YAML
        std::cerr << "Parsing error: " << e.what() << std::endl;
    }

    system("xterm -e 'roscore' &");
    system(("xterm -e './robsim_node " + mapName + "' &").c_str());
    system("xterm -e 'rviz' &");

    return 0;
}