#include <yaml-cpp/yaml.h>

int main(){
    try {
        YAML::Node config = YAML::LoadFile("config.yaml");
    } catch (const YAML::BadFile& e) {
        // Can't open the file
        std::cerr << "Cannot open the configuration file." << std::endl;
    } catch (const YAML::ParserException& e) {
        // The file is not a valid YAML
        std::cerr << "Parsing error: " << e.what() << std::endl;
    }
}