#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"
using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    /* Marcos Darino - Start  - Added a input validation but for most of the cases, not for all, for example "4-5" will return as valid number 4 */

    std::string start_x_str, start_y_str, end_x_str, end_y_str;
    
    std::cout << "Enter the start X position: ";
    std::cin >> start_x_str;
    if( start_x_str.find_first_not_of("1234567890.-") != std::string::npos )
    {
        std::cout << "invalid number: " << start_x_str << std::endl;
        return -1;
    }
    
    std::cout << "Enter the start Y position: ";
    std::cin >> start_y_str;
    if( start_y_str.find_first_not_of("1234567890.-") != std::string::npos )
    {
        std::cout << "invalid number: " << start_y_str << std::endl;
        return -1;
    }
    
    std::cout << "Enter the end X position: ";
    std::cin >> end_x_str;
    if( end_x_str.find_first_not_of("1234567890.-") != std::string::npos )
    {
        std::cout << "invalid number: " << end_x_str << std::endl;
        return -1;
    }
    
    std::cout << "Enter the end Y position: ";
    std::cin >> end_y_str;
    if( end_y_str.find_first_not_of("1234567890.-") != std::string::npos )
    {
        std::cout << "invalid number: " << end_y_str << std::endl;
        return -1;
    }

    float start_x, start_y, end_x, end_y;
    start_x = atof(start_x_str.c_str());
    start_y = atof(start_y_str.c_str());
    end_x = atof(end_x_str.c_str());
    end_y = atof(end_y_str.c_str());

    std::cout << "Start(" << start_x << ", " << start_y << ")";
    std::cout << " End(" << end_x << ", " << end_y << ")\n";
    /* Marcos Darino - End */

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y}; /* Marcos Darino - Added the user params for the start and end position */
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
