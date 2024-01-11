#ifndef slic3r_RemoveComments_hpp_
#define slic3r_RemoveComments_hpp_

#include "../PrintConfig.hpp"
#include "../GCodeReader.hpp"

namespace Slic3r {

class RemoveComments {
public:
    RemoveComments(const PrintConfig &print_config) { activated = print_config.gcode_no_comment; }
    
    std::string process_layer(const std::string &gcode);
    void process_string(std::string &gcode);

protected:
    void _process_gcode_line(GCodeReader& reader, const GCodeReader::GCodeLine& line, std::string &process_output);
    bool activated = true;
};

}

#endif // slic3r_RemoveComments_hpp_
