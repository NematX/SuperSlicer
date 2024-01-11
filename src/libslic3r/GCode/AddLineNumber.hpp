#ifndef slic3r_AddLineNumber_hpp_
#define slic3r_AddLineNumber_hpp_

#include "../PrintConfig.hpp"
#include "../GCodeReader.hpp"

#include <atomic>

namespace Slic3r {

class AddLineNumber {
public:
    AddLineNumber(const PrintConfig &print_config) { activated = print_config.gcode_line_number; }
    
    std::string process_layer(const std::string &gcode);
    void process_string(std::string &gcode);

protected:
    void _process_gcode_line(GCodeReader& reader, const GCodeReader::GCodeLine& line, std::string &process_output);
    bool activated = true;
    // ok as it's sequential, not in //, no need of atomic
    size_t nb_lines = 0;
};

}

#endif // slic3r_AddLineNumber_hpp_
