#include "RemoveComments.hpp"

namespace Slic3r {

void RemoveComments::process_string(std::string &gcode)
{
    if (activated) {
        std::string ret = this->process_layer(gcode);
        gcode           = ret;
    }
}

std::string RemoveComments::process_layer(const std::string &gcode)
{
    if (!activated)
        return gcode;

    std::string process_output = "";
    GCodeReader parser;
    //process_output.append(";<===START layer===>\n");
    if (!gcode.empty())
        parser.parse_buffer(gcode, [this, &process_output](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
            /*m_process_output += line.raw() + "\n";*/
            this->_process_gcode_line(reader, line, process_output);
        });
    
    //process_output.append(";<===END layer===>\n");
    return process_output;
}

void RemoveComments::_process_gcode_line(GCodeReader &reader, const GCodeReader::GCodeLine &line, std::string &process_output)
{
    //remove commands that aren't G1 or M
    std::string_view cmd = line.cmd();
    if (!cmd.empty()) {
        //too extreme
        //if (cmd.front() == 'N' ||cmd.front() == 'G' || cmd.front() == 'M') {
        //    // TODO: remove comment inside line.raw()
        //    else {
        //        process_output.append(line.raw().substr(0,idx)).append("\n");
        //    }
        //}
         if (auto idx = line.raw().find(";"); idx == std::string::npos) {
             if(!boost::trim_copy(line.raw()).empty())
                process_output.append(line.raw()).append("\n");
         } else {
             process_output.append(line.raw().substr(0,idx)).append("\n");
         }
    }
}

} // namespace Slic3r
