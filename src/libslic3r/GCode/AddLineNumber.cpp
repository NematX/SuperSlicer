#include "AddLineNumber.hpp"

namespace Slic3r {

void AddLineNumber::process_string(std::string &gcode)
{
    if (activated) {
        std::string ret = this->process_layer(gcode);
        gcode           = ret;
    }
}

std::string AddLineNumber::process_layer(const std::string &gcode)
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

void AddLineNumber::_process_gcode_line(GCodeReader &reader, const GCodeReader::GCodeLine &line, std::string &process_output)
{
    //remove commands that aren't G1 or M
    std::string_view cmd = line.cmd();
    if (!cmd.empty()) {
        if (cmd.front() == 'G' || cmd.front() == 'M') {
            process_output.append("N").append(std::to_string(++nb_lines)).append(" ").append(line.raw()).append("\n");
            //process_output.append(line.raw()).append("\n");
        }else
            process_output.append(line.raw()).append("\n");
    } else
        process_output.append(line.raw()).append("\n");
}

} // namespace Slic3r
