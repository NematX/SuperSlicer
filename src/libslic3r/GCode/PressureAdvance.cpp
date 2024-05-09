#include "PressureAdvance.hpp"

#include "GCodeReader.hpp"
#include "LocalesUtils.hpp"

#include <iomanip>

namespace Slic3r {

    
PressureAdvance::PressureAdvance(const GCodeWriter &writer, const FullPrintConfig &print_config, uint16_t current_extruder_id)
    : m_current_extruder(current_extruder_id)
    , m_config(print_config)
    , m_writer(writer)
    , m_relative_e(print_config.use_relative_e_distances.value)
{
    m_parser.apply_config(print_config);
        
    double max_extruding_acceleration = 0;
    if (print_config.machine_limits_usage.value != MachineLimitsUsage::Ignore) {
        max_x_speed        = print_config.machine_max_feedrate_x.get_at(0);
        max_y_speed        = print_config.machine_max_feedrate_y.get_at(0);
        max_z_speed        = print_config.machine_max_feedrate_z.get_at(0);
        max_e_speed        = print_config.machine_max_feedrate_e.get_at(0);
        max_x_acceleration = print_config.machine_max_acceleration_x.get_at(0);
        max_y_acceleration = print_config.machine_max_acceleration_y.get_at(0);
        max_z_acceleration = print_config.machine_max_acceleration_z.get_at(0);
        max_e_acceleration = print_config.machine_max_acceleration_e.get_at(0);
        max_extruding_acceleration = print_config.machine_max_acceleration_extruding.get_at(0);
    }
    //TODO: remove from max_e_acceleration wht's needed for the algo to work.
    if (max_x_speed <= 0)
        max_x_speed = 100000;
    if (max_y_speed <= 0)
        max_y_speed = 100000;
    if (max_z_speed <= 0)
        max_z_speed = 100000;
    if (max_e_speed <= 0)
        max_e_speed = 100000;
    if (max_x_acceleration <= 0)
        max_x_acceleration = 100000;
    if (max_y_acceleration <= 0)
        max_y_acceleration = 100000;
    if (max_z_acceleration <= 0)
        max_z_acceleration = 100000;
    if (max_e_acceleration <= 0)
        max_e_acceleration = 100000;
    if (max_extruding_acceleration > 0) {
        max_x_acceleration = std::min(max_extruding_acceleration, max_x_acceleration);
        max_y_acceleration = std::min(max_extruding_acceleration, max_y_acceleration);
        max_z_acceleration = std::min(max_extruding_acceleration, max_z_acceleration);
    }
    m_current_speed = max_x_speed;
    m_current_acceleration = m_current_deceleration = max_x_acceleration;
    xyz_decimals = uint8_t(print_config.gcode_precision_xyz.value);
    e_decimals = uint8_t(print_config.gcode_precision_e.value);
}

const std::string &PressureAdvance::process_gcode(const std::string &gcode, bool do_flush, uint16_t extruder_id)
{
    m_process_output   = "";
    m_current_extruder = extruder_id;

    // recompute buffer time to recover from rounding
    m_buffer_distance_size = 0;
    for (auto &data : m_buffer) m_buffer_distance_size += data.max_axle_xyz;
    m_buffer_useful_size = 0;
    for (auto &data : m_buffer)
        if (data.is_useful_move())
            m_buffer_useful_size += data.max_axle_xyz;

    if (!gcode.empty())
        m_parser.parse_buffer(gcode, [this](GCodeReader &reader, const GCodeReader::GCodeLine &line) {
            /*m_process_output += line.raw() + "\n";*/
            this->_process_gcode_line(reader, line);
        });

    if (do_flush) {
        flush();
    }

    return m_process_output;
}

namespace PressureAdvance_func {

bool is_end_of_word(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == 0; }

float get_axis_value(const std::string &line, char axis)
{
    char match[3] = " X";
    match[1]      = axis;

    size_t pos = line.find(match) + 2;
    assert(pos != std::string::npos);
    if (pos == std::string::npos)
        return 0.f;
    // size_t end = std::min(line.find(' ', pos + 1), line.find(';', pos + 1));
    // Try to parse the numeric value.
    const char *c    = line.c_str();
    char *      pend = nullptr;
    errno            = 0;
    double v         = strtod(c + pos, &pend);
    if (pend != nullptr && errno == 0 && pend != c) {
        // The axis value has been parsed correctly.
        return float(v);
    }
    return NAN;
}

void change_axis_value(std::string &line, char axis, const float new_value, const int decimal_digits)
{
    char match[3] = " X";
    match[1]      = axis;

    assert(line.find(match) != std::string::npos);
    size_t pos = line.find(match) + 2;
    if (pos == std::string::npos)
        return;
    size_t end = std::min(line.find(' ', pos + 1), line.find(';', pos + 1));
    line       = line.replace(pos, end - pos, to_string_nozero(new_value, decimal_digits));
}

bool parse_number(const std::string_view sv, int &out)
{
    {
        // Legacy conversion, which is costly due to having to make a copy of the string before conversion.
        try {
            assert(sv.size() < 1024);
            assert(sv.data() != nullptr);
            std::string str{sv};
            size_t      read = 0;
            out              = std::stoi(str, &read);
            return str.size() == read;
        } catch (...) {
            return false;
        }
    }
}
bool parse_double(const std::string_view sv, double &out)
{
    {
        // Legacy conversion, which is costly due to having to make a copy of the string before conversion.
        try {
            assert(sv.size() < 1024);
            assert(sv.data() != nullptr);
            std::string str{sv};
            size_t      read = 0;
            out              = std::stol(str, &read);
            return str.size() == read;
        } catch (...) {
            return false;
        }
    }
}

double compute_delta_speed(double accel, double initial_speed, double distance)
{
    assert(accel > 0);
    assert(distance > 0);
    // at=-v0+sqrt(v0²+2ad)
    return std::sqrt(initial_speed * initial_speed + 2 * accel * distance) - initial_speed;
}

double compute_distance(double accel, double initial_speed, double final_speed)
{
    assert(accel > 0);
    // d =((v0+at)²-v0²)/2a
    //double final_speed = initial_speed + std::abs(delta_speed);
    assert(final_speed >= initial_speed);
    return (final_speed * final_speed - initial_speed * initial_speed) / (2 * accel);
}

} // namespace PressureAdvance_func

// FIXME: add other firmware
// or just create that damn new gcode writer arch
void PressureAdvance::_process_T(const std::string_view command)
{
    if (command.length() > 1) {
        int eid = 0;
        if (!PressureAdvance_func::parse_number(command.substr(1), eid) || eid < 0 || eid > 255) {
            GCodeFlavor flavor = m_writer.config.gcode_flavor;
            // Specific to the MMU2 V2 (see https://www.help.prusa3d.com/en/article/prusa-specific-g-codes_112173):
            if ((flavor == gcfMarlinLegacy || flavor == gcfMarlinFirmware) &&
                (command == "Tx" || command == "Tc" || command == "T?"))
                return;

            // T-1 is a valid gcode line for RepRap Firmwares (used to deselects all tools) see
            // https://github.com/prusa3d/PrusaSlicer/issues/5677
            if ((flavor != gcfRepRap && flavor != gcfSprinter) || eid != -1) {
                if (m_current_extruder != static_cast<uint16_t>(0))
                    flush();
                m_current_extruder = static_cast<uint16_t>(0);
            }
        } else {
            if(m_current_extruder != static_cast<uint16_t>(eid))
                flush();
            m_current_extruder = static_cast<uint16_t>(eid);
        }
    }
}

// copy/pasted from gcodeprocessor
void PressureAdvance::_process_M204(const GCodeReader::GCodeLine &line)
{
    float value;
    if (line.raw().front() == ';') {
        size_t S_pos = line.raw().find("S");
        assert(S_pos != std::string::npos);
        if (S_pos < line.raw().size()) {
            value = atof(line.raw().c_str() + S_pos + 1);
            if (value > 0) {
                m_current_acceleration = value;
                m_current_deceleration = m_current_acceleration;
            }
        }
    }else if (line.has_value('S', value)) {
        // Legacy acceleration format. This format is used by the legacy Marlin, MK2 or MK3 firmware
        // It is also generated by PrusaSlicer to control acceleration per extrusion type
        // (perimeters, first layer etc) when 'Marlin (legacy)' flavor is used.
        if (value > 0) {
            m_current_acceleration = value;
            m_current_deceleration = m_current_acceleration;
        }
    } else {
        // New acceleration format, compatible with the upstream Marlin.
        if (line.has_value('P', value) && value > 0)
            m_current_acceleration = value;
        m_current_deceleration         = m_current_acceleration;
        m_current_retract_acceleration = m_current_acceleration;
        if (line.has_value('R', value) && value > 0)
            m_current_retract_acceleration = value;
    }
}

void PressureAdvance::_process_ACTIVATE_EXTRUDER(const std::string_view cmd)
{
    if (size_t cmd_end = cmd.find("ACTIVATE_EXTRUDER"); cmd_end != std::string::npos) {
        bool   error              = false;
        size_t extruder_pos_start = cmd.find("EXTRUDER", cmd_end + std::string_view("ACTIVATE_EXTRUDER").size()) +
                                    std::string_view("EXTRUDER").size();
        assert(cmd[extruder_pos_start - 1] == 'R');
        if (extruder_pos_start != std::string::npos) {
            // remove next char until '-' or [0-9]
            while (
                extruder_pos_start < cmd.size() &&
                (cmd[extruder_pos_start] == ' ' || cmd[extruder_pos_start] == '=' || cmd[extruder_pos_start] == '\t'))
                ++extruder_pos_start;
            size_t extruder_pos_end = extruder_pos_start + 1;
            while (extruder_pos_end < cmd.size() && cmd[extruder_pos_end] != ' ' && cmd[extruder_pos_end] != '\t' &&
                   cmd[extruder_pos_end] != '\r' && cmd[extruder_pos_end] != '\n')
                ++extruder_pos_end;
            std::string_view extruder_name = cmd.substr(extruder_pos_start, extruder_pos_end - extruder_pos_start);
            // we have a "name". It may be whatever or "extruder" + X
            for (const Extruder &extruder : m_writer.extruders()) {
                if (m_writer.config.tool_name.get_at(extruder.id()) == extruder_name) {
                    if(m_current_extruder != static_cast<uint16_t>(extruder.id()))
                        flush();
                    m_current_extruder = static_cast<uint16_t>(extruder.id());
                    return;
                }
            }
            std::string extruder_str("extruder");
            if (extruder_str == extruder_name) {
                if(m_current_extruder != static_cast<uint16_t>(0))
                    flush();
                m_current_extruder = static_cast<uint16_t>(0);
                return;
            }
            for (const Extruder &extruder : m_writer.extruders()) {
                if (extruder_str + std::to_string(extruder.id()) == extruder_name) {
                    if(m_current_extruder != static_cast<uint16_t>(extruder.id()))
                        flush();
                    m_current_extruder = static_cast<uint16_t>(extruder.id());
                    return;
                }
            }
        }
        BOOST_LOG_TRIVIAL(error) << "invalid ACTIVATE_EXTRUDER gcode command: '" << cmd
                                 << "', ignored by the fam mover post-process.";
    }
}

void PressureAdvance::update_last_move(size_t last_move_idx, double junction_axle_speed)
{
    //std::cout << "update last\n";
    size_t      iteration           = 0;
    double      loop_junction_speed = junction_axle_speed;
    // note: be sure to relink this pointer evrytime m_buffer is modified
    BufferData *loop_last_move      = &m_buffer[last_move_idx];
    size_t      loop_last_idx       = last_move_idx;
    assert(loop_junction_speed <= loop_last_move->speed_end);
    BufferData *debug_check_no_infinite_loop = nullptr;
    loop_last_move->check_integrity();
    // if loop_junction_speed < end_speed
    while (loop_last_move->speed_end > loop_junction_speed) {
        iteration++;
        // check no infinite loop
        assert(debug_check_no_infinite_loop != loop_last_move);
        debug_check_no_infinite_loop = loop_last_move;
        // check it's only acce, only decel, or only flat.
        loop_last_move->check_integrity();
        // if start speed == end_speed or (start_speed >= loop_junction_speed && start_speed < end_speed),
        // it means, it's flat OR accelerating, but need to decelerate at some point to junction speed.
        if (loop_last_move->speed_start == loop_last_move->speed_end ||
            (loop_last_move->speed_start < loop_last_move->speed_end &&
             loop_last_move->speed_start >= loop_junction_speed)) {
            // if there is enough time to decelerate
            const double max_delta_speed_decel = PressureAdvance_func::compute_delta_speed(
                loop_last_move->max_axle_acceleration,
                loop_junction_speed, // because loop_junction_speed < loop_last_move->speed_start
                loop_last_move->max_axle);
            if (max_delta_speed_decel >= (loop_last_move->speed_start - loop_junction_speed)) {
                const double dist_deceleration = PressureAdvance_func::compute_distance(loop_last_move->decceleration,
                                                                                  loop_junction_speed,
                                                                                  loop_last_move->speed_start);
                // double      time_deceleration = ((loop_last_move->speed_start + loop_junction_speed) /
                // 2) / dist_deceleration; assert(loop_last_move->speed_start / loop_last_move->distance +
                // EPSILON > time_deceleration);
                assert(loop_last_move->max_axle + EPSILON > dist_deceleration);
                if (dist_deceleration / loop_last_move->max_axle > 0.99) {
                    assert(dist_deceleration / loop_last_move->max_axle < 1 + EPSILON);
                    // if almost all the dist is used to decelerate, just approximate.
                    loop_last_move->speed_end = loop_junction_speed;
                    double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
                    double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
                    loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                        std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                        std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                        " ; decel direct";
                    //std::cout << " f/a -> d\n";
                    loop_last_move->check_integrity();
                } else {
                    // split, set the second to decelerate, insert the second
                    BufferData check_data = *loop_last_move;
                    remove_from_buffer(*loop_last_move);
                    m_buffer.insert(m_buffer.begin() + loop_last_idx + 1, m_buffer[loop_last_idx]);
                    loop_last_idx++;
                    // update pointer, as the buffer have changed with insert.
                    // note: const_speed_part may be not constant (accelerating), but it's not very important.
                    BufferData &first_part       = m_buffer[loop_last_idx - 1];
                    loop_last_move               = &m_buffer[loop_last_idx];
                    double assert_check_old_dist = loop_last_move->max_axle;
                    if (first_part.speed_start == first_part.speed_end) {
                        // first part has constant speed
                        double ratio_second_part = (dist_deceleration / loop_last_move->max_axle);
                        // update pos & delta
                        first_part.ratio(1 - ratio_second_part);
                        loop_last_move->ratio(ratio_second_part);
                        loop_last_move->x += first_part.dx;
                        loop_last_move->y += first_part.dy;
                        loop_last_move->z += first_part.dz;
                        loop_last_move->e += first_part.de;
                        assert(std::isnan(check_data.x) || check_data.x == first_part.x);
                        assert(std::isnan(check_data.x) || std::abs(first_part.x + first_part.dx - loop_last_move->x) < EPSILON);
                        assert(std::abs(first_part.dx + loop_last_move->dx - check_data.dx) < EPSILON);
                        assert(std::abs(loop_last_move->max_axle - dist_deceleration) < EPSILON);
                        assert(std::abs(first_part.max_axle + loop_last_move->max_axle - assert_check_old_dist) < EPSILON);
                        // change speed_end
                        loop_last_move->speed_end = loop_junction_speed;
                        assert(first_part.speed_start == first_part.speed_end);
                        assert(first_part.speed_start == loop_last_move->speed_start);
                        double first_part_time_s = first_part.max_axle / first_part.speed_start;
                        double first_part_e_speed = first_part.speed_end * first_part.de / first_part.max_axle;
                        assert(std::abs(first_part.de / first_part_time_s - first_part_e_speed) < EPSILON /100);
                        first_part.comment = std::string(" ;F") + std::to_string(int(first_part.speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(first_part_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(first_part_time_s*1000000)) +
                            " ; steady " + std::to_string(int(first_part.speed_start * 60));
                        double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
                        double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
                        loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                            std::string(" ; decel ") + std::to_string(int(loop_last_move->speed_start * 60)) +
                                                  std::string("->") + std::to_string(int(loop_last_move->speed_end * 60));
                                                  //std::string(",d=") + std::to_string(Vec2d(loop_last_move->dx, loop_last_move->dy).norm())+
                                                  //std::string(",px=") + std::to_string(loop_last_move->x + loop_last_move->dx)+
                                                  //std::string(",ma=") + std::to_string(dist_deceleration);
                        //std::cout << " f -> f & d\n";
                        first_part.check_integrity();
                        loop_last_move->check_integrity();
                        first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                    } else /*if (loop_last_move->speed_start < loop_last_move->speed_end)*/ {
                        // first part is accelerating, second decelerating
                        double dist_remaining  = loop_last_move->max_axle - dist_deceleration;
                        double min_accel_decel = std::min(loop_last_move->max_axle_acceleration, loop_last_move->decceleration);
                        double max_delta_speed_accel_decel =
                            PressureAdvance_func::compute_delta_speed(min_accel_decel, loop_last_move->speed_start,
                                                                      dist_remaining / 2);
                        // check: there was just enough space for accelerating to end speed, now there is
                        // not enough space for it.
                        assert(max_delta_speed_accel_decel < loop_last_move->speed_end - loop_last_move->speed_start);
                        double ratio_first_part = (dist_remaining / 2) / loop_last_move->max_axle;
                        // update pos & delta
                        first_part.ratio(ratio_first_part);
                        loop_last_move->ratio(1 - ratio_first_part);
                        loop_last_move->x += first_part.dx;
                        loop_last_move->y += first_part.dy;
                        loop_last_move->z += first_part.dz;
                        loop_last_move->e += first_part.de;
                        assert(std::isnan(check_data.x) || check_data.x == first_part.x);
                        assert(std::isnan(check_data.x) || first_part.x + first_part.dx == loop_last_move->x);
                        assert(std::abs(first_part.dx + loop_last_move->dx - check_data.dx) < EPSILON);
                        assert(first_part.max_axle <= loop_last_move->max_axle);
                        assert(std::abs(first_part.max_axle - dist_remaining / 2) < EPSILON);
                        assert(std::abs(loop_last_move->max_axle - dist_deceleration - dist_remaining / 2) < EPSILON);
                        assert(std::abs(first_part.max_axle + loop_last_move->max_axle - assert_check_old_dist) <
                               EPSILON);
                        // update speed change (accel->decel)
                        first_part.speed_end        = first_part.speed_start + max_delta_speed_accel_decel;
                        loop_last_move->speed_start = first_part.speed_end;
                        loop_last_move->speed_end   = loop_junction_speed;
                        double first_part_time_s = std::abs(first_part.speed_end - first_part.speed_start) / first_part.max_axle_acceleration;
                        double first_part_e_speed = first_part.speed_end * first_part.de / first_part.max_axle;
                        first_part.comment = std::string(" ;F") + std::to_string(int(first_part.speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(first_part_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(first_part_time_s*1000000)) +
                            std::string(" ; accel ") + std::to_string(int(first_part.speed_start * 60)) +
                                          std::string("->") + std::to_string(int(first_part.speed_end * 60));
                        double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
                        double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
                        loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                            std::string(" ; decel ") + std::to_string(int(loop_last_move->speed_start * 60)) +
                                          std::string("->") + std::to_string(int(loop_last_move->speed_end * 60));
                        //std::cout << " a -> a & d\n";
                        first_part.check_integrity();
                        loop_last_move->check_integrity();
                        first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                    }
                    assert(first_part.speed_end == loop_last_move->speed_start);
                    put_in_buffer(first_part);
                    put_in_buffer(*loop_last_move);
                }
            } else {
                // change the start speed to the available speed
                double start_speed = loop_junction_speed + max_delta_speed_decel;
                assert(start_speed > loop_junction_speed);
                assert(start_speed < loop_last_move->speed_start);
                loop_last_move->speed_start = start_speed;
                loop_last_move->speed_end   = loop_junction_speed;
                loop_last_move->check_integrity();
                loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                // get previous -> recursive with junction speed = min speed
                size_t previous_idx = get_last_move_idx(loop_last_idx);
                assert(loop_last_idx != previous_idx);
                double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
                double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
                loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                    std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                    std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                    std::string(" ; decel ") + std::to_string(int(loop_last_move->speed_start * 60)) +
                                          std::string("->") + std::to_string(int(loop_last_move->speed_end * 60));
                if (previous_idx < m_buffer.size()) {
                    // it exists. iterate the loop
                    loop_junction_speed = start_speed;
                    loop_last_idx       = previous_idx;
                    loop_last_move      = &m_buffer[loop_last_idx];
                } else {
                    std::cout << "error\n";
                    // buffer not wide enough. weird.
                    BOOST_LOG_TRIVIAL(warning) << "Warning: not enough buffer for pressure control.";
                    break;
                }
            }
        } else if (loop_last_move->speed_start > loop_last_move->speed_end) {
            // if start_speed > end_speed : decelerating
            // lower speed, recursive with new start_speed
            double delta_speed = loop_last_move->speed_end - loop_junction_speed;
            assert(delta_speed > 0);
            loop_last_move->speed_start -= delta_speed;
            loop_last_move->speed_end = loop_junction_speed;
            loop_last_move->check_integrity();
            loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
            double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
            double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
            loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                std::string(" ; decel ") + std::to_string(int(loop_last_move->speed_start * 60)) +
                                     std::string("->") + std::to_string(int(loop_last_move->speed_end * 60));
            //std::cout << " d -> d (" << loop_last_idx << ") , iter(";
            // iterate
            size_t previous_idx = get_last_move_idx(loop_last_idx);
            assert(loop_last_idx != previous_idx);
            if (previous_idx < m_buffer.size()) {
                // it exists. iterate the loop
                loop_junction_speed = loop_last_move->speed_start;
                loop_last_idx       = previous_idx;
                loop_last_move      = &m_buffer[loop_last_idx];
                //std::cout << loop_last_idx << ")\n";
            } else {
                std::cout << "error\n";
                // buffer not wide enough. weird.
                BOOST_LOG_TRIVIAL(warning) << "Warning: not enough buffer for pressure control.";
                break;
            }
        } else {
            // if start_speed < end_speed : accelerating
            assert(loop_last_move->speed_start < loop_last_move->speed_end);
            // if start_speed < loop_junction_speed
            if (loop_last_move->speed_start < loop_junction_speed) {
                // find best spot for switching acceleration (or just accelerate to our junction speed)
                double dist_accel_start_to_junction =
                    PressureAdvance_func::compute_distance(loop_last_move->max_axle_acceleration, loop_last_move->speed_start,
                                                           loop_junction_speed);
                assert(dist_accel_start_to_junction < loop_last_move->max_axle);
                double dist_remaining  = loop_last_move->max_axle - dist_accel_start_to_junction;
                double min_accel_decel = std::min(loop_last_move->max_axle_acceleration, loop_last_move->decceleration);
                double max_delta_speed_accel_decel = PressureAdvance_func::compute_delta_speed(min_accel_decel,
                                                                                               loop_junction_speed,
                                                                                               dist_remaining / 2);
                assert(loop_junction_speed + max_delta_speed_accel_decel <= loop_last_move->speed_end);
                // split in two:: accel & decel
                double     assert_check_old_dist = loop_last_move->max_axle;
                BufferData check_data            = *loop_last_move;
                remove_from_buffer(*loop_last_move);
                m_buffer.insert(m_buffer.begin() + loop_last_idx + 1, m_buffer[loop_last_idx]);
                loop_last_idx++;
                // update pointer, as the buffer have changed with insert.
                // note: const_speed_part may be not constant (accelerating), but it's not very important.
                BufferData &first_part = m_buffer[loop_last_idx - 1];
                loop_last_move         = &m_buffer[loop_last_idx];
                assert(assert_check_old_dist == first_part.max_axle);
                assert(assert_check_old_dist == loop_last_move->max_axle);
                assert(max_delta_speed_accel_decel < loop_last_move->speed_end - loop_last_move->speed_start);
                double ratio_second_part = dist_remaining / (2 * loop_last_move->max_axle);
                // update pos & delta
                first_part.ratio(1 - ratio_second_part);
                first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                loop_last_move->ratio(ratio_second_part);
                loop_last_move->x += first_part.dx;
                loop_last_move->y += first_part.dy;
                loop_last_move->z += first_part.dz;
                loop_last_move->e += first_part.de;
                loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                assert(std::isnan(check_data.x) || check_data.x == first_part.x);
                assert(std::isnan(check_data.x) || std::abs(first_part.x + first_part.dx - loop_last_move->x) < EPSILON);
                assert(std::abs(first_part.dx + loop_last_move->dx - check_data.dx) < EPSILON);
                assert(std::abs(loop_last_move->max_axle - dist_remaining / 2) < EPSILON);
                assert(std::abs(first_part.max_axle + loop_last_move->max_axle - assert_check_old_dist) < EPSILON);
                // update speed change (accel->decel)
                first_part.speed_end        = loop_junction_speed + max_delta_speed_accel_decel;
                loop_last_move->speed_start = first_part.speed_end;
                loop_last_move->speed_end   = loop_junction_speed;
                put_in_buffer(first_part);
                put_in_buffer(*loop_last_move);
                double first_part_time_s = std::abs(first_part.speed_end - first_part.speed_start) / first_part.max_axle_acceleration;
                double first_part_e_speed = first_part.speed_end * first_part.de / first_part.max_axle;
                first_part.comment = std::string(" ;F") + std::to_string(int(first_part.speed_end * 60)) +
                    std::string(" ;H") + std::to_string(int(first_part_e_speed * 60000)/1000.) +
                    std::string(" ;T") + std::to_string(int(first_part_time_s*1000000)) +
                    std::string(" ; accel ") + std::to_string(int(first_part.speed_start * 60)) +
                                     std::string("->") + std::to_string(int(first_part.speed_end * 60));
                double loop_last_move_time_s = std::abs(loop_last_move->speed_end - loop_last_move->speed_start) / loop_last_move->max_axle_acceleration;
                double loop_last_move_e_speed = loop_last_move->speed_end * loop_last_move->de / loop_last_move->max_axle;
                loop_last_move->comment = std::string(" ;F") + std::to_string(int(loop_last_move->speed_end * 60)) +
                    std::string(" ;H") + std::to_string(int(loop_last_move_e_speed * 60000)/1000.) +
                    std::string(" ;T") + std::to_string(int(loop_last_move_time_s*1000000)) +
                    std::string(" ; decel ") + std::to_string(int(loop_last_move->speed_start * 60)) +
                                     std::string("->") + std::to_string(int(loop_last_move->speed_end * 60));
                //std::cout << " a -> a & d (2)\n";
                first_part.check_integrity();
                loop_last_move->check_integrity();
                first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                loop_last_move->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
            } else {
                // else, start_speed >= loop_junction_speed -> already done in first case
                assert(false);
            }
        }
    }
}

void PressureAdvance::_process_gcode_line(GCodeReader &reader, const GCodeReader::GCodeLine &line)
{
    // processes 'normal' gcode lines
    bool        need_flush = false;
    std::string cmd(line.cmd());
    double      time          = 0; // no accel/decel
    double      fly_distance  = 0; // no accel/decel
    double      max_axle_dist = 0; // no accel/decel
    double      max_axle_xyz_dist = 0; // no accel/decel
    double      written_speed = 0;
    double      is_G123 = false;
    if (line.raw().size() > 3 && line.raw().front() == ';' && line.raw()[1] == 'M') {
        size_t end_cmd = line.raw().find(' ');
        cmd = line.raw().substr(1, end_cmd == std::string::npos ? line.raw().size() - 1 : end_cmd -1);
    }
    if (cmd.length() > 1) {
        if (line.has_f()) {
            written_speed = m_current_speed = line.f() / 60.0f;
        } else {
            written_speed = m_current_speed;
        }
        switch (::toupper(cmd[0])) {
        case 'A': _process_ACTIVATE_EXTRUDER(line.raw()); break;
        case 'T':
        case 't': _process_T(cmd); break;
        case 'G': {
            if (::atoi(&cmd[1]) == 1 || ::atoi(&cmd[1]) == 0) {
                double distx = line.dist_X(reader);
                double disty = line.dist_Y(reader);
                double distxy = Vec2d(distx, disty).norm();
                double distz = line.dist_Z(reader);
                double diste = line.dist_E(reader);
                if (m_speedxy) {
                    max_axle_dist = std::max(max_axle_dist, std::abs(distxy));
                } else {
                    max_axle_dist = std::max(max_axle_dist, std::abs(distx));
                    max_axle_dist = std::max(max_axle_dist, std::abs(disty));
                }
                max_axle_dist = std::max(max_axle_dist, std::abs(distz));
                max_axle_xyz_dist = max_axle_dist;
                max_axle_dist = std::max(max_axle_dist, std::abs(diste));
                fly_distance  = distx * distx + disty * disty + distz * distz;
                if (fly_distance > 0) {
                    fly_distance = std::sqrt(fly_distance);
                }
                if(max_axle_dist > 0)
                    time = max_axle_dist / written_speed;
                is_G123 = true;
            } else if (::atoi(&cmd[1]) == 2 || ::atoi(&cmd[1]) == 3) {
                // TODO: compute real dist
                double distx = line.dist_X(reader);
                double disty = line.dist_Y(reader);
                double distxy = Vec2d(distx, disty).norm();
                double distz = line.dist_Z(reader);
                double diste = line.dist_E(reader);
                fly_distance = distxy;
                if (m_speedxy) {
                    max_axle_dist = std::max(max_axle_dist, std::abs(distxy));
                } else {
                    max_axle_dist = std::max(max_axle_dist, std::abs(distx));
                    max_axle_dist = std::max(max_axle_dist, std::abs(disty));
                }
                max_axle_dist = std::max(max_axle_dist, std::abs(distz));
                max_axle_xyz_dist = max_axle_dist;
                max_axle_dist = std::max(max_axle_dist, std::abs(diste));
                assert(distz == 0);
                if(max_axle_dist > 0)
                    time = max_axle_dist / written_speed;
                BOOST_LOG_TRIVIAL(warning) << "Pressure advance doesn't support arcs (G2/G3).";
                is_G123 = true;
            } else if (::atoi(&cmd[1]) == 9 || ::atoi(&cmd[1]) == 2) {
                // G92: reset position
                m_back_buffer_e       = line.e();
                m_back_buffer_e_reset = true;
            }
            break;
        }
        case 'M': {
            if (cmd.length() > 3 && cmd[1] == '2' && cmd[2] == '0' && cmd[3] == '4') {
                _process_M204(line);
            }
        }
        default: {
            if (cmd.length() > 10 && (cmd[0] == 'A' || cmd[0] == 'B') && cmd.at(1) == '[' &&
                line.raw().find("SET_POSITION") != std::string::npos &&
                line.raw().find("POS=") != std::string::npos) {
                char   axis          = cmd.at(0);
                size_t str_pos_start = line.raw().find("POS=") + 4;
                assert(str_pos_start > 0 && str_pos_start != std::string::npos);
                size_t str_pos_end = line.raw().find("]", str_pos_start);
                assert(str_pos_end > str_pos_start && str_pos_end != std::string::npos);
                double pos;
                if (!PressureAdvance_func::parse_double(line.raw().substr(str_pos_start, str_pos_end - str_pos_start),
                                                        pos)) {
                    BOOST_LOG_TRIVIAL(error)
                        << "GCodeProcessor encountered an invalid value for position set (" << line.raw() << ").";
                    m_back_buffer_e = 0;
                }
                if (axis >= 'A' && axis < 'F') {
                    m_back_buffer_e       = pos;
                    m_back_buffer_e_reset = true;
                }
            }
        }
        }
    } else {
        if (!line.raw().empty() && line.raw().front() == ';') {
            if (line.raw().size() > 10 && line.raw().rfind(";TYPE:", 0) == 0) {
                // get the type of the next extrusions
                std::string extrusion_string = line.raw().substr(6, line.raw().size() - 6);
                current_role                 = ExtrusionEntity::string_to_role(extrusion_string);
            }
            if (line.raw().size() > 16) {
                if (line.raw().rfind("; custom gcode", 0) != std::string::npos)
                    if (line.raw().rfind("; custom gcode end", 0) != std::string::npos)
                        m_is_custom_gcode = false;
                    else
                        m_is_custom_gcode = true;
                if (line.raw().rfind("; PRESSURE_FACTOR=", 0) != std::string::npos) {
                    size_t      strsize         = std::string("; PRESSURE_FACTOR=").size();
                    std::string pressure_number = line.raw().substr(strsize, line.raw().size() - strsize);
                    if (pressure_number.empty()) {
                        this->pressure_factor_override = std::numeric_limits<double>::quiet_NaN();
                    } else {
                        char *      endptr;
                        const char *c_str_number = pressure_number.c_str();
                        double      value        = std::strtof(c_str_number, &endptr);
                        if (c_str_number == endptr) {
                            this->pressure_factor_override = std::numeric_limits<double>::quiet_NaN();
                        } else {
                            assert(value >= 0);
                            this->pressure_factor_override = value;
                        }
                    }
                }
            }
        }
    }

    if (time >= 0) {
        assert(max_axle_dist >= 0);
        m_buffer.emplace_back(
            //BufferData(
                line.raw(), max_axle_dist, max_axle_xyz_dist, written_speed, m_current_acceleration, m_current_deceleration
                //)
            );
        assert((max_axle_dist == 0 && time == 0) || std::abs(max_axle_dist - written_speed * time) < EPSILON);
        BufferData *new_data = &m_buffer.back();
        put_in_buffer(*new_data);
        if (line.has(Axis::X)) {
            new_data->x  = reader.x();
            new_data->dx = line.dist_X(reader);
        }
        if (line.has(Axis::Y)) {
            new_data->y  = reader.y();
            new_data->dy = line.dist_Y(reader);
        }
        if (m_speedxy && (line.has(Axis::X) || line.has(Axis::Y))) {
            new_data->dxy = Vec2d(new_data->dx, new_data->dy).norm();
        }
        if (line.has(Axis::Z)) {
            new_data->z  = reader.z();
            new_data->dz = line.dist_Z(reader);
        }
        if (line.has(Axis::E)) {
            new_data->e = reader.e();
            new_data->m_e_char = line.e_char();
            if (m_relative_e) {
                new_data->de = line.e();
                // GCode reader doesn't know it's relative extrusion, we have to do it ourself.
                // assert(new_data->e == 0);
                new_data->e = 0;
            } else {
                new_data->de = line.dist_E(reader);
                if (m_back_buffer_e_reset) {
                    // G92 : reset e
                    m_back_buffer_e_reset   = false;
                    new_data->after_e_reset = true;
                    // de : from the reset position (m_back_buffer_e)
                    new_data->de = new_data->de - m_back_buffer_e;
                }
            }
        }
        if (line.has_f()) {
            new_data->f = written_speed * 60;
        }
        assert(new_data->dx == 0 || reader.x() == new_data->x);
        assert(new_data->dx == 0 || std::abs(reader.x() + new_data->dx - line.x()) < 0.00001f);
        assert(new_data->dy == 0 || reader.y() == new_data->y);
        assert(new_data->dy == 0 || std::abs(reader.y() + new_data->dy - line.y()) < 0.00001f);
        assert(new_data->de == 0 || (m_relative_e ? 0 : reader.e()) == new_data->e);
        assert(new_data->de == 0 || std::abs((m_relative_e ? 0.f : reader.e()) + new_data->de - line.e()) < 0.00001f);
        // assert(new_data->de == 0 ||(relative_e?0.f:reader.e()) + new_data->de == line.e());

        // ensure the speed is in limits
        double x_min_time = std::abs(new_data->dx) / max_x_speed;
        double y_min_time = std::abs(new_data->dy) / max_y_speed;
        double xy_min_time = std::abs(new_data->dxy) / max_x_speed;
        double z_min_time = std::abs(new_data->dz) / max_z_speed;
        double e_min_time = std::abs(new_data->de) / max_e_speed;
        double real_time  = time;
        double max_accel_s2 = new_data->acceleration / (new_data->max_axle != 0 ? std::abs(new_data->max_axle) : 1.);
        if (m_speedxy) {
            if (xy_min_time > 0) {
                if (xy_min_time > time)
                    real_time = xy_min_time;
                const double minxy_accel_s2 = std::min(max_x_acceleration / std::abs(new_data->dx),
                                                       max_y_acceleration / std::abs(new_data->dy));
                max_accel_s2 = std::min(max_accel_s2, minxy_accel_s2);
            }
        } else {
            if (x_min_time > 0) {
                if (x_min_time > time)
                    real_time = x_min_time;
                max_accel_s2 = std::min(max_accel_s2, max_x_acceleration / std::abs(new_data->dx));
            }
            if (y_min_time > 0) {
                if (y_min_time > time)
                    real_time = y_min_time;
                max_accel_s2 = std::min(max_accel_s2, max_y_acceleration / std::abs(new_data->dy));
            }
        }
        if (z_min_time > 0) {
            if (z_min_time > time)
                real_time = z_min_time;
            max_accel_s2 = std::min(max_accel_s2, max_z_acceleration / std::abs(new_data->dz));
        }
        if (e_min_time > 0) {
            if (e_min_time > time)
                real_time = e_min_time;
            max_accel_s2 = std::min(max_accel_s2, max_e_acceleration / std::abs(new_data->de));
        }
        //new_data->max_xyz_axle_acceleration = max_accel_s2 * (new_data->max_axle_xyz != 0 ? new_data->max_axle_xyz : 1.);
        assert(max_accel_s2 >= 0);
        assert(new_data->max_axle != 0 || max_accel_s2 == new_data->acceleration);
        new_data->max_axle_acceleration = max_accel_s2 * (new_data->max_axle != 0 ? std::abs(new_data->max_axle) : 1.);

        assert(real_time >= time);
        if (real_time > time) {
            // also update speed
            assert(new_data->speed_start == new_data->speed_end);
            assert(std::abs((new_data->max_axle / new_data->speed_end) - time) < EPSILON);
            new_data->speed_start = new_data->speed_end = new_data->speed_start * time / real_time;
            // update 'time'
            time = real_time;
        }
        BufferData debug_copy_new_data = *new_data;

        // if not extruding: consider 0 speed. set everything to 0 speed
        if ((std::isnan(new_data->e) || (new_data->e == 0 && new_data->de == 0))) {
            new_data->speed_start   = 0;
            new_data->speed_end     = 0;
            new_data->acceleration  = 0;
            new_data->decceleration = 0;
            //std::cout << "zero speed for: " <<new_data->raw<<"\n";
        }
        if (new_data->is_useful_move() || is_G123) {
            // get latest move
            size_t      last_idx  = get_last_move_idx(m_buffer.size() - 1);
            BufferData *last_move = nullptr;
            if (last_idx < m_buffer.size()) {
                last_move = &m_buffer[last_idx];
                //std::cout << "update latest move: " <<last_move->raw<<"\n";
            }
            // if extruding:
            // if not moving, then ignore (0 speed)
            double junction_speed;
            double angle = -1;
            if ((new_data->max_axle_xyz == 0) || (std::isnan(new_data->e) || (new_data->e == 0 && new_data->de == 0))) {
                new_data->speed_start = 0;
                new_data->speed_end   = 0;
                junction_speed        = 0;
                //std::cout << "zero speed for me: " <<new_data->raw<<"\n";
            } else {
                // compute junction_speed
                if (last_idx >= m_buffer.size() ||
                    (std::isnan(last_move->e) || (last_move->e == 0 && last_move->de == 0)) ||
                    last_move->max_axle_xyz == 0) {
                    // if last one whasn't extruding (0), then junction_speed = 0
                    junction_speed = 0;
                } else {
                    // else, compute junction_speed via the angle between the two (180 -> 90° : 1->0mm/s, <90: 0mm/s)
                    Point  p_last_begin(-last_move->dx, -last_move->dy);
                    Point  p_junction(0, 0);
                    Point  p_current_end(new_data->dx, new_data->dy);
                    angle = p_junction.ccw_angle(p_last_begin, p_current_end);
                    if (angle > PI)
                        angle = (2 * PI) - angle;
                    assert(angle >= 0 && angle <= PI);
                    assert(last_move->speed_end > 0);
                    assert(new_data->speed_end > 0);
                    if (angle < PI / 2) {
                        junction_speed = 0;
                    } else {
                        // junction_speed = min(junction_speed, previous.end_speed)
                        junction_speed = std::min(last_move->speed_end, new_data->speed_end);
                        junction_speed = junction_speed * ((angle - PI / 2) / (PI / 2));
                    }
                }
            }
            if (new_data->is_useful_move()) {
                // use junction_speed as our start speed.
                new_data->speed_start = junction_speed;
            } else {
                assert(junction_speed == 0);
            }

            // Update buffer to get to the current junction_speed.
            if (last_move && new_data->is_useful_move()) {
                update_last_move(last_idx, junction_speed);
                //m_buffer.back().raw += std::string(";a=") + std::to_string(angle) + std::string(";js=") + std::to_string(junction_speed*60);
            }

            //need to refresh new_data_idx pointer, as the buffer may have been modified
            new_data = &m_buffer.back();

            // now update our section
            if (new_data->is_useful_move()) {
                // if junction_speed == current speed: nothing to do
                if (new_data->speed_start == new_data->speed_end) {
                    assert(junction_speed == new_data->speed_end);
                    new_data->check_integrity();
                    new_data->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                } else if (new_data->speed_start < new_data->speed_end) {
                    // if speed > current speed: acceleration
                    // check distance for acceleration
                    const double dist_axle_accel = PressureAdvance_func::compute_distance(new_data->max_axle_acceleration,
                                                                               new_data->speed_start,
                                                                               new_data->speed_end);
                    if (dist_axle_accel >= new_data->max_axle) {
                        // then reduce end speed
                        const double max_delta_axle_speed = PressureAdvance_func::compute_delta_speed(new_data->max_axle_acceleration,
                                                                                           new_data->speed_start,
                                                                                           new_data->max_axle);
                        // change current speed
                        new_data->speed_end = new_data->speed_start + max_delta_axle_speed;
                        //std::cout << "set my speed to " << int(new_data->speed_end * 60) << ": " << new_data->raw << "\n";
                        new_data->check_integrity();
                        new_data->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        const double new_data_time_s = std::abs(max_delta_axle_speed) / new_data->max_axle_acceleration;
                        const double new_data_e_speed = new_data->speed_end * new_data->de / new_data->max_axle;
                        new_data->comment = std::string(" ;F") + std::to_string(int(new_data->speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(new_data_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(new_data_time_s*1000000)) +
                            std::string(" ; accel ") +
                                            std::to_string(int(new_data->speed_start * 60)) + std::string(" -> ") +
                                            std::to_string(int(new_data->speed_end * 60));
                    } else {
                        // if enough time
                        // then split, with the first part with just enough time to accelerate
                        const double     assert_check_old_dist = new_data->max_axle;
                        BufferData check_data            = *new_data;
                        remove_from_buffer(*new_data);
                        m_buffer.push_back(*new_data);
                        assert(m_buffer.size() > 1);
                        BufferData &first_part = m_buffer[m_buffer.size() - 2];
                        new_data               = &m_buffer.back();
                        assert(assert_check_old_dist == first_part.max_axle);
                        assert(assert_check_old_dist == new_data->max_axle);
                        // first part accel, second is constant speed
                        const double ratio_first_part = (dist_axle_accel / new_data->max_axle);
                        // update pos & delta
                        first_part.ratio(ratio_first_part);
                        first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        new_data->ratio(1 - ratio_first_part);
                        new_data->x += first_part.dx;
                        new_data->y += first_part.dy;
                        new_data->z += first_part.dz;
                        new_data->e += first_part.de;
                        new_data->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        assert(std::isnan(check_data.x) || check_data.x == first_part.x);
                        assert(std::isnan(check_data.x) || std::abs(first_part.x + first_part.dx - new_data->x) < EPSILON);
                        assert(std::abs(first_part.dx + new_data->dx - check_data.dx) < EPSILON);
                        assert(std::abs(first_part.max_axle - dist_axle_accel) < EPSILON);
                        assert(std::abs(first_part.max_axle + new_data->max_axle - assert_check_old_dist) < EPSILON);
                        // set speed
                        assert(first_part.speed_start == junction_speed);
                        assert(first_part.speed_end > first_part.speed_start);
                        assert(first_part.speed_end == new_data->speed_end);
                        new_data->speed_start = new_data->speed_end;
                        //std::cout << "set my end speed to " << int(new_data->speed_end * 60) << ": " << new_data->raw << "\n";
                        put_in_buffer(first_part);
                        put_in_buffer(*new_data);
                        const double first_part_time_s = std::abs(first_part.speed_end - first_part.speed_start) / first_part.max_axle_acceleration;
                        const double first_part_e_speed = first_part.speed_end * first_part.de;
                        first_part.comment = std::string(" ;F") + std::to_string(int(first_part.speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(first_part_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(first_part_time_s*1000000)) +
                            std::string(" ; accel ") +
                                             std::to_string(int(first_part.speed_start * 60)) + std::string(" -> ") +
                                             std::to_string(int(first_part.speed_end * 60));
                                                  //std::string(",d=") + std::to_string(Vec2d(first_part.dx, first_part.dy).norm())+
                                                  //std::string(",ma=") + std::to_string(dist_axle_accel);
                        const double new_data_time_s = new_data->max_axle / new_data->speed_end;
                        const double new_data_e_speed = new_data->speed_end * new_data->de / new_data->max_axle;
                        assert(std::abs(new_data->de / new_data_time_s - new_data_e_speed) < EPSILON /100);
                        new_data->comment = std::string(" ;F") + std::to_string(int(new_data->speed_end * 60)) +
                            std::string(" ;H") + std::to_string(int(new_data_e_speed * 60000)/1000.) +
                            std::string(" ;T") + std::to_string(int(new_data_time_s*1000000)) +
                            std::string(" ; steady ") + std::to_string(int(new_data->speed_end * 60));
                        first_part.check_integrity();
                        new_data->check_integrity();
                        first_part.update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        new_data->update_axis_values(xyz_decimals, e_decimals, m_relative_e, get_pressure_factor());
                        assert(new_data->speed_start == new_data->speed_end);
                    }
                } else {
                    assert(new_data->speed_start > new_data->speed_end);
                    // if speed < current speed: deceleration
                    // then you have to decelerate before going into this thing.
                    // so it's already the junction speed
                    // so it's not possible to be here
                    assert(false);
                }
            }
        }
    }
    // puts the line back into the gcode
    // if buffer too big, flush it.
    if (time >= 0) {
        if (need_flush && !m_buffer.empty()) {
            size_t last_idx = get_last_move_idx(m_buffer.size());
            if (last_idx < m_buffer.size() && m_buffer[last_idx].speed_end > 0)
                update_last_move(last_idx, 0);
        }
        while (!m_buffer.empty() && (need_flush || (m_buffer_distance_size > m_buffer_distance_span &&
                                                    m_buffer_useful_size > m_buffer_useful_size_span))) {
            write_buffer_data();
        }
    }
#if _DEBUG
    double sum = 0;
    for (auto &data : m_buffer) sum += data.max_axle_xyz;
    assert(std::abs(m_buffer_distance_size - sum) < 0.01);
#endif
}

double PressureAdvance::compute_pressure_change_mm_filament(const BufferData &command)
{
    float pressure_factor = get_pressure_factor();
    // pressure factor increase e by 1 for each 1 mm/s of speed difference
    double speed_delta = command.speed_end;// - command.speed_start;
    return speed_delta * pressure_factor;

    // TODO: real pressure advance, that is just pressure compensation.
    // get current max extruder speed.
    // compute needed extruder speed
    // split in two: one very fast extruder speed (the delta e fully at the start), one normal one (without the delta e)
}

void PressureAdvance::write_buffer_data()
{
    BufferData &frontdata = m_buffer.front();
    if (frontdata.comment.empty() || !this->m_config.gcode_comments.value)
        m_process_output += frontdata.raw  + "\n";
    else
        m_process_output += frontdata.raw + "; " + frontdata.comment + "\n";
    remove_from_buffer(m_buffer.front());
    m_buffer.erase(m_buffer.begin());
}

} // namespace Slic3r
