#ifndef slic3r_GCode_PressureAdvance_hpp_
#define slic3r_GCode_PressureAdvance_hpp_


#include "../libslic3r.h"
#include "../PrintConfig.hpp"
#include "../ExtrusionEntity.hpp"

#include "../Point.hpp"
#include "../GCodeReader.hpp"
#include "../GCodeWriter.hpp"

#include <deque>
#include <regex>


namespace Slic3r {
    
namespace PressureAdvance_func {
    void change_axis_value(std::string &line, char axis, const float new_value, const int decimal_digits);
    double compute_delta_speed(double accel, double initial_speed, double distance);
    double compute_distance(double accel, double initial_speed, double final_speed);
}

class PressureAdvance
{
public:
    class BufferData
    {
    public:
        // raw string, contains end position
        std::string raw;
        // time to go from start to end
        //float time;
        double max_axle; 
        double max_axle_xyz; // speed / time.
        // speed of this line, from start to end.
        double written_speed; // speed written in the line. To know if we need to edit it.
        double speed_start; // previous speed (our speed if stable or decelerating, next speed if accelerating). speed of the biggest axle
        double speed_end; // next speed (our speed if stable or accelerating, next speed if decelerating). speed of the biggest axle
        double acceleration; // written acceleration at this time.
        double max_axle_acceleration; // max acceleration for the biggest axle
        double decceleration; // max decceleration at this time. for the max axle. TODO currently == acceleration by design. need to change the mid-point algo to use it.
        bool after_e_reset = false;
        // start position (not the end that is written in the gcode)
        double x, y, z, e;
        double f; // end speed for this section (if written)
        // delta to go to end position
        double dx = 0, dy = 0, dxy= 0, dz = 0, de = 0;
        char m_e_char; // char of the extruder
        std::string comment;
        BufferData(std::string line, double max_axle, double max_axle_xyz, double speed, double acceleration, double decceleration)
            : raw(line)
            , max_axle(max_axle)
            , max_axle_xyz(max_axle_xyz)
            , written_speed(speed)
            , speed_start(speed)
            , speed_end(speed)
            , acceleration(acceleration)
            , decceleration(decceleration)
        {
            // avoid double \n
            if (!line.empty() && line.back() == '\n')
                line.pop_back();
            x = std::numeric_limits<double>::quiet_NaN();
            y = std::numeric_limits<double>::quiet_NaN();
            z = std::numeric_limits<double>::quiet_NaN();
            e = std::numeric_limits<double>::quiet_NaN();
            f = std::numeric_limits<double>::quiet_NaN();
        }

        void ratio(double ratio) {
            assert(ratio > 0);
            dx = ratio * dx;
            dy = ratio * dy;
            dxy = ratio * dxy;
            dz = ratio * dz;
            de = ratio * de;
            max_axle *= ratio;
            max_axle_xyz *= ratio;
            assert(max_axle_xyz > 0);
        }

        void update_axis_values(uint8_t xyz_decimals, uint8_t e_decimals, bool is_relative_e, double pressure_factor)
        {
            if (!std::isnan(this->x))
                PressureAdvance_func::change_axis_value(this->raw, 'X', this->x + this->dx, xyz_decimals);
            if (!std::isnan(this->y))
                PressureAdvance_func::change_axis_value(this->raw, 'Y', this->y + this->dy, xyz_decimals);
            if (!std::isnan(this->z))
                PressureAdvance_func::change_axis_value(this->raw, 'Z', this->z + this->dz, xyz_decimals);
            if (!std::isnan(this->e)) {
                //compute start & stop speed for extruder.
                double time_start    = this->speed_start <= 0 ? 0 : (max_axle / this->speed_start);
                double e_speed_start = time_start <= 0 ? 0 : (this->de / time_start);
                double time_end      = this->speed_end <= 0 ? 0 : (max_axle / this->speed_end);
                double e_speed_end   = time_end <= 0 ? 0 : (this->de / time_end);
                assert(!std::isnan(e_speed_start));
                assert(!std::isnan(e_speed_end));
                if (is_relative_e) {
                    double e_compute = this->de + this->speed_end * pressure_factor - this->speed_start * pressure_factor;
                    PressureAdvance_func::change_axis_value(this->raw, m_e_char, e_compute, e_decimals);
                } else {
                    double e_compute = this->e + this->de + e_speed_end * pressure_factor;
                    PressureAdvance_func::change_axis_value(this->raw, m_e_char, e_compute, e_decimals);
                }
            }
            //if (!std::isnan(this->f) && is_useful_move() && !std::isnan(this->e) && this->de != 0 && this->speed_end > 0)
            //    PressureAdvance_func::change_axis_value(this->raw, 'F', this->speed_end * 60, 0);
            //else if(speed_end*60 < f + EPSILON)
                //PressureAdvance_func::add_axis_value(this->raw, 'F', this->speed_end * 60, 0);

        }
        void check_integrity() const
        {
            if (this->speed_start < this->speed_end) {
                double accel_dist = PressureAdvance_func::compute_distance(this->acceleration,
                                                                           this->speed_start,
                                                                           this->speed_end);
                assert(std::abs(accel_dist - this->max_axle_xyz) < EPSILON);
            } else if (this->speed_start > this->speed_end) {
                double decel_dist = PressureAdvance_func::compute_distance(this->decceleration,
                                                                           this->speed_end,
                                                                           this->speed_start);
                assert(std::abs(decel_dist - this->max_axle_xyz) < EPSILON ||
                       this->max_axle_xyz / decel_dist > 0.99);
            } else {
                assert(this->max_axle_xyz == 0 || this->speed_end == this->speed_start);
            }
            {
                //check max_axle;
                double max_abs_axle = std::abs(dx);
                max_abs_axle = std::max(max_abs_axle, std::abs(dy));
                max_abs_axle = std::max(max_abs_axle, std::abs(dxy));
                max_abs_axle = std::max(max_abs_axle, std::abs(dz));
                assert(std::abs(max_abs_axle - max_axle_xyz) < EPSILON);
                max_abs_axle = std::max(max_abs_axle, std::abs(de));
                assert(std::abs(max_abs_axle - max_axle) < EPSILON);
            }
        }
        bool is_useful_move() const { return (this->dx != 0 || this->dy != 0 || this->dz != 0 || this->de != 0 || (!std::isnan(this->e) && this->e != 0)); }

        //double get_max_xyz_axle_speed_ratio(bool is_cartesian) {
        //    if (is_cartesian) {
        //        int nbaxles = dx != 0 ? 1 : 0;
        //        if(dy) nbaxles++;
        //        if(dz) nbaxles++;
        //        if (nbaxles <= 1) {
        //            return 1.;
        //        } else if (nbaxles = 2 && dz == 0) {
        //            //xy
        //            assert(std::abs(max_axle_xyz) - std::max(std::abs(dx) , std::abs(dy)) < EPSILON);
        //            assert(std::abs(max_axle_xyz) / Vec2d(dx,dy).norm() >= 1);
        //            return std::abs(max_axle_xyz) / Vec2d(dx,dy).norm();
        //        } else {
        //            assert(std::abs(max_axle_xyz) / Vec2d(dx,dy).norm() >= 1);
        //            return std::abs(max_axle_xyz) / Vec2d(dx,dy).norm();
        //        }
        //    } else {
        //        //TODO: corexy
        //        // currently, using the line ratio
        //        return 1.;
        //    }
        //}
    };

private:
    const std::regex regex_fan_speed;
    const FullPrintConfig &m_config;
    const bool m_relative_e;

    GCodeReader m_parser{};
    const GCodeWriter& m_writer;

    //current value (at the back of the buffer), when parsing a new line
    ExtrusionRole current_role = ExtrusionRole::erCustom;
    // in unit/second
    double m_current_speed = 1000 / 60.0;
    double m_current_acceleration = 150;
    double m_current_deceleration = 150;
    double m_current_retract_acceleration = 150;
    bool m_is_custom_gcode = false;
    uint16_t m_current_extruder = 0;
    // false if speed is for each axle, true if dx & dy are combined for speed & accel command.
    // TODO: struct printer type (cartesian, corexy, delta)
    bool m_speedxy = false;

    double max_x_speed = 0;
    double max_y_speed = 0;
    double max_z_speed = 0;
    double max_e_speed = 0;
    double max_x_acceleration = 0;
    double max_y_acceleration = 0;
    double max_z_acceleration = 0;
    double max_e_acceleration = 0;

    uint8_t xyz_decimals;
    uint8_t e_decimals;

    double pressure_factor_override = std::numeric_limits<double>::quiet_NaN();

    // variable for when you add a line (front of the buffer)
    //double m_front_buffer_speed = 0;
    //double m_back_buffer_speed = 0;
    double m_front_buffer_e = 0;
    double m_front_buffer_de = 0;
    bool m_back_buffer_e_reset = false;
    double m_back_buffer_e = 0; // only for reset

    //buffer
    std::deque<BufferData> m_buffer;
    size_t m_buffer_useful_size = 0;
    const size_t m_buffer_useful_size_span = 100; // 100 commands
    double m_buffer_distance_size = 0;
    const double m_buffer_distance_span = 100.; // 100 mm

    // The output of process_layer()
    std::string m_process_output;

public:
    PressureAdvance(const GCodeWriter &writer, const FullPrintConfig &print_config, uint16_t current_extruder_id);

    // Adds the gcode contained in the given string to the analysis and returns it after removing the workcodes
    // should set he current extruder id, unless it's the same as the previous call, and there is no extruder change.
    const std::string& process_gcode(const std::string& gcode, bool flush, uint16_t current_extruder_id = uint16_t(-1));

private:
    void put_in_buffer(const BufferData& data) {
        if (data.is_useful_move())
            m_buffer_useful_size++;
        m_buffer_distance_size += data.max_axle_xyz;
    }
    void remove_from_buffer(const BufferData& data) {
        if (data.is_useful_move())
            m_buffer_useful_size--;
        m_buffer_distance_size -= data.max_axle_xyz;
    }
    size_t get_last_move_idx(size_t end) {
        for (size_t i = end - 1; i < m_buffer.size(); --i) {
            const BufferData &data = m_buffer[i];
            if (data.is_useful_move()) {
                assert(data.max_axle_xyz > 0 || data.de != 0);
                return i;
            }
        }
        return size_t(-1);
    }
    // Processes the given gcode line
    void _process_gcode_line(GCodeReader& reader, const GCodeReader::GCodeLine& line);
    void _process_ACTIVATE_EXTRUDER(const std::string_view command);
    void _process_T(const std::string_view command);
    void _process_M204(const GCodeReader::GCodeLine& line);
    //void _put_in_middle_G1(std::list<BufferData>::iterator item_to_split, float nb_sec, BufferData&& line_to_write, float max_time);
    //void _print_in_middle_G1(BufferData& line_to_split, float nb_sec, const std::string& line_to_write);
    void write_buffer_data();
    double compute_pressure_change_mm_filament(const BufferData& command);
    void update_last_move(size_t last_move_idx, double junction_axle_speed);

    double get_pressure_factor() {
        double val = this->m_config.extruder_pressure_factor.get_float(m_current_extruder);
        if (!std::isnan(pressure_factor_override)) {
            val = pressure_factor_override;
        }
        return val;
    }
    void flush() {
        if (!m_buffer.empty()) {
            size_t last_idx = get_last_move_idx(m_buffer.size());
            if (last_idx < m_buffer.size() && m_buffer[last_idx].speed_end > 0)
                update_last_move(last_idx, 0);
        }
        while (!m_buffer.empty()) { write_buffer_data(); }
    }
};

} // namespace Slic3r


#endif /* slic3r_GCode_PressureAdvance_hpp_ */
