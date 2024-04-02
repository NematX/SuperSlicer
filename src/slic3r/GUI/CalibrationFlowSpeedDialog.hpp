#ifndef slic3r_GUI_CalibrationFlowSpeedDialog_hpp_
#define slic3r_GUI_CalibrationFlowSpeedDialog_hpp_

#include "CalibrationAbstractDialog.hpp"
#include "libslic3r/Flow.hpp"

#include <tuple>

namespace Slic3r { 
namespace GUI {

class CalibrationFlowSpeedDialog : public CalibrationAbstractDialog
{

public:
    CalibrationFlowSpeedDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Flow speed calibration") { create(boost::filesystem::path("calibration") / "filament_flow","filament_flow.html", wxSize(900, 500));  }
    virtual ~CalibrationFlowSpeedDialog() {}
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    void create_speed(wxCommandEvent& event_args);
    void create_overlap(wxCommandEvent& event_args);
    void create_geometry(float min_speed, float max_speed, float min_overlap, float max_overlap);
    std::tuple<float,float, Flow> get_cube_size(float overlap);
    
    wxComboBox* cmb_gram;
    wxComboBox* cmb_nb_steps;
    wxTextCtrl* txt_min_speed;
    wxTextCtrl* txt_max_speed;
    wxComboBox* cmb_min_overlap;
    wxComboBox* cmb_max_overlap;
};

} // namespace GUI
} // namespace Slic3r

#endif
