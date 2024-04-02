#ifndef slic3r_GUI_CalibrationFlowSpeedDialog_hpp_
#define slic3r_GUI_CalibrationFlowSpeedDialog_hpp_

#include "CalibrationAbstractDialog.hpp"

namespace Slic3r { 
namespace GUI {

class CalibrationFlowSpeedDialog : public CalibrationAbstractDialog
{

public:
    CalibrationFlowSpeedDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Flow speed calibration") { create(boost::filesystem::path("calibration") / "filament_flow","filament_flow.html", wxSize(900, 500));  }
    virtual ~CalibrationFlowSpeedDialog() {}
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    void create_geometry(wxCommandEvent& event_args);
    std::pair<float,float> get_cube_size();
    
    wxComboBox* cmb_gram;
    wxComboBox* cmb_nb_steps;
    wxTextCtrl* txt_min_speed;
    wxTextCtrl* txt_max_speed;
};

} // namespace GUI
} // namespace Slic3r

#endif
