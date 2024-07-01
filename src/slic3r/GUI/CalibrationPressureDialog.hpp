#ifndef slic3r_GUI_CalibrationPressureDialog_hpp_
#define slic3r_GUI_CalibrationPressureDialog_hpp_

#include "CalibrationAbstractDialog.hpp"

namespace Slic3r { 
namespace GUI {

class CalibrationPressureDialog : public CalibrationAbstractDialog
{

public:
    CalibrationPressureDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Pressure advance Calibration") { create(boost::filesystem::path("calibration") / "pressure", "pressure.html"); }
    virtual ~CalibrationPressureDialog(){ }
    
protected:
    void create_buttons(wxStdDialogButtonSizer* sizer) override;
    void create_geometry(wxCommandEvent& event_args);

    wxComboBox* cmb_height;
    wxTextCtrl* txt_min_pressure;
    wxTextCtrl* txt_max_pressure;
};

} // namespace GUI
} // namespace Slic3r

#endif
