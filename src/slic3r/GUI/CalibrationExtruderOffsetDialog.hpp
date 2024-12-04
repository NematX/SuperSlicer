#ifndef slic3r_GUI_CalibrationExtruderOffsetDialog_hpp_
#define slic3r_GUI_CalibrationExtruderOffsetDialog_hpp_

#include "CalibrationAbstractDialog.hpp"

namespace Slic3r { 
namespace GUI {

class CalibrationExtruderOffsetDialog : public CalibrationAbstractDialog
{

public:
    CalibrationExtruderOffsetDialog(GUI_App* app, MainFrame* mainframe) : CalibrationAbstractDialog(app, mainframe, "Extruder offset calibration") { create(boost::filesystem::path("calibration") / "extruder_offset", "extruder_offset.html", wxSize(850, 400)); }
    virtual ~CalibrationExtruderOffsetDialog() { }
    
protected:
    void create_buttons(wxStdDialogButtonSizer* buttons) override;
    void create_geometry(wxCommandEvent& event_args);
    
    wxComboBox* cmb_teeth_count;
    wxComboBox* cmb_increment;
    wxComboBox* cmb_toolname;
};

} // namespace GUI
} // namespace Slic3r

#endif
