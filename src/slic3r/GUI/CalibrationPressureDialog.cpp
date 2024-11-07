#include "CalibrationPressureDialog.hpp"
#include "I18N.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "GLCanvas3D.hpp"
#include "GUI.hpp"
#include "GUI_ObjectList.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include <wx/scrolwin.h>
#include <wx/display.h>
#include <wx/file.h>
#include "wxExtensions.hpp"

#if ENABLE_SCROLLABLE
static wxSize get_screen_size(wxWindow* window)
{
    const auto idx = wxDisplay::GetFromWindow(window);
    wxDisplay display(idx != wxNOT_FOUND ? idx : 0u);
    return display.GetClientArea().GetSize();
}
#endif // ENABLE_SCROLLABLE

namespace Slic3r {
namespace GUI {

void CalibrationPressureDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    
    wxString choices_height[] = { "5","10","20","30","40","50","60","70","80","90","100"};
    cmb_height = new wxComboBox(this, wxID_ANY, wxString{ "20" }, wxDefaultPosition, wxDefaultSize, 8, choices_height);
    cmb_height->SetToolTip(_L("Minimum overlap (0%: llnes don't touch (bad but easy to print)"
        " ; 100%: no empty spaces (almost impossible, the filament isn't liquid enough)."));
    cmb_height->SetSelection(3);
    
    auto txt_size = wxSize(6 * em_unit(), wxDefaultCoord);

    txt_min_pressure = new wxTextCtrl(this, wxID_ANY, std::to_string(0), wxDefaultPosition, txt_size);
    txt_min_pressure->SetToolTip(_L("Minimum pressure factor."));

    txt_max_pressure = new wxTextCtrl(this, wxID_ANY, std::to_string(1), wxDefaultPosition, txt_size);
    txt_max_pressure->SetToolTip(_L("Maximum pressure factor."));

    wxButton* bt = new wxButton(this, wxID_FILE1, _(L("Generate")));
    bt->Bind(wxEVT_BUTTON, &CalibrationPressureDialog::create_geometry, this);
    bt->SetToolTip(_L("Generate the geometry and the settings."));
    
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("Pressure ")));
    buttons->AddSpacer(10);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("min:")));
    buttons->Add(txt_min_pressure);
    buttons->AddSpacer(20);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("max:")));
    buttons->Add(txt_max_pressure);
    buttons->AddSpacer(40);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("height:")));
    buttons->Add(cmb_height);
    buttons->AddSpacer(40);

    buttons->Add(bt);
}

void CalibrationPressureDialog::create_geometry(wxCommandEvent& event_args) {
    Plater* plat = this->main_frame->plater();
    Model& model = plat->model();
    if (!plat->new_project(L("Pressure Calibration")))
        return;

    //GLCanvas3D::set_warning_freeze(true);
    std::vector<size_t> objs_idx = plat->load_files(std::vector<std::string>{
            (boost::filesystem::path(Slic3r::resources_dir()) / "calibration"/"pressure"/ "pressure_shape.stl").string()}, true, false, false, false);

    assert(objs_idx.size() == 1);
    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    
    
    std::string str = cmb_height->GetValue().ToStdString();
    const float height = std::stof(str);

    /// --- scale ---
    //model is created for a 0.4 nozzle, scale xy with nozzle size.
    const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    assert(nozzle_diameter_config->size() > 0);
    float nozzle_diameter = nozzle_diameter_config->get_at(0);
    double xyzScale = nozzle_diameter / 0.4;
    //do scaling
    model.objects[objs_idx[0]]->scale(xyzScale, xyzScale, height / 10);


    /// --- translate ---
    const ConfigOptionPoints* bed_shape = printer_config->option<ConfigOptionPoints>("bed_shape");
    Vec2d bed_size = BoundingBoxf(bed_shape->get_values()).size();
    Vec2d bed_min = BoundingBoxf(bed_shape->get_values()).min;
    model.objects[objs_idx[0]]->translate({ bed_min.x() + bed_size.x() / 2, bed_min.y() + bed_size.y() / 2, 0 });
    

    /// --- custom ovbject's print config ---
    model.objects[objs_idx[0]]->config.set_key_value("perimeters", new ConfigOptionInt(1));
    model.objects[objs_idx[0]]->config.set_key_value("top_solid_layers", new ConfigOptionInt(0));
    model.objects[objs_idx[0]]->config.set_key_value("bottom_solid_layers", new ConfigOptionInt(0));
    model.objects[objs_idx[0]]->config.set_key_value("brim_width", new ConfigOptionFloat(0));
    model.objects[objs_idx[0]]->config.set_key_value("fill_density", new ConfigOptionPercent(0));
    
    // custom printer config
    DynamicPrintConfig new_printer_config = *printer_config; //make a copy
    // pressure setting
    std::string str_parse = txt_min_pressure->GetValue().ToStdString();
    float min_pressure = std::stof(str_parse);
    str_parse = txt_max_pressure->GetValue().ToStdString();
    float max_pressure = std::stof(str_parse);
    if (min_pressure > max_pressure) {
        min_pressure += max_pressure;
        max_pressure = min_pressure - max_pressure;
        min_pressure = min_pressure - max_pressure;
    }
    float diff_pressure = max_pressure - min_pressure;
    float layer_height = print_config->get_float("layer_height");
    float incr_pressure_perlayer = diff_pressure * layer_height / height;
    std::string macro_pa_per_layer = ";AFTER_LAYER_CHANGE\n";
    macro_pa_per_layer += ";[layer_z]\n";
    macro_pa_per_layer += "; PRESSURE_FACTOR={";
    macro_pa_per_layer += std::to_string(min_pressure);
    macro_pa_per_layer += " + layer_num * ";
    macro_pa_per_layer += std::to_string(incr_pressure_perlayer);
    macro_pa_per_layer += "}\n";
    new_printer_config.set_key_value("layer_gcode", new ConfigOptionString(macro_pa_per_layer));

    //add seam position
    {
        ModelObject & model_object = *model.objects[objs_idx[0]];
        BoundingBoxf3 instance_bb  = model_object.instance_bounding_box(0);
        TriangleMesh  mesh         = TriangleMesh(its_make_sphere(nozzle_diameter * 10, PI / 18));
        // Mesh will be centered when loading.
        ModelVolume *new_volume = model_object.add_volume(std::move(mesh), ModelVolumeType::SEAM_POSITION_CENTER);
        new_volume->set_offset(Vec3d(0,-51*xyzScale,0));
        new_volume->name = into_u8(_L("Seam") + "-" + _("Sphere"));
        new_volume->source.is_from_builtin_objects = true;
    }
    //update plater
    this->gui_app->get_tab(Preset::TYPE_PRINTER)->load_config(new_printer_config);
    plat->on_config_change(new_printer_config);

    //update everything, easier to code.
    ObjectList* obj = this->gui_app->obj_list();
    obj->update_after_undo_redo();


    plat->reslice();

}

} // namespace GUI
} // namespace Slic3r
