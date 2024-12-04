#include "CalibrationExtruderOffsetDialog.hpp"
#include "I18N.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/AppConfig.hpp"
//#include "Jobs/ArrangeJob2.hpp"
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

void CalibrationExtruderOffsetDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    // get extruder count
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    //const ConfigOptionFloats* nozzle_diameter_config = printer_config->option<ConfigOptionFloats>("nozzle_diameter");
    const ConfigOptionStrings* tool_name_config = printer_config->option<ConfigOptionStrings>("tool_name");
    size_t extruder_count = tool_name_config->size();
    if (extruder_count > 1) {
        std::vector<wxString> choices_extruder;
        for (size_t extruder_idx = 1; extruder_idx < extruder_count; ++extruder_idx) {
            if (!tool_name_config->get_at(extruder_idx).empty()) {
                choices_extruder.push_back(tool_name_config->get_at(extruder_idx));
            } else {
                choices_extruder.push_back(std::to_string(extruder_idx + 1));
            }
        }
        cmb_toolname = new wxComboBox(this, wxID_ANY, choices_extruder.front(), wxDefaultPosition, wxDefaultSize, 3, &choices_extruder[0]);
        cmb_toolname->SetToolTip(_L("Select the extruder to calibrate (against the first extruder)."));
        cmb_toolname->SetSelection(0);
        wxString step[] = {"0.025", "0.1", "0.25", "1"};
        cmb_increment = new wxComboBox(this, wxID_ANY, wxString{"0.1"}, wxDefaultPosition, wxDefaultSize, 3, step);
        cmb_increment->SetToolTip(_L("step in mm for each teeth compared to it's neighbours."));
        cmb_increment->SetSelection(1);
        wxString nb_teeth[] = {"5", "10", "20"};
        cmb_teeth_count = new wxComboBox(this, wxID_ANY, wxString{"10"}, wxDefaultPosition, wxDefaultSize, 3, nb_teeth);
        cmb_teeth_count->SetToolTip(_L("Number of theeth for each direction."));
        cmb_teeth_count->SetSelection(1);
        
        buttons->Add(new wxStaticText(this, wxID_ANY, _L("Extruder:")));
        buttons->Add(cmb_toolname);
        buttons->AddSpacer(20);
        buttons->Add(new wxStaticText(this, wxID_ANY, _L("mm per step:")));
        buttons->Add(cmb_increment);
        buttons->AddSpacer(20);
        buttons->Add(new wxStaticText(this, wxID_ANY, _L("Step count:")));
        buttons->Add(cmb_teeth_count);
        buttons->AddSpacer(20);
        wxButton *bt = new wxButton(this, wxID_FILE1, _L("Create"));
        bt->Bind(wxEVT_BUTTON, &CalibrationExtruderOffsetDialog::create_geometry, this);
        buttons->Add(bt);
    }
}

void CalibrationExtruderOffsetDialog::create_geometry(wxCommandEvent& event_args) {
    Plater* plat = this->main_frame->plater();
    assert(plat);
    Model& model = plat->model();
    if (!plat->new_project(L("Extruder calibration")))
        return;

    //GLCanvas3D::set_warning_freeze(true);
    bool autocenter = gui_app->app_config->get("autocenter") == "1";
    if (autocenter) {
        //disable auto-center for this calibration.
        gui_app->app_config->set("autocenter", "0");
    }

    int32_t extruder_idx = cmb_toolname->GetSelection();
    if (extruder_idx == wxNOT_FOUND) {
        extruder_idx = 0;
    }
    // first extruder isn't in the list.
    extruder_idx++;

    
    // number of test in each dir
    long nb_teeth = 1;
    if (!cmb_teeth_count->GetValue().ToLong(&nb_teeth)) {
        nb_teeth = 7;
    }
    
    // how far the tests goes
    double teeth_increment = 0;
    if (!cmb_increment->GetValue().ToDouble(&teeth_increment)) {
        teeth_increment = 0.1;
    }
    const double max_displacement = teeth_increment * nb_teeth;


    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();

    bool has_to_arrange = true;
    const double first_layer_height = std::min(print_config->get_computed_value("first_layer_height", 0), print_config->get_computed_value("first_layer_height", extruder_idx));
    const double filament_max_overlap_0 = filament_config->get_computed_value("filament_max_overlap", 0);
    const double filament_max_overlap_X = filament_config->get_computed_value("filament_max_overlap", extruder_idx);
    const double nozzle_diameter_0 = printer_config->option("nozzle_diameter")->get_float(0);
    const double nozzle_diameter_X = printer_config->option("nozzle_diameter")->get_float(extruder_idx);
    
    const Flow ext_peri_flow_0 = Flow::new_from_config(FlowRole::frExternalPerimeter, *print_config, nozzle_diameter_0,
                                                first_layer_height, filament_max_overlap_0, true);
    const Flow ext_peri_flow_X = Flow::new_from_config(FlowRole::frExternalPerimeter, *print_config, nozzle_diameter_X,
                                                first_layer_height, filament_max_overlap_X, true);
    const Flow peri_flow_0 = Flow::new_from_config(FlowRole::frPerimeter, *print_config, nozzle_diameter_0,
                                            first_layer_height, filament_max_overlap_0, true);
    const Flow peri_flow_X = Flow::new_from_config(FlowRole::frPerimeter, *print_config, nozzle_diameter_X,
                                            first_layer_height, filament_max_overlap_X, true);


    const double external_extrusion_doublewidth = std::max(ext_peri_flow_0.width() + ext_peri_flow_0.spacing(), ext_peri_flow_X.width() + ext_peri_flow_X.spacing());
    const double extrusion_doublewidth = std::max(peri_flow_0.spacing() * 2, peri_flow_X.spacing() * 2);
    const double layer_height = print_config->option("layer_height")->get_float();
    const double teeth_length = extrusion_doublewidth * 5;
    const double separation_teeth = std::max(external_extrusion_doublewidth * 2, max_displacement);

    //note: cube is created in the bottom left corner as origin.

    //create new object
    plat->take_snapshot(_L("Create calibration"));
    TriangleMesh cube_mesh = TriangleMesh(its_make_cube(separation_teeth * (nb_teeth * 2 + 1),
                                                external_extrusion_doublewidth + extrusion_doublewidth, first_layer_height));
    wxGetApp().obj_list()->load_mesh_object(cube_mesh, _u8L("Extruder Calibration"));
    assert(model.objects.size() == 1);
    ModelObject &model_object = *model.objects[0];
    assert(model_object.volumes.size() == 1);
    // ote: the volumes of the object are moved at its mass center.
    model_object.volumes.front()->translate(0, -(external_extrusion_doublewidth + extrusion_doublewidth) / 2 - teeth_length, first_layer_height / 2);
    model_object.volumes.front()->name = "extruder 1 mat";
    model_object.volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(0));
    model_object.volumes.front()->config.set_key_value("first_layer_extruder", new ConfigOptionInt(0));

    ModelVolume *new_volume;

    //create teeth of the first extruder
    for (size_t i_teeth = 0; i_teeth < nb_teeth + nb_teeth + 1; ++i_teeth) {
        cube_mesh = TriangleMesh(its_make_cube(external_extrusion_doublewidth, teeth_length, first_layer_height));
        new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
        new_volume->translate(-separation_teeth * nb_teeth + i_teeth * separation_teeth - external_extrusion_doublewidth / 2, -teeth_length, 0);
        new_volume->name = "extruder 1 teeth";
        new_volume->config.set_key_value("extruder", new ConfigOptionInt(0));
        new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(0));
    }

    //create teeths of the second extruder (with separation)
    cube_mesh = TriangleMesh(its_make_cube(separation_teeth * (nb_teeth * 2 + 1) + max_displacement,
                                           external_extrusion_doublewidth + extrusion_doublewidth,
                                           first_layer_height));
    new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    new_volume->translate(-separation_teeth * (nb_teeth * 2 + 1) / 2 - max_displacement / 2, teeth_length, 0);
    new_volume->name = "extruder 2 mat";
    new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
    new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));

    
    for (size_t i_teeth = 0; i_teeth < nb_teeth + nb_teeth + 1; ++i_teeth) {
        cube_mesh = TriangleMesh(its_make_cube(external_extrusion_doublewidth, teeth_length, first_layer_height));
        new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
        double displacement = -max_displacement + i_teeth * teeth_increment;
        new_volume->translate(-separation_teeth * nb_teeth + i_teeth * separation_teeth - external_extrusion_doublewidth / 2 + displacement, 0, 0);
        new_volume->name = "extruder 2 teeth";
        new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
        new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));
    }

    // Draw middle teeth thicker
    cube_mesh = TriangleMesh(its_make_cube(external_extrusion_doublewidth, teeth_length, layer_height));
    new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    new_volume->translate(-external_extrusion_doublewidth / 2, -teeth_length - external_extrusion_doublewidth - extrusion_doublewidth, first_layer_height);
    new_volume->name = "extruder 1 midde teeth";
    new_volume->config.set_key_value("extruder", new ConfigOptionInt(0));
    new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(0));
    cube_mesh = TriangleMesh(its_make_cube(external_extrusion_doublewidth, teeth_length, layer_height));
    new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    new_volume->translate(-external_extrusion_doublewidth / 2, external_extrusion_doublewidth + extrusion_doublewidth, first_layer_height);
    new_volume->name = "extruder 2 middle teeth";
    new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
    new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));

    // Draw a - and a + with three cubes
    cube_mesh = TriangleMesh(its_make_cube(extrusion_doublewidth * 4 , extrusion_doublewidth, layer_height));
    new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    new_volume->translate(-separation_teeth * (nb_teeth),
                          (external_extrusion_doublewidth + extrusion_doublewidth) / 2 + teeth_length - extrusion_doublewidth / 2,
                          first_layer_height);
    new_volume->name = "minus";
    new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
    new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));

    // Draw test marks
    //cube_mesh = TriangleMesh(its_make_cube(2,4,1));
    //new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    //new_volume->translate(0,0,0);
    //new_volume->name = "test";
    //new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
    //new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));
    //cube_mesh = TriangleMesh(its_make_cylinder(0.1, 2));
    //new_volume = model_object.add_volume(std::move(cube_mesh), ModelVolumeType::MODEL_PART);
    //new_volume->translate(0,0,0);
    //new_volume->name = "test_cyl";
    //new_volume->config.set_key_value("extruder", new ConfigOptionInt(extruder_idx + 1));
    //new_volume->config.set_key_value("first_layer_extruder", new ConfigOptionInt(extruder_idx + 1));

    ///// --- custom config ---
    model.objects.back()->config.set_key_value("first_layer_size_compensation", new ConfigOptionFloat(0));
    model.objects.back()->config.set_key_value("support_material", new ConfigOptionBool(false));
    model.objects.back()->config.set_key_value("support_material_auto", new ConfigOptionBool(false));
    // 
    //for (size_t i = 0; i < nb_items; i++) {
    //    model.objects[objs_idx[i]]->config.set_key_value("brim_width", new ConfigOptionFloat(brim_width));
    //    model.objects[objs_idx[i]]->config.set_key_value("brim_ears", new ConfigOptionBool(false));
    //    model.objects[objs_idx[i]]->config.set_key_value("perimeters", new ConfigOptionInt(2));
    //    model.objects[objs_idx[i]]->config.set_key_value("bottom_solid_layers", new ConfigOptionInt(2));
    //    model.objects[objs_idx[i]]->config.set_key_value("gap_fill_enabled", new ConfigOptionBool(false));
    //    model.objects[objs_idx[i]]->config.set_key_value(setting_to_test, new ConfigOptionPercent(start + (add ? 1 : -1) * i * step));
    //    model.objects[objs_idx[i]]->config.set_key_value("layer_height", new ConfigOptionFloat(nozzle_diameter / 2));
    //    model.objects[objs_idx[i]]->config.set_key_value("no_perimeter_unsupported_algo", new ConfigOptionEnum<NoPerimeterUnsupportedAlgo>(npuaBridges));
    //    //model.objects[objs_idx[i]]->config.set_key_value("top_fill_pattern", new ConfigOptionEnum<InfillPattern>(ipSmooth)); /not needed
    //    model.objects[objs_idx[i]]->config.set_key_value("ironing", new ConfigOptionBool(false)); // not needed, and it slow down things.
    //}
    ///// if first ayer height is excactly at the wrong value, the text isn't drawed. Fix that by switching the first layer height just a little bit.
    //double first_layer_height = full_print_config.get_computed_value("first_layer_height", 0);
    //double layer_height = nozzle_diameter * 0.5;
    //if (layer_height > 0.01 && (int(first_layer_height * 100) % int(layer_height * 100)) == int(layer_height * 50)) {
    //    double z_step = printer_config->option<ConfigOptionFloat>("z_step")->value;
    //    if (z_step == 0)
    //        z_step = 0.1;
    //    double max_height = full_print_config.get_computed_value("max_layer_height",0);
    //    if (max_height < EPSILON || !full_print_config.option("max_layer_height")->is_enabled())
    //        max_height = 0.75 * nozzle_diameter;
    //    if (max_height > first_layer_height + z_step)
    //        for (size_t i = 0; i < nb_items; i++)
    //            model.objects[objs_idx[i]]->config.set_key_value("first_layer_height", new ConfigOptionFloatOrPercent(first_layer_height + z_step, false));
    //    else
    //        for (size_t i = 0; i < nb_items; i++)
    //            model.objects[objs_idx[i]]->config.set_key_value("first_layer_height", new ConfigOptionFloatOrPercent(first_layer_height - z_step, false));
    //}

    //update plater
    plat->changed_objects(std::vector<size_t>{0});
    //this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->update_dirty();
    //update everything, easier to code.
    ObjectList* obj = this->gui_app->obj_list();
    obj->update_after_undo_redo();

    // arrange if needed, after new settings, to take them into account
    if (has_to_arrange) {
        //update print config (done at reslice but we need it here)
        if (plat->printer_technology() == ptFFF)
            plat->fff_print().apply(plat->model(), *plat->config());
        plat->arrange();
    }

    plat->reslice();

    if (autocenter) {
        //re-enable auto-center after this calibration.
        gui_app->app_config->set("autocenter", "1");
    }
}

} // namespace GUI
} // namespace Slic3r
