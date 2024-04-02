#include "CalibrationFlowSpeedDialog.hpp"
#include "I18N.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/AppConfig.hpp"
#include "Jobs/ArrangeJob.hpp"
#include "GLCanvas3D.hpp"
#include "GUI.hpp"
#include "GUI_ObjectList.hpp"
#include "Plater.hpp"
#include "Tab.hpp"
#include <wx/scrolwin.h>
#include <wx/display.h>
#include <wx/file.h>
#include "wxExtensions.hpp"

#include <string>

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

void CalibrationFlowSpeedDialog::create_buttons(wxStdDialogButtonSizer* buttons){
    wxString choices_gram[] = { "0.2","0.5","1","2","5","10" };
    cmb_gram = new wxComboBox(this, wxID_ANY, wxString{ "1" }, wxDefaultPosition, wxDefaultSize, 5, choices_gram);
    cmb_gram->SetToolTip(_L("Choose the size of the patch to print (in gramme). A bigger weight allow to have more precision but it takes longer to print"));
    cmb_gram->SetSelection(3);
    
    wxString choices_nb[] = { "1","2","3","4","5","6","7","8" };
    cmb_nb_steps = new wxComboBox(this, wxID_ANY, wxString{ "4" }, wxDefaultPosition, wxDefaultSize, 8, choices_nb);
    cmb_nb_steps->SetToolTip(_L("Select the number of patches."));
    cmb_nb_steps->SetSelection(4);

    // check max speed (constrained by filament max volumetric flow)

    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();
    float max_vol_flow = filament_config->option("filament_max_volumetric_speed")->get_float(0);
    float max_speed = filament_config->option("filament_max_speed")->get_float(0);
    float curr_speed = print_config->get_computed_value("solid_infill_speed", 0);

    // compute max speed
    if (max_vol_flow == 0 && max_speed == 0)
        max_speed = curr_speed * 10;
    if (max_vol_flow > 0) {
        float layer_height = print_config->option("layer_height")->get_float();
        layer_height = std::max(layer_height, float(print_config->get_computed_value("first_layer_height", 0)));
        float nz = printer_config->option("nozzle_diameter")->get_float(0);
        float filament_max_overlap = filament_config->option<ConfigOptionPercents>("filament_max_overlap")->get_abs_value(0, 1.);
        Flow flow = Flow::new_from_config(FlowRole::frSolidInfill, *print_config, nz, layer_height, filament_max_overlap, false);
        float current_flow = flow.mm3_per_mm();
        if (max_speed > 0) {
            max_speed = std::min(max_speed, curr_speed * max_vol_flow / current_flow);
        } else {
            max_speed = curr_speed * max_vol_flow / current_flow;
        }
        max_speed = std::min(max_speed, curr_speed * 10);
        if (printer_config->option<ConfigOptionEnum<MachineLimitsUsage>>("machine_limits_usage")->value != MachineLimitsUsage::Ignore) {
            float max_feedarete_x = printer_config->option("machine_max_feedrate_x")->get_float(0);
            max_speed             = std::min(max_speed, max_feedarete_x);
        }
    }
    
    auto size = wxSize(6 * em_unit(), wxDefaultCoord);
    float min_speed = std::max(filament_config->option("min_print_speed")->get_float(0), 0);
    if (min_speed <= 0)
        min_speed = curr_speed / 10;
    txt_min_speed = new wxTextCtrl(this, wxID_ANY, std::to_string(min_speed), wxDefaultPosition, size);
    txt_min_speed->SetToolTip(_L("Speed of the first patch."));

    txt_max_speed = new wxTextCtrl(this, wxID_ANY, std::to_string(max_speed), wxDefaultPosition, size);
    txt_max_speed->SetToolTip(_L("Speed of the first patch."));

    buttons->Add(new wxStaticText(this, wxID_ANY, _L("Patch weight:")));
    buttons->Add(cmb_gram);
    buttons->Add(new wxStaticText(this, wxID_ANY, ("g")));
    buttons->AddSpacer(5);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("min speed:")));
    buttons->Add(txt_min_speed);
    buttons->Add(new wxStaticText(this, wxID_ANY, ("mm/s")));
    buttons->AddSpacer(5);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("max speed:")));
    buttons->Add(txt_max_speed);
    buttons->Add(new wxStaticText(this, wxID_ANY, ("mm/s")));
    buttons->AddSpacer(5);
    buttons->Add(new wxStaticText(this, wxID_ANY, _L("steps:")));
    buttons->Add(cmb_nb_steps);
    buttons->AddSpacer(20);

    wxButton* bt = new wxButton(this, wxID_FILE1, _L("Generate"));
    bt->Bind(wxEVT_BUTTON, &CalibrationFlowSpeedDialog::create_geometry, this);
    buttons->Add(bt);
}


std::pair<float, float> CalibrationFlowSpeedDialog::get_cube_size() {
    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();

    // compute patch size

    float density = filament_config->option("filament_density")->get_float(0);
    if (density<=0)
        density = 1.25;
    // g/cm3 to g/mm3
    density = density / 1000.f;

    std::string str = cmb_gram->GetValue().ToStdString();
    float weight = std::stof(str);

    // get max height
    float max_height = print_config->option("extruder_clearance_height")->get_float();
    // multiple of layer height
    float layer_height = print_config->option("layer_height")->get_float();
    layer_height = std::max(layer_height, float(print_config->get_computed_value("first_layer_height", 0)));
    max_height = int(max_height / layer_height) * layer_height;

    for(size_t max_iter = 0; max_iter < 100; max_iter++) {
        // get cube size
        // x*y*z*density = weight
        // x*y = weight / (z*density)
        float size_x = std::sqrt(weight / (density * max_height));

        // enlarge if overlap < 1
        float nz           = printer_config->option("nozzle_diameter")->get_float(0);
        float overlap      = print_config->option("solid_infill_overlap")->get_float();
        float filament_max_overlap = filament_config->option<ConfigOptionPercents>("filament_max_overlap")->get_abs_value(0, 1.);
        overlap = std::min(overlap, filament_max_overlap);
        if (overlap < 1) {
            //Flow flow_full = Flow::new_from_config(FlowRole::frSolidInfill, *print_config, nz, layer_height, 1, false);
            Flow flow_full = Flow::Flow::new_from_config_width(FlowRole::frSolidInfill, 
                *print_config->option<ConfigOptionFloatOrPercent>("solid_infill_extrusion_width"),
                *print_config->option<ConfigOptionFloatOrPercent>("solid_infill_extrusion_spacing"),
                nz, layer_height, 1 /*overlap ratio*/);
            Flow flow      = Flow::new_from_config(FlowRole::frSolidInfill, *print_config, nz, layer_height, filament_max_overlap, false/*bridge*/);

            if (flow.width() - flow_full.width() < -EPSILON) {
                //same spacing, -> flow has lower width  and lower flow
                assert(flow.mm3_per_mm() < flow_full.mm3_per_mm());
                size_x *= flow_full.width() / flow.width();
            } else {
                //same width, -> flow has higher spacing, same flow
                assert(std::abs(flow.mm3_per_mm() - flow_full.mm3_per_mm()) < EPSILON);
                assert(flow.spacing() > flow_full.spacing());
                size_x *= flow.spacing() / flow_full.spacing();
            }
        }

        // add external perimeter width/spacing diff
        float spacing_diff = layer_height * float(1. - 0.25 * PI);
        size_x += spacing_diff;

        if (size_x > max_height / 2)
            return {size_x, max_height};

        max_height = max_height / 2;
        max_height = int(max_height / layer_height) * layer_height;
    }
    assert(false);
    return { 0, 0 };
}

void CalibrationFlowSpeedDialog::create_geometry(wxCommandEvent& event_args) {
    Plater* plat = this->main_frame->plater();
    Model& model = plat->model();
    if (!plat->new_project(L("Flow calibration")))
        return;

    //GLCanvas3D::set_warning_freeze(true);
    bool autocenter = gui_app->app_config->get("autocenter") == "1";
    if (autocenter) {
        //disable auto-center for this calibration.
        gui_app->app_config->set("autocenter", "0");
    }
    
    std::string str_parse = cmb_nb_steps->GetValue().ToStdString();
    int nb_steps = std::stoi(str_parse);
    
    str_parse = txt_min_speed->GetValue().ToStdString();
    float min_speed = std::stof(str_parse);
    
    str_parse = txt_max_speed->GetValue().ToStdString();
    float max_speed = std::stof(str_parse);

    auto [cube_xy,cube_z] = get_cube_size();
    model.clear_objects();
    std::vector<ModelObject*> objs;
    for (size_t i = 0; i < nb_steps; i++) {
        objs.push_back(model.add_object("cube", "", Slic3r::make_cube(cube_xy, cube_xy, cube_z)));
        objs.back()->add_instance();
    }
    
    const DynamicPrintConfig* print_config = this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->get_config();
    const DynamicPrintConfig* printer_config = this->gui_app->get_tab(Preset::TYPE_PRINTER)->get_config();
    const DynamicPrintConfig* filament_config = this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->get_config();

    /// --- main config, please modify object config when possible ---
    DynamicPrintConfig new_print_config = *print_config; //make a copy
    new_print_config.set_key_value("complete_objects", new ConfigOptionBool(true));
    //if skirt, use only one
    //if (print_config->option<ConfigOptionInt>("skirts")->get_int() > 0 && print_config->option<ConfigOptionInt>("skirt_height")->get_int() > 0) {
    //    new_print_config.set_key_value("complete_objects_one_skirt", new ConfigOptionBool(true));
    //}

    // same for printer config
    DynamicPrintConfig new_printer_config = *printer_config; //make a copy
    new_printer_config.option<ConfigOptionFloatsOrPercents>("seam_gap")->set_at(FloatOrPercent{0, false}, 0);

    // same for filament config
    DynamicPrintConfig new_filament_config = *filament_config; //make a copy
    new_filament_config.option<ConfigOptionFloats>("slowdown_below_layer_time")->set_at(0, 0);

    /// --- custom config ---
    float overlap = print_config->option("solid_infill_overlap")->get_float();
    float filament_max_overlap = filament_config->option<ConfigOptionPercents>("filament_max_overlap")->get_abs_value(0, 1.);
    overlap = std::min(overlap, filament_max_overlap);
    float layer_height = print_config->option("layer_height")->get_float();
    layer_height = std::max(layer_height, float(print_config->get_computed_value("first_layer_height", 0)));
    float nz = printer_config->option("nozzle_diameter")->get_float(0);
    Flow flow = Flow::new_from_config(FlowRole::frSolidInfill, *print_config, nz, layer_height, overlap, false);
    for (size_t i = 0; i < nb_steps; i++) {
        objs[i]->config.set_key_value("perimeter_overlap", new ConfigOptionPercent(overlap));
        objs[i]->config.set_key_value("external_perimeter_overlap", new ConfigOptionPercent(overlap));

        objs[i]->config.set_key_value("solid_infill_extrusion_width", new ConfigOptionFloatOrPercent(flow.width(), false));
        objs[i]->config.set_key_value("top_infill_extrusion_width", new ConfigOptionFloatOrPercent(flow.width(), false));
        objs[i]->config.set_key_value("perimeter_extrusion_width", new ConfigOptionFloatOrPercent(flow.width(), false));
        objs[i]->config.set_key_value("external_perimeter_extrusion_width", new ConfigOptionFloatOrPercent(flow.width(), false));
        // keep first_layer_extrusion_width, it doesn't change the weight.
        //objs[i]->config.set_key_value("first_layer_extrusion_width", new ConfigOptionFloatOrPercent(flow.width(), false));

        objs[i]->config.set_key_value("first_layer_size_compensation", new ConfigOptionFloat(0));
        
        // no brim (but a skirt for primming)
        objs[i]->config.set_key_value("brim_ears", new ConfigOptionBool(false));
        objs[i]->config.set_key_value("brim_width", new ConfigOptionFloat(0));

        objs[i]->config.set_key_value("enforce_full_fill_volume", new ConfigOptionBool(true));
        objs[i]->config.set_key_value("bottom_solid_layers", new ConfigOptionInt(10000));
        objs[i]->config.set_key_value("top_solid_layers", new ConfigOptionInt(10000));
        objs[i]->config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
        objs[i]->config.set_key_value("first_layer_height", new ConfigOptionFloatOrPercent(layer_height, false));
        objs[i]->config.set_key_value("solid_fill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        objs[i]->config.set_key_value("top_fill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        objs[i]->config.set_key_value("perimeter_generator", new ConfigOptionEnum<PerimeterGeneratorType>(PerimeterGeneratorType::Classic));
        //disable ironing post-process
        objs[i]->config.set_key_value("ironing", new ConfigOptionBool(false));
        //set speed
        if (nb_steps > 1){
            float speed = float(min_speed + i * double(max_speed - min_speed) / (nb_steps - 1));
            objs[i]->config.set_key_value("perimeter_speed", new ConfigOptionFloatOrPercent(speed, false));
            objs[i]->config.set_key_value("external_perimeter_speed", new ConfigOptionFloatOrPercent(speed, false));
            objs[i]->config.set_key_value("solid_infill_speed", new ConfigOptionFloatOrPercent(speed, false));
            objs[i]->config.set_key_value("top_solid_infill_speed", new ConfigOptionFloatOrPercent(speed, false));
        }
        // keep first_layer_speed.
    }

    //update plater
    //GLCanvas3D::set_warning_freeze(false);
    this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->load_config(new_print_config);
    plat->on_config_change(new_print_config);
    this->gui_app->get_tab(Preset::TYPE_PRINTER)->load_config(new_printer_config);
    plat->on_config_change(new_printer_config);
    this->gui_app->get_tab(Preset::TYPE_FFF_FILAMENT)->load_config(new_filament_config);
    plat->on_config_change(new_filament_config);
    //plat->changed_objects(objs_idx);
    this->gui_app->get_tab(Preset::TYPE_FFF_PRINT)->update_dirty();
    //update everything, easier to code.
    ObjectList* obj = this->gui_app->obj_list();
    obj->update_after_undo_redo();

    // arrange if needed, after new settings, to take them into account
    if (true) { //has_to_arrange) {
        //update print config (done at reslice but we need it here)
        if (plat->printer_technology() == ptFFF)
            plat->fff_print().apply(plat->model(), *plat->config());
        std::shared_ptr<ProgressIndicatorStub> fake_statusbar = std::make_shared<ProgressIndicatorStub>();
        ArrangeJob arranger(std::dynamic_pointer_cast<ProgressIndicator>(fake_statusbar), plat);
        arranger.prepare_all();
        arranger.process();
        arranger.finalize();
    }

    plat->reslice();

    if (autocenter) {
        //re-enable auto-center after this calibration.
        gui_app->app_config->set("autocenter", "1");
    }
}

} // namespace GUI
} // namespace Slic3r
