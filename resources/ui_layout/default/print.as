//overhangs : quick set/unset like the one in prusalicer

int s_overhangs_get()
{
	if (get_float("overhangs_width_speed") == 0) return 0;
	float width = get_float("overhangs_width");
	bool percent = is_percent("overhangs_width");
	if((percent && width > 50.f) || ((!percent) && width > 0.2f)) return 1;
	return -1;
}

void s_overhangs_set(bool set)
{
	if (set) {
		set_percent("overhangs_width_speed", 55.f);
		float width = get_float("overhangs_width");
		bool percent = is_percent("overhangs_width");
		if((percent && width < 50.f) || ((!percent) && width < 0.2f))
			set_percent("overhangs_width", 75.f);
	} else {
		set_float("overhangs_width_speed", 0.);
	}
}

// "not thick bridge" like in prusaslicer

float compute_overlap()
{
	float height = get_float("layer_height");
	float width = get_computed_float("solid_infill_extrusion_width");
	if(height <= 0) return 1;
	if(width <= 0) return 1;
	float solid_spacing = (width - height * 0.215);
	float solid_flow = height * solid_spacing;
	float bridge_spacing = sqrt(solid_flow*1.2739);
	float round_val = floor((bridge_spacing / solid_spacing) * 1000. + 0.5) / 1000.;
	return round_val;
}

int s_not_thick_bridge_get()
{
	bool is_set = false;
	get_custom_bool(0,"not_thick_bridge", is_set);
	if(is_set){
		//set other vars
		ask_for_refresh();
		return 1;
	}
	return 0;
}

void s_not_thick_bridge_reset()
{
	set_custom_string(0,"not_thick_bridge", "");
	back_initial_value("bridge_type");
	back_initial_value("bridge_overlap");
	back_initial_value("bridge_overlap_min");
}

void s_not_thick_bridge_set(bool set)
{
	bool var_set = false;
	get_custom_bool(0,"not_thick_bridge", var_set);
	if (var_set != set) {
		set_custom_bool(0,"not_thick_bridge", set);
	}
	if (set) {
		if (get_int("bridge_type") != 2)
			set_int("bridge_type", 2);
		float overlap = compute_overlap();
		set_float("bridge_overlap", overlap);
		set_float("bridge_overlap_min", overlap);
	} else if (var_set != set) {
		back_initial_value("bridge_type");
		back_initial_value("bridge_overlap");
		back_initial_value("bridge_overlap_min");
	}
}

// seam position
//    spRandom [spNearest] spAligned spRear [spCustom] spCost
// ("Cost-based") ("Random") ("Aligned") ("Rear")
// -> Corners Nearest Random Aligned Rear Custom

//    spRandom spAllRandom [spNearest] spAligned spExtrAligned spRear [spCustom] spCost
// ("Cost-based") ("Scattered") ("Random") ("Aligned") ("Contiguous") ("Rear")
// -> Corners Nearest Scattered Random  Aligned Contiguous Rear Custom
float user_angle = 0;
float user_travel = 0;

int s_seam_position_get(string &out get_val)
{
	int pos = get_int("seam_position");
	string seam_pos;
	get_string("seam_position", seam_pos);
	if(pos < 7){
		if (pos == 0) return 2;// Scattered
		if (pos == 1) return 3;// Random
		return pos + 1;
	} else {
		float angle = get_float("seam_angle_cost");
		float travel = get_float("seam_travel_cost");
		if(angle >= 1. && travel <= 0.8) return 0; //corner
		if(angle <= 1. && travel >= 1.0) return 1; //nearest
		user_angle = angle;
		user_travel = travel;
	}
	return 7;
}

void s_seam_position_set(string &in set_val, int idx)
{
	if (idx == 2 ) {
		set_int("seam_position", 0); // Scattered
	} else if (idx == 3) {
		set_int("seam_position", 1); // Random
	} else if (idx == 4) {
		set_int("seam_position", 3); // Aligned
	} else if (idx == 5) {
		set_int("seam_position", 4); // Contiguous
	} else if (idx == 6) {
		set_int("seam_position", 5); // Rear
	} else if (idx <= 1) {
		set_int("seam_position", 7);
		if (idx == 0) { //corner
			set_percent("seam_angle_cost", 120);
			set_percent("seam_travel_cost", 40);
		} else { // == 1 // nearest
			set_percent("seam_angle_cost", 80);
			set_percent("seam_travel_cost", 100);
		}
	} else {
		set_int("seam_position", 7); // custom
		if(user_angle > 0 || user_travel > 0){
			set_percent("seam_angle_cost", user_angle);
			set_percent("seam_travel_cost", user_travel);
		} else {
			back_initial_value("seam_angle_cost");
			back_initial_value("seam_travel_cost");
		}
	}
}

bool s_seam_position_is_enabled()
{
    return get_int("perimeters") > 0;
}

// s_wall_thickness
// set the perimeter_spacing & external_perimeter_spacing
// as m * 2 perimeter_spacing + n * 2 * external_perimeter_spacing = o * s_wall_thickness

float s_wall_thickness_get()
{
	int nb_peri = 2;
	if (!get_custom_int(0,"wall_thickness_lines", nb_peri)) nb_peri = 2;
	float ps = get_computed_float("perimeter_extrusion_spacing");
	float eps = get_computed_float("external_perimeter_extrusion_spacing");
	//print("s_wall_thickness_get "+ps+" "+eps+" *"+nb_peri+"\n");
	if (nb_peri == 0) return 0; // fake 'disable'
	if (nb_peri < 2) nb_peri = 2; // too thin value
	if( eps > 100000) return 0;
	if( ps > 100000) return 0;
	return eps * 2 + (nb_peri-2) * ps;
}

void s_wall_thickness_set(float new_val)
{
	float diameter = get_float("nozzle_diameter");
	float nb = new_val / diameter;
	int int_nb = int(floor(nb+0.1));
	//print("float "+nb+" cast into "+int_nb+"\n");
	if (int_nb > 1 && int_nb < 4) {
		float ext_spacing = new_val / int_nb;
		set_float("external_perimeter_extrusion_spacing", ext_spacing);
		set_float("perimeter_extrusion_spacing", ext_spacing);
		set_custom_int(0,"wall_thickness_lines", int_nb);
	} else if(int_nb > 3) {
		//try with thin external
		float ext_spacing = diameter;
		float spacing = (new_val - ext_spacing * 2) / (int_nb - 2);
		if (spacing > diameter * 1.5) {
			// too different, get back to same value
			ext_spacing = new_val / int_nb;
			spacing = ext_spacing;
		}
		set_float("external_perimeter_extrusion_spacing", ext_spacing);
		set_float("perimeter_extrusion_spacing", spacing);
		set_custom_int(0,"wall_thickness_lines", int_nb);
	} else if(new_val == 0) {
		// fake 'disable' to not confuse people susi#2700
		set_custom_int(0,"wall_thickness_lines", 0);
		back_initial_value("external_perimeter_extrusion_spacing");
		back_initial_value("perimeter_extrusion_spacing");
	} else {
		back_custom_initial_value(0,"wall_thickness_lines");
		back_initial_value("external_perimeter_extrusion_spacing");
		back_initial_value("perimeter_extrusion_spacing");
		// refresh the displayed value to a valid one
		ask_for_refresh();
	}
//	ask_for_refresh();
}

// quick settings brim

float last_brim_val = 5;

int s_brim_get()
{
	float bw = get_float("brim_width");
	if (bw > 0) {
		last_brim_val = bw;
		return 1;
	}
	return 0;
}

void s_brim_set(bool new_val)
{
	if(new_val) {
		float bw = get_float("brim_width");
		set_float("brim_width", last_brim_val);
	} else {
			set_float("brim_width", 0);
	}
}

// quick settings support

int s_support_fff_get(string &out get_val)
{
	bool support_material = get_bool("support_material");
	if (!support_material) { // None
		return 0;
	}
	bool support_material_auto = get_bool("support_material_auto");
	if (!support_material_auto) { // For support enforcers only
		return 2;
	}
	bool support_material_buildplate_only = get_bool("support_material_buildplate_only");
	if (support_material_buildplate_only) { // Support on build plate only
		return 1;
	}
	// everywhere
	return 3;
}

void s_support_fff_set(string &in new_val, int idx)
{
	if(idx == 0) { // None
		back_initial_value("support_material_buildplate_only");
		back_initial_value("support_material_auto");
		set_bool("support_material", false);
	} else if(idx == 1) { // Support on build plate only
		set_bool("support_material_buildplate_only", true);
		set_bool("support_material_auto", true);
		set_bool("support_material", true);
	} else if(idx == 2) { // For support enforcers only
		set_bool("support_material_buildplate_only", false);
		set_bool("support_material_auto", false);
		set_bool("support_material", true);
	} else if(idx == 3) { // everywhere
		set_bool("support_material_buildplate_only", false);
		set_bool("support_material_auto", true);
		set_bool("support_material", true);
	}
}


// quick settings bed type (nematx)

int s_bed_fff_get(string &out get_val)
{
    int bed_temperature = get_int("bed_temperature");
    int fl_bed_temperature = get_int("first_layer_bed_temperature");
    if (bed_temperature == fl_bed_temperature) {
        if (bed_temperature == 130) {
            return 1; //glue
        }
        if (bed_temperature == 170) {
            return 2; //noglue
        }
    }
    return 0; // custom
}

void s_bed_fff_set(string &in new_val, int idx)
{
	if(idx == 0) { // custom
		back_initial_value("bed_temperature");
		back_initial_value("first_layer_bed_temperature");
	} else if(idx == 1) { // glue
		set_int("bed_temperature", 130);
		set_int("first_layer_bed_temperature", 130);
	} else if(idx == 2) { // noglue
		set_int("bed_temperature", 170);
		set_int("first_layer_bed_temperature", 170);
	}
}

//TODO to replicate prusa:
// brim_type
// cooling
// xy compensation (both)


//test:
//	setting:script:bool:easy:depends$enforce_full_fill_volume:label$fullfill-lol:s_fullfill
//	setting:script:int:easy:depends$perimeters:label$perimeters-lol:s_perimeter
//	setting:script:float:easy:depends$top_solid_min_thickness:label$thickness-lol:s_thickness
//	setting:script:percent:easy:depends$bridge_flow_ratio:label$bridgeflow-lol:s_bridgeflow
//	setting:script:string:easy:depends$notes:label$notes-lol:s_notes
//	setting:script:enum$b$bof$m$mouaif:easy:depends$no_perimeter_unsupported_algo:label$noperi-lol:s_noperi

int s_fullfill_get()
{
	if (get_bool("enforce_full_fill_volume")) return 1;
	return 0;
}
void s_fullfill_set(bool set)
{
	set_bool("enforce_full_fill_volume", set);
}


int s_perimeter_get()
{
	return get_int("perimeters");
}
void s_perimeter_set(int set)
{
	set_int("perimeters", set);
}


float s_thickness_get()
{
	return get_float("top_solid_min_thickness");
}
void s_thickness_set(float set)
{
	set_float("top_solid_min_thickness", set);
}


float s_bridgeflow_get()
{
	return get_float("bridge_flow_ratio");
}
void s_bridgeflow_set(float set)
{
	set_percent("bridge_flow_ratio", set);
}


void s_notes_get(string &out get_val)
{
	get_string("notes", get_val);
}
void s_notes_set(string &out set_val)
{
	set_string("notes", set_val);
}


int s_noperi_get(string &out get_val)
{
	return get_int("no_perimeter_unsupported_algo") == 0 ? 0 : 1;
}
void s_noperi_set(string &out set_val, int idx)
{
	//set_int("no_perimeter_unsupported_algo", idx == 0 ? 0 : 3);
	if (idx == 0) set_int("no_perimeter_unsupported_algo",0);
	else set_string("no_perimeter_unsupported_algo", "filled");
}


/// simple modes

// s_simple_fill_pattern
int s_simple_fill_pattern_get(string &out get_val)
{
	string seam_pos;
	get_string("fill_pattern", seam_pos);
    if (seam_pos == "rectilinear")
        return 0;
    if (seam_pos == "grid")
        return 1;
    if (seam_pos == "triangles")
        return 2;
    if (seam_pos == "stars")
        return 3;
    if (seam_pos == "cubic")
        return 4;
    if (seam_pos == "gyroid")
        return 5;
    if (seam_pos == "honeycomb")
        return 6;
    if (seam_pos == "lightning")
        return 7;
	return 8;
}

void s_simple_fill_pattern_set(string &in set_val, int idx)
{
    if (set_val != "other") {
        set_string("fill_pattern", set_val);
    } else {
        back_initial_value("fill_pattern");
    }
}

// s_simple_top_fill_pattern
int s_simple_top_fill_pattern_get(string &out get_val)
{
	string seam_pos;
	get_string("top_fill_pattern", seam_pos);
    if (seam_pos == "monotonic")
        return 0;
    if (seam_pos == "monotonicgapfill")
        return 1;
    if (seam_pos == "concentric")
        return 2;
    if (seam_pos == "concentricgapfill")
        return 3;
	return 4;
}

void s_simple_top_fill_pattern_set(string &in set_val, int idx)
{
    if (set_val != "other") {
        set_string("top_fill_pattern", set_val);
    } else {
        back_initial_value("fill_pattern");
    }
}

// s_easy_speed
int s_easy_speed_get(string &out get_val)
{
	float speed = get_computed_float("default_speed");
    if (speed < 25)
        return 0; // quality
    if (speed < 35)
        return 1; // normal
	return 2; //speed
}

void s_easy_speed_set(string &in set_val, int idx)
{
    if (idx == 0 ) {
        //quality
        set_float("default_speed", 20);
    } else if (idx == 1 ) {
        //normal
        set_float("default_speed", 30);
    } else {
        // speed
        set_float("default_speed", 50);
    }
}

// s_fill_angle
int s_fill_angle_get(string &out get_val)
{
    float incr = get_float("fill_angle_increment");
    bool can_cross = get_bool("fill_angle_cross");
    int nb_template_val = size("fill_angle_template");
    if (!can_cross && nb_template_val == 0 && incr == 0)
        return 0; // unidir
    if (can_cross && nb_template_val == 0 && incr == 0)
        return 1; // cross
    if (!can_cross && nb_template_val == 4 && incr == 0) {
        if(get_float_idx("fill_angle_template",0)==0
        && get_float_idx("fill_angle_template",1)==90
        && get_float_idx("fill_angle_template",2)==45
        && get_float_idx("fill_angle_template",3)==135) {
            return 2; //Quasi-Isotropic
        }
    }
	return 3; 
}

void s_fill_angle_set(string &in set_val, int idx)
{
    if (idx == 0) {
        //unidir
        set_float("fill_angle_increment", 0);
        set_bool("fill_angle_cross", false);
        clear("fill_angle_template");
    } else if (idx == 1 ) {
        //cross
        set_float("fill_angle_increment", 0);
        set_bool("fill_angle_cross", true);
        clear("fill_angle_template");
    } else if (idx == 2) {
        // Quasi-Isotropic
        set_float("fill_angle_increment", 0);
        set_bool("fill_angle_cross", false);
        clear("fill_angle_template");
        set_float_idx("fill_angle_template", 0, 0);
        set_float_idx("fill_angle_template", 1, 90);
        set_float_idx("fill_angle_template", 2, 45);
        set_float_idx("fill_angle_template", 3, 135);
    } else {
        back_initial_value("fill_angle_cross");
        back_initial_value("fill_angle_template");
    }
}

// s_has_skirts
int s_has_skirts_get()
{
    int skirts_count = get_int("skirts");
    if (skirts_count > 0)
        return 1;
    return 0;
}

void s_has_skirts_set(bool has)
{
    if (has) {
        set_int("skirts", 3);
    } else {
        set_int("skirts", 0);
    }
}

// s_support_material_angle
int s_support_material_angle_get(string &out get_val)
{
    bool has_sup = get_bool("support_material");
    bool is_auto = get_bool("support_material_auto");
    int angle = get_int("support_material_threshold");
    if (!has_sup || !is_auto) {
        return 0; // none
    }
    if (angle == 0) {
        return 1; // auto
    }
    if (angle < 25) {
        return 2; //easy
    }
    if (angle < 55) {
        return 3; //medium
    }
	return 4; //hard
}

void s_support_material_angle_set(string &in set_val, int idx)
{
    if (idx == 0) {
        //none
        set_bool("support_material", true);
        set_bool("support_material_auto", false);
        back_initial_value("support_material_threshold");
    } else if (idx == 1 ) {
        //auto
        set_bool("support_material", true);
        set_bool("support_material_auto", true);
        set_int("support_material_threshold", 0);
    } else if (idx == 2) {
        // easy
        set_bool("support_material", true);
        set_bool("support_material_auto", true);
        set_int("support_material_threshold", 20);
    } else if (idx == 3) {
        set_bool("support_material", true);
        set_bool("support_material_auto", true);
        set_int("support_material_threshold", 50);
    } else {
        set_bool("support_material", true);
        set_bool("support_material_auto", true);
        set_int("support_material_threshold", 70);
    }
}

