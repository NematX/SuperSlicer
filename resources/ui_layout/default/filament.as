
// s_has_skirts
int s_easy_temp_get()
{
    int temp = get_int("temperature");
    int ftemp = get_int("first_layer_temperature");
    if (temp > ftemp) {
        return temp;
    }
    return ftemp;
}

void s_easy_temp_set(int temp)
{
    set_int("temperature", temp);
    set_int("first_layer_temperature", temp);
}

// bed type

int s_bed_fil_fff_get(string &out get_val)
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

void s_bed_fil_fff_set(string &in new_val, int idx)
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

