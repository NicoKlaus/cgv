#include "vr_log.h"

/* Logfile lines
<pose> : 12 floats representing a 4*3 matrix (column major)
<button-mask>: 32 bit integer
<axis-state>: 8 floats for the axes of the controller
<vibration>: 2 floats for the vibration intensity
<timestamp>: time in ms

<timestamp> (C <controller_id> [P <pose>] [B <button-mask>] [A <axes-state>] [V <vibration>])* [H <pose>]
*/


const std::string filter_to_string(vr::vr_log::Filter f){
	switch (f) {
	case vr::vr_log::F_AXES:
		return "AXES";
	case vr::vr_log::F_BUTTON:
		return "BUTTON";
	case vr::vr_log::F_HMD:
		return "HMD";
	case vr::vr_log::F_POSE:
		return "POSE";
	case vr::vr_log::F_VIBRATION:
		return "VIBRATION";
	default:
		return "UNKNOWN_FILTER";
	}
}

const std::unordered_map<std::string, vr::vr_log::Filter> filter_map = {
   {"AXES", vr::vr_log::F_AXES},
   {"BUTTON", vr::vr_log::F_BUTTON},
   {"HMD", vr::vr_log::F_HMD },
   {"POSE", vr::vr_log::F_POSE},
   {"VIBRATION", vr::vr_log::F_VIBRATION},
   {"UNKNOWN_FILTER", vr::vr_log::F_NONE}
};


const vr::vr_log::Filter filter_from_string(const std::string& f) {
	const auto it = filter_map.find(f);
	if (it != filter_map.cend()) {
		return (*it).second;
	}
	return vr::vr_log::F_NONE;
}


void vr::vr_log::disable_log()
{
	log_storage_mode = SM_NONE;
}

void vr::vr_log::enable_in_memory_log()
{
	if (!setting_locked)
		log_storage_mode = log_storage_mode | SM_IN_MEMORY;
}

void vr::vr_log::enable_ostream_log()
{
	if (!setting_locked)
		log_storage_mode = log_storage_mode | SM_OSTREAM;
}


vr::vr_log::vr_log(std::istringstream& is) {
	load_state(is);
}

void vr::vr_log::log_vr_state(const vr::vr_kit_state& state, const int mode, const int filter, const double time,std::ostream* log_stream)
{
	if (!setting_locked)
		return;
	++nr_vr_states;
	//time stamp
	if (mode & SM_IN_MEMORY) {
		this->time_stamp.push_back(time);
	}
	if (mode & SM_OSTREAM) {
		*(log_stream) << time;
	}

	//controller state
	for (int i = 0; i < 4; ++i) {
		controller_status->push_back(state.controller[i].status);
		if (mode & SM_IN_MEMORY) {
			if (filter & F_VIBRATION) {
				vec2 vibration = vec2(state.controller[i].vibration[0], state.controller[i].vibration[1]);
				this->controller_vibration[i].push_back(vibration);
			}
			if (filter & F_AXES) {
				vec8 axes;
				for (int i = 0; i < 8; ++i) {
					axes(i) = state.controller[i].axes[i];
				}
				this->controller_axes[i].push_back(axes);
			}
			if (filter & F_POSE) {
				mat34 pose = mat34(3, 4, state.controller[i].pose);
				this->controller_pose[i].push_back(pose);
			}
			if (filter & F_BUTTON) {
				this->controller_button_flags[i].push_back(state.controller[i].button_flags);
			}
		}
		if (mode & SM_OSTREAM) {
			//C <timestamp> <controller_id> [P <pose>] [B <button-mask>] [T <throttle-state>] [V <vibration>]
			*(log_stream) << " C " << i;
			if (filter & F_POSE) {
				*(log_stream) << " P";
				for (int j = 0; j < 12; ++j)
					*(log_stream) << ' ' << state.controller[i].pose[j];
			}
			if (filter & F_BUTTON) {
				*(log_stream) << " B " << state.controller[i].button_flags;
			}
			if (filter & F_AXES) {
				*(log_stream) << " A";
				for (int j = 0; j < 8; ++j)
					*(log_stream) << ' ' << state.controller[i].axes[j];
			}
			if (filter & F_VIBRATION) {
				*(log_stream) << " V";
				for (int j = 0; j < 2; ++j)
					*(log_stream) << ' ' << state.controller[i].vibration[j];
			}
		}
	}

	//hmd state
	if (filter & F_HMD) {
		mat34 pose = mat34(3, 4, state.hmd.pose);
		if (mode & SM_IN_MEMORY) {
			hmd_pose.push_back(pose);
			hmd_status.push_back(state.hmd.status);
		}
		if ((mode & SM_OSTREAM) && log_stream) {
			*(log_stream) << "H " << time;
			for (int j = 0; j < 12; ++j)
				*(log_stream) << ' ' << state.hmd.pose[j];
		}
	}
	//end line
	if ((mode & SM_OSTREAM) && log_stream) {
		*(log_stream) << '\n';
	}
}

template <typename T,unsigned SIZE>
void parse_array(std::istringstream& line,T* storage) {
	for (int i = 0; i < SIZE; ++i) {
		line >> storage[i];
	}
}

unsigned parse_controller_state(std::istringstream& line, vr::vr_controller_state& state) {
	unsigned filter = 0;
	std::string cinfo_type;
	line >> cinfo_type;

	if (cinfo_type == "P") {
		filter |= vr::vr_log::F_POSE;
			parse_array<float, 12>(line, state.pose);

	}
	return filter;
}

//returns active filters found
unsigned parse_vr_kit_state(std::istringstream& line, vr::vr_kit_state& state,double& time) {
	unsigned filter = 0;
	while (!line.eof()) {
		std::string type;
		line >> time >> type;
		if (type == "C") { //parse controller info
			int cid = -1; //controller id
			line >> cid;
			if (cid > 0 && cid < 2)
				filter |= parse_controller_state(line, state.controller[cid]);
			else
				throw std::string("invalid controller id");
		}
		else if (type == "H") { //parse hmd info
			filter |= vr::vr_log::F_HMD;
			parse_array<float, 12>(line, state.hmd.pose);
		}
	}
	return 0;
}


bool vr::vr_log::load_state(std::istringstream& is) {
	//log lines look like this: 
	//	<timestamp> (C <controller_id>[P <pose>][B <button - mask>][A <axes - state>][V <vibration>])* [H <pose>]
	if (setting_locked)
		return false;
	log_storage_mode = SM_IN_MEMORY;
	set_filter(F_ALL);
	lock_settings();
	//write log
	
	try {
		while (!is.eof()) {
			std::string line;
			std::getline(is, line);
			std::istringstream l(line);
			double time = -1;
			vr::vr_kit_state state;
			filters &= parse_vr_kit_state(l,state,time);
			log_vr_state(state, time);
		}
	}
	catch (std::string err) {
		return false;
	}

	disable_log();
	assert(false);
	return false;
}