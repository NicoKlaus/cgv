#include "vr_state_log.h"

/* Logfile lines
<pose> : 12 floats representing a 4*3 matrix (column major)
<button-mask>: 32 bit integer
<throttle-state>: 8 floats for the axes of the controller
<vibration>: 2 floats for the vibration intensity
<timestamp>: time in ms

C <timestamp> <controller_id> [P <pose>] [B <button-mask>] [T <throttle-state>] [V <vibration>]
H <timestamp> <pose>
*/


void vr::vr_state_log::disable_log()
{
	log_storage_mode = SM_NONE;
}

vr::vr_state_log::vr_state_log(std::istringstream& is, char terminator) {
	load_state(is, terminator);
}

void vr::vr_state_log::log_vr_state(const vr::vr_kit_state& state, const int mode, const int filter, const double time,std::ostream* log_stream)
{
	if (!setting_locked)
		return;
	for (int i = 0; i < 4; ++i) {
		if (state.controller[i].status == vr::VRS_TRACKED) {
			if (mode && SM_IN_MEMORY) {
				this->controller_time_stamp.push_back(time);
				this->controller_id.push_back(i);
				if (filter & F_VIBRATION) {
					vec2 vibration = vec2(state.controller[i].vibration[0], state.controller[i].vibration[1]);
					this->controller_vibration.push_back(vibration);
				}
				if (filter & F_THROTTLE) {
					vec8 axes;
					for (int i = 0; i < 8; ++i) {
						axes(i) = state.controller[i].axes[i];
					}
					this->controller_axes.push_back(axes);
				}
				if (filter & F_POSE) {
					mat34 pose = mat34(3, 4, state.controller[i].pose);
					this->controller_pose.push_back(pose);
				}
				if (filter & F_BUTTON) {
					this->controller_button_flags.push_back(state.controller[i].button_flags);
				}
			}
			if (mode & SM_OSTREAM) {
				//C <timestamp> <controller_id> [P <pose>] [B <button-mask>] [T <throttle-state>] [V <vibration>]
				*(log_stream) << "C " << time << ' ' << i;
				if (filter & F_POSE) {
					*(log_stream) << " P";
					for (int j = 0; j < 12; ++j)
						*(log_stream) << ' ' << state.controller[i].pose[j];
				}
				if (filter & F_BUTTON) {
					*(log_stream) << " B " << state.controller[i].button_flags;
				}
				if (filter & F_THROTTLE) {
					*(log_stream) << " T";
					for (int j = 0; j < 8; ++j)
						*(log_stream) << ' ' << state.controller[i].axes[j];
				}
				if (filter & F_VIBRATION) {
					*(log_stream) << " V";
					for (int j = 0; j < 2; ++j)
						*(log_stream) << ' ' << state.controller[i].vibration[j];
				}
				*(log_stream) << "\n";
			}
		}
	}

	if (state.hmd.status == VRS_TRACKED) {
		if (mode & SM_IN_MEMORY) {
			hmd_time_stamp.push_back(time);
			mat34 pose = mat34(3, 4, state.hmd.pose);
			hmd_pose.push_back(pose);
		}
		if ((mode & SM_OSTREAM) && log_stream) {
			*(log_stream) << "H " << time;
			for (int j = 0; j < 12; ++j)
				*(log_stream) << ' ' << state.hmd.pose[j];
		}
	}
}

void  vr::vr_state_log::load_state(std::istringstream& is, const char terminator) {
	typedef cgv::media::color<float, cgv::media::HLS> hls;

	std::string line;
	int line_number = 0;

	int prev_pose_index[2] = { -1,-1 };
	int last_pose_index[2] = { -1,-1 };

	try {
		while (!is.eof()) {
			std::getline(is, line); 	//read a line
			std::istringstream l(line);
			int32_t controller_id = -1, grab_id = -1;
			double time_in_ms;
			std::string event_type;
			++line_number;

			std::string type;

			l >> type;

			if (type == "C") { //is controller state

			}
			else if (type == "H") { //is head tracker

			}
		}
	}
	catch (std::string err) {
		std::cerr << err << '\n' << "vr_state_log::load_log: failed parsing data from stream\n";
	}
}