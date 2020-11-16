#include "vr_state_log.h"

void vr::vr_state_log::disable_log()
{
	log_storage_mode = SM_NONE;
	if (log_stream) {
		log_stream->flush();
		log_stream = nullptr;
	}
}

void vr::vr_state_log::log_vr_state(const vr::vr_kit_state& state, const int mode, const int filter, const double time)
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
		if (mode & SM_OSTREAM) {
			*(log_stream) << "H " << time;
			for (int j = 0; j < 12; ++j)
				*(log_stream) << ' ' << state.hmd.pose[j];
		}
	}
}

