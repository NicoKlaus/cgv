#include "vr_state_log.h"

void vr::vr_state_log::reset_timer()
{
	//TODO add implementation
}

void vr::vr_state_log::disable_log()
{
	log_storage_mode = SM_NONE;
	if (log_stream) {
		log_stream->flush();
		log_stream = nullptr;
	}
}

void vr::vr_state_log::log_vr_state(vr::vr_kit_state& state, int mode, int filter, double time)
{
	for (int i = 0; i < 4; ++i) {
		if (state.controller[i].status == vr::VRS_TRACKED) {
			if (mode && SM_IN_MEMORY) {
				this->controller_time_stamp.push_back(time);
				this->controller_id.push_back(i);
				if (filter && F_VIBRATION) {
					this->controller_vibration.push_back(state.controller[i].vibration);
				}
				if (filter && F_THROTTLE) {
					this->controller_axes.push_back(state.controller[i].axes);
				}
				if (filter && F_POSE) {
					this->controller_pose.push_back(state.controller[i].pose);
				}
				if (filter && F_BUTTON) {
					this->controller_button_flags.push_back(state.controller[i].button_flags);
				}
			}
			if (mode && SM_OSTREAM) {
				//C <timestamp> <controller_id> [P <pose>] [B <button-mask>] [T <throttle-state>] [V <vibration>]
				*(log_stream) << "C " << time << ' ' << i;
				if (filter && F_POSE) {
					*(log_stream) << " P " << state.controller[i].pose;
				}
				if (filter && F_BUTTON) {
					*(log_stream) << " B " << state.controller[i].button_flags;
				}
				if (filter && F_THROTTLE) {
					*(log_stream) << " T " << state.controller[i].axes;
				}
				if (filter && F_VIBRATION) {
					*(log_stream) << " V " << state.controller[i].vibration;
				}
				*(log_stream) << "\n";
			}
		}
	}

	if (state.hmd.status == VRS_TRACKED) {
		if (mode && SM_IN_MEMORY) {
			hmd_time_stamp.push_back(time);
			hmd_pose.push_back(state.hmd.pose);
		}
		if (mode && SM_OSTREAM) {
			*(log_stream) << "H " << time << " " << state.hmd.pose;
		}
	}
}

