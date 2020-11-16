#pragma once
#include <cgv/render/render_types.h>
#include <vector>
#include <unordered_map>
#include <ostream>

#include <libs/vr/vr_state.h>
#include "vr_driver.h"

#include "lib_begin.h"

namespace vr {
	//! helper struct for logging vr events
	struct CGV_API vr_state_log : public cgv::render::render_types {
		template<class T>
		using container = std::vector<T, std::allocator<T>>;
		using vec8 = cgv::math::fvec<float, 8>;
		enum StorageMode {
			SM_IN_MEMORY = 1,
			SM_OSTREAM = 2,
			SM_IN_MEMORY_AND_OSTREAM = 3,
			SM_NONE = 0
		};

		enum Filter {
			F_POSE = 1,
			F_BUTTON = 2,
			F_THROTTLE = 4,
			F_VIBRATION = 8,
			F_HMD = 16,
			F_ALL = 31
		};

	private:
		bool setting_locked = false;
		std::ostream* log_stream;
		int log_storage_mode = SM_NONE;
		int filters = 0;
	public:
		//hmd state
		container<mat34> hmd_pose;
		container<double> hmd_time_stamp;
		//controller states
		container<double> controller_time_stamp;
		container<int8_t> controller_id;
		container<vec8> controller_axes;
		container<mat34> controller_pose;
		container<vec2> controller_vibration;
		container<unsigned> controller_button_flags;

	protected:
		void log_vr_state(const vr::vr_kit_state& state, const int mode, const int filter, const double time);
	public:
		inline void log_vr_state(const vr::vr_kit_state& state, const double& time) {
			log_vr_state(state, log_storage_mode, filters, time);
		}
		// close streams and disable in memory logging
		void disable_log();
		/// enable in memory log
		inline void enable_in_memory_log() {
			if (!setting_locked)
				log_storage_mode = log_storage_mode | SM_IN_MEMORY;
		}

		/// enable writing to provided ostream. disable_log must be used before deleting the given ostream object
		inline void enable_ostream_log(std::ostream& os) {
			if (setting_locked)
				return;
			log_storage_mode = log_storage_mode | SM_OSTREAM;
			log_stream = &os;
		}
		inline void set_filter(int f) {
			if (setting_locked)
				return;
			filters = f;
		}
		/// prevent changes to settings and enables log_vr_state methods
		inline void lock_settings() {
			setting_locked = true;
		}
	};
}

#include <cgv/config/lib_end.h>