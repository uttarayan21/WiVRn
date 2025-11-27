/*
 * WiVRn VR streaming
 * Copyright (C) 2025  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "tracking_control.h"

void wivrn::tracking_control::add_request(device_id device, XrTime now, XrTime at_ns)
{
	std::lock_guard lock(mutex);
	samples.push_back(
	        {
	                .device = device,
	                .request_time = now,
	                .prediction = at_ns,
	        });
}

void wivrn::tracking_control::resolve(wivrn_connection &, XrTime display_time, XrDuration frame_time)
{
	spare.clear();
	{
		std::lock_guard lock(mutex);
		std::swap(samples, spare);
	}

	display_time %= frame_time;

	// Express request time relative to display time
	for (auto & item: spare)
		item.request_time = (item.request_time - display_time) % frame_time;
}
