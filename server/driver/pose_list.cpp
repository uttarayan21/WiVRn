/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
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

#include "pose_list.h"
#include "driver/clock_offset.h"
#include "math/m_api.h"
#include "math/m_eigen_interop.hpp"
#include "math/m_space.h"
#include "math/m_vec3.h"
#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt_cast.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <limits>
#include <ranges>

using namespace xrt::auxiliary::math;

namespace wivrn
{

xrt_space_relation pose_list::interpolate(const xrt_space_relation & a, const xrt_space_relation & b, float t)
{
	xrt_space_relation result;
	xrt_space_relation_flags flags = xrt_space_relation_flags(a.relation_flags & b.relation_flags);
	m_space_relation_interpolate(const_cast<xrt_space_relation *>(&a), const_cast<xrt_space_relation *>(&b), t, flags, &result);
	return result;
}

xrt_space_relation pose_list::extrapolate(const xrt_space_relation & a, const xrt_space_relation & b, int64_t ta, int64_t tb, int64_t t)
{
	float h = (tb - ta) / 1.e9;

	xrt_space_relation res = t < ta ? a : b;

	xrt_vec3 lin_vel = res.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT ? res.linear_velocity : (b.pose.position - a.pose.position) / h;

	float dt = (t - tb) / 1.e9;

	float dt2_over_2 = dt * dt / 2;
	res.pose.position = res.pose.position + lin_vel * dt;

	if (res.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT)
	{
		xrt_vec3 dtheta = res.angular_velocity * dt;
		xrt_quat dq;
		math_quat_exp(&dtheta, &dq);

		map_quat(res.pose.orientation) = map_quat(res.pose.orientation) * map_quat(dq);
	}

	return res;
}

bool pose_list::update_tracking(const from_headset::tracking & tracking, const clock_offset & offset)
{
	if (source)
		return false;

	for (const auto & pose: tracking.device_poses)
	{
		if (pose.device != device)
			continue;

		return add_sample(tracking.timestamp, convert_pose(pose), offset);
	}
	return true;
}

void pose_list::set_derived(pose_list * source, xrt_pose offset, bool force)
{
	if (force)
	{
		derive_forced = true;
	}
	else if (derive_forced)
	{
		return;
	}
	// TODO: check for loops?
	if (source == this)
		this->source = nullptr;
	else
	{
		this->offset = offset;
		this->source = source;
	}
}

std::tuple<std::chrono::nanoseconds, xrt_space_relation, device_id> pose_list::get_pose_at(XrTime at_timestamp_ns)
{
	if (auto source = this->source.load())
	{
		auto res = source->get_pose_at(at_timestamp_ns);
		math_pose_transform(&std::get<1>(res).pose, &offset, &std::get<1>(res).pose);
		return res;
	}

	return std::tuple_cat(get_at(at_timestamp_ns), std::make_tuple(device));
}

static xrt_space_relation_flags convert_flags(uint8_t flags)
{
	static_assert(int(from_headset::tracking::position_valid) == XRT_SPACE_RELATION_POSITION_VALID_BIT);
	static_assert(int(from_headset::tracking::orientation_valid) == XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);
	static_assert(int(from_headset::tracking::linear_velocity_valid) == XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT);
	static_assert(int(from_headset::tracking::angular_velocity_valid) == XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	static_assert(int(from_headset::tracking::orientation_tracked) == XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	static_assert(int(from_headset::tracking::position_tracked) == XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
	return xrt_space_relation_flags(flags);
}

xrt_space_relation pose_list::convert_pose(const from_headset::tracking::pose & pose)
{
	return xrt_space_relation{
	        .relation_flags = convert_flags(pose.flags),
	        .pose = xrt_cast(pose.pose),
	        .linear_velocity = xrt_cast(pose.linear_velocity),
	        .angular_velocity = xrt_cast(pose.angular_velocity),
	};
}

void pose_list::reset()
{
	std::lock_guard lock(mutex);
	data.fill({});
	last_request = os_monotonic_get_ns();
}

bool pose_list::add_sample(XrTime timestamp, const xrt_space_relation & sample, const clock_offset & offset)
{
	std::lock_guard lock(mutex);
	auto iter = std::ranges::min_element(data, {}, &std::pair<XrTime, xrt_space_relation>::first);

	timestamp = offset.from_headset(timestamp);
	*iter = {timestamp, sample};

	return timestamp - last_request < 1'000'000'000;
}

std::pair<std::chrono::nanoseconds, xrt_space_relation> pose_list::get_at(XrTime at_timestamp_ns)
{
	std::lock_guard lock(mutex);

	last_request = os_monotonic_get_ns();

	struct index
	{
		int abs_Δt;
		int i;
		bool has_value;
		bool has_deriv;
	};

	std::array<index, history_size> indices;
	for (const auto && [i, sample]: std::ranges::enumerate_view(data))
	{
		XrTime Δt = std::min<XrTime>(std::abs(sample.first - at_timestamp_ns), std::numeric_limits<int>::max());

		const auto flag_value = XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
		const auto flag_deriv = XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;

		bool has_value = (sample.second.relation_flags & flag_value) == flag_value;
		bool has_deriv = (sample.second.relation_flags & flag_deriv) == flag_deriv;

		indices[i] = {(int)Δt, (int)i, has_value, has_deriv};
	}

	std::ranges::sort(indices, {}, &index::abs_Δt);

	int n_values = std::min<int>(nb_samples, std::ranges::count_if(indices, [&](const index & value) { return value.has_value; }));
	int n_derivs = std::min<int>(nb_samples, std::ranges::count_if(indices, [&](const index & value) { return value.has_deriv; }));

	Eigen::Matrix<float, Eigen::Dynamic, polynomial_order + 1> A(n_values + n_derivs, polynomial_order + 1);
	Eigen::Matrix<float, Eigen::Dynamic, 7> b(n_values + n_derivs, 7); // x, y, z, qw, qx, qy, qz

	for (int i = 0; i < n_values; ++i)
	{
		auto sample = data[indices[i].i];
		float x = (sample.first - at_timestamp_ns) / 1.e9;
		float xⁱ = 1;

		for (int j = 0; j < polynomial_order; ++j)
		{
			A(i, j) = xⁱ;
			xⁱ *= x;
		}

		b(i, 0) = sample.second.pose.position.x;
		b(i, 1) = sample.second.pose.position.y;
		b(i, 2) = sample.second.pose.position.z;
		b(i, 3) = sample.second.pose.orientation.x;
		b(i, 4) = sample.second.pose.orientation.y;
		b(i, 5) = sample.second.pose.orientation.z;
		b(i, 6) = sample.second.pose.orientation.w;
	}

	for (int i = 0; i < n_derivs; ++i)
	{
		auto sample = data[indices[i].i];
		float x = (sample.first - at_timestamp_ns) / 1.e9;
		float xⁱ = 1;

		A(i + n_values, 0) = 0;
		for (int j = 1; j < polynomial_order; ++j)
		{
			A(i + n_values, j) = τ * j * xⁱ;
			xⁱ *= x;
		}

		b(i + n_values, 0) = τ * sample.second.linear_velocity.x;
		b(i + n_values, 1) = τ * sample.second.linear_velocity.y;
		b(i + n_values, 2) = τ * sample.second.linear_velocity.z;

		Eigen::Quaternionf q = map_quat(sample.second.pose.orientation);
		Eigen::Quaternionf ω{0, sample.second.angular_velocity.x, sample.second.angular_velocity.y, sample.second.angular_velocity.z};
		Eigen::Quaternionf two_dq = ω * q;

		b(i + n_values, 3) = 0.5 * τ * two_dq.x();
		b(i + n_values, 4) = 0.5 * τ * two_dq.y();
		b(i + n_values, 5) = 0.5 * τ * two_dq.z();
		b(i + n_values, 6) = 0.5 * τ * two_dq.w();
	}

	Eigen::Matrix<float, 7, polynomial_order + 1> sol = (A.transpose() * A).ldlt().solve(A.transpose() * b).transpose();

	xrt_space_relation ret = XRT_SPACE_RELATION_ZERO;
	map_vec3(ret.pose.position) = sol.block<3, 1>(0, 0);
	map_vec3(ret.linear_velocity) = sol.block<3, 1>(0, 1);

	map_quat(ret.pose.orientation) = sol.block<4, 1>(3, 0).normalized();

	Eigen::Quaternionf dq = Eigen::Quaternionf::FromCoeffsScalarLast(sol(3, 1), sol(4, 1), sol(5, 1), sol(6, 1));
	Eigen::Quaternionf half_ω = dq * map_quat(ret.pose.orientation).conjugate();
	ret.angular_velocity = {2 * half_ω.x(), 2 * half_ω.y(), 2 * half_ω.z()};

	ret.relation_flags = data[indices[0].i].second.relation_flags;

	std::chrono::nanoseconds ex(std::max<XrDuration>(0, at_timestamp_ns - data[indices[0].i].first));

	return {ex, ret};
}

} // namespace wivrn
