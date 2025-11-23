/*
 * WiVRn VR streaming
 * Copyright (C) 2025  Guillaume Meunier <guillaume.meunier@centraliens.net>
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

#pragma once

// xlib sucks
#ifdef Success
#undef Success
#endif

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <optional>
#include <ranges>

#include "openxr/openxr.h"

template <int N, int polynomial_order = 2, int stored_samples = 10>
class polynomial_interpolator
{
public:
	using value_type = Eigen::Vector<float, N>;

	struct sample
	{
		XrTime timestamp = std::numeric_limits<XrTime>::lowest();
		std::optional<value_type> y;
		std::optional<value_type> dy;
	};

private:
	std::array<sample, stored_samples> data;

public:
	void reset()
	{
		data.fill({});
	}

	void add_sample(const sample & s)
	{
		*std::ranges::min_element(data, {}, &sample::timestamp) = s;
	}

	sample get_at(XrTime timestamp, XrDuration max_Δt, XrDuration max_extrapolation, float τ)
	{
		Eigen::Matrix<float, 2 * stored_samples, polynomial_order + 1> A;
		Eigen::Matrix<float, 2 * stored_samples, N> b;

		auto closest_sample = std::ranges::min_element(data, {}, [&](const sample & sample) {
			if (sample.timestamp == std::numeric_limits<XrTime>::lowest())
				return std::numeric_limits<XrTime>::max();
			else
				return std::abs(sample.timestamp - timestamp);
		});

		if (not closest_sample->y)
			return {};

		timestamp = std::min(timestamp, closest_sample->timestamp + max_extrapolation);

		int row = 0;
		for (const auto && [i, sample]: std::ranges::enumerate_view(data))
		{
			int abs_Δt = std::min<XrDuration>(std::abs(sample.timestamp - closest_sample->timestamp), std::numeric_limits<int>::max());

			if (abs_Δt > max_Δt or not sample.y.has_value())
				continue;

			float Δt = (sample.timestamp - timestamp) * 1.e-9;
			float Δtⁱ = 1;
			for (int i = 0; i <= polynomial_order; ++i)
			{
				A(row, i) = Δtⁱ;
				Δtⁱ *= Δt;
			}
			b.template block<1, N>(row, 0) = *sample.y;
			row++;

			if (sample.dy.has_value())
			{
				A(row, 0) = 0;

				Δtⁱ = 1;
				for (int i = 1; i <= polynomial_order; ++i)
				{
					A(row, i) = τ * i * Δtⁱ;
					Δtⁱ *= Δt;
				}
				b.template block<1, N>(row, 0) = *sample.dy * τ;
				row++;
			}
		}

		if (row == 0)
			return {};

		if (row == 1)
			return *closest_sample;

		auto Aprime = A.block(0, 0, row, polynomial_order + 1);
		auto bprime = b.block(0, 0, row, N);

		Eigen::Matrix<float, N, polynomial_order + 1> sol = (Aprime.transpose() * Aprime).ldlt().solve(Aprime.transpose() * bprime).transpose();
		return sample{
		        .timestamp = closest_sample->timestamp,
		        .y = sol.template block<N, 1>(0, 0),
		        .dy = sol.template block<N, 1>(0, 1),
		};
	}
};
