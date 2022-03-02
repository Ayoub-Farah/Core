/*
 * Copyright (c) 2022 LAAS-CNRS
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGLPV2.1
 */

/**
 * @date   2022
 * @author Clément Foucher <clement.foucher@laas.fr>
 */


#include "ngnd.h"

static const struct device* ngnd_switch = nullptr;


void ngnd_config_on()
{
	if (ngnd_switch == NULL)
	{
		ngnd_switch = device_get_binding(NGND_DEVICE);
	}

	ngnd_set(ngnd_switch, 1);
}

void ngnd_config_off()
{
	if (ngnd_switch == NULL)
	{
		ngnd_switch = device_get_binding(NGND_DEVICE);
	}

	ngnd_set(ngnd_switch, 0);
}
