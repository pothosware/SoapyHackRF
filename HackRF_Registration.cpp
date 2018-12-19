/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Wei Jiang
 * Copyright (c) 2015-2017 Josh Blum
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyHackRF.hpp"
#include <SoapySDR/Registry.hpp>

static std::map<std::string, SoapySDR::Kwargs> _cachedResults;

static std::vector<SoapySDR::Kwargs> find_HackRF(const SoapySDR::Kwargs &args)
{
	SoapyHackRFSession Sess;

	std::vector<SoapySDR::Kwargs> results;

	hackrf_device_list_t *list;

	list =hackrf_device_list();

	if (list->devicecount > 0) {
	
		for (int i = 0; i < list->devicecount; i++) {
		
			hackrf_device* device = NULL;
			uint8_t board_id = BOARD_ID_INVALID;
			read_partid_serialno_t read_partid_serialno;

			hackrf_device_list_open(list, i, &device);

			SoapySDR::Kwargs options;

			if (device!=NULL) {

				hackrf_board_id_read(device, &board_id);

				options["device"] = hackrf_board_id_name((hackrf_board_id) board_id);

				char version_str[100];

				hackrf_version_string_read(device, &version_str[0], 100);

				options["version"] = version_str;

				hackrf_board_partid_serialno_read(device, &read_partid_serialno);

				char part_id_str[100];

				sprintf(part_id_str, "%08x%08x", read_partid_serialno.part_id[0], read_partid_serialno.part_id[1]);

				options["part_id"] = part_id_str;

				char serial_str[100];
				sprintf(serial_str, "%08x%08x%08x%08x", read_partid_serialno.serial_no[0],
						read_partid_serialno.serial_no[1], read_partid_serialno.serial_no[2],
						read_partid_serialno.serial_no[3]);
				options["serial"] = serial_str;

				//generate a displayable label string with trimmed serial
				size_t ofs = 0;
				while (ofs < sizeof(serial_str) and serial_str[ofs] == '0') ofs++;
				char label_str[100];
				sprintf(label_str, "%s #%d %s", options["device"].c_str(), i, serial_str+ofs);
				options["label"] = label_str;

				//filter based on serial and idx
				const bool serialMatch = args.count("serial") == 0 or args.at("serial") == options["serial"];
				const bool idxMatch = args.count("hackrf") == 0 or std::stoi(args.at("hackrf")) == i;
				if (serialMatch and idxMatch)
				{
					results.push_back(options);
					_cachedResults[serial_str] = options;
				}

				hackrf_close(device);
			}
		
		}
	
	}

	hackrf_device_list_free(list);

	//fill in the cached results for claimed handles
	for (const auto &serial : HackRF_getClaimedSerials())
	{
		if (_cachedResults.count(serial) == 0) continue;
		if (args.count("serial") != 0 and args.at("serial") != serial) continue;
		results.push_back(_cachedResults.at(serial));
	}

	return results;
}

static SoapySDR::Device *make_HackRF(const SoapySDR::Kwargs &args)
{
    return new SoapyHackRF(args);
}

static SoapySDR::Registry register_hackrf("hackrf", &find_HackRF, &make_HackRF, SOAPY_SDR_ABI_VERSION);
