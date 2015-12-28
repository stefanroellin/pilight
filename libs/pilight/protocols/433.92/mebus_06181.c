/*
	Copyright (C) 2014 CurlyMo

	This file is part of pilight.

	pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

	pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

// a little bit adapted for Mebus 06181 Sensors by Stefan Roellin

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../../core/pilight.h"
#include "../../core/common.h"
#include "../../core/dso.h"
#include "../../core/log.h"
#include "../protocol.h"
#include "../../core/binary.h"
#include "../../core/gc.h"
#include "mebus_06181.h"

//
// Protocol characteristics: Bit relevant: low signal length: bit = 0 about 2000 us, Bit = 1 about 4000 us 
// valid footer length between 7000 and 10000 us (6970 and 10030 us)

#define PULSE_MULTIPLIER	12
#define MIN_PULSE_LENGTH	205 // min zero signal after sequence (..x 34)us
#define AVG_PULSE_LENGTH	250 // limit puls length for binary analyses x PULSE_MULTIPLIER
#define MAX_PULSE_LENGTH	295 // max zero signal after sequence(..x 34)us
#define MIN_RAW_LENGTH		74
#define MAX_RAW_LENGTH		74
#define RAW_LENGTH		74

// 1110 | 01      | 10110 | 1  | 11110100000 | 0    | 0001              | 1010               | 0110
// ID   | Channel | ?     | TX | Temperature | Sign | Humidity low byte | Humidity high byte | Checksum
//
// see http://www.mikrocontroller.net/topic/38129#281913
//
// remark: ID changes after battery exchange

#define ID_FIRST_BIT 0
#define ID_LAST_BIT 3
#define CHANNEL_FIRST_BIT 4
#define CHANNEL_LAST_BIT 5
#define TEMPERATURE_FIRST_BIT 12
#define TEMPERATURE_LAST_BIT 22
#define TEMPERATURE_SIGN_BIT 23
#define HUMIDITY_LOW_BYTE_FIRST_BIT 24
#define HUMIDITY_LOW_BYTE_LAST_BIT 27
#define HUMIDITY_HIGH_BYTE_FIRST_BIT 28
#define HUMIDITY_HIGH_BYTE_LAST_BIT 31
#define CHECKSUM_FIRST_BIT 32
#define CHECKSUM_LAST_BIT 35

typedef struct settings_t {
	double id;
	double channel;
	double temp;
	double humi;
	struct settings_t *next;
} settings_t;

static struct settings_t *settings = NULL;

static int validate(void) {
	if(mebus_06181->rawlen == RAW_LENGTH) {
		if(mebus_06181->raw[mebus_06181->rawlen-1] >= (MIN_PULSE_LENGTH*PULSE_DIV) &&
		   mebus_06181->raw[mebus_06181->rawlen-1] <= (MAX_PULSE_LENGTH*PULSE_DIV)) {
			return 0;
		}
	}
	return -1;
}

static void fillBinary(int* binary) {
	int i = 0;
	int x;
	for(x=1; x<mebus_06181->rawlen; x+=2) {
		if(mebus_06181->raw[x] > AVG_PULSE_LENGTH*PULSE_MULTIPLIER) {
			binary[i++] = 1;
		} else {
			binary[i++] = 0;
		}
	}
}

static int validateChecksum(int* binary) {
	int i;
	int sum = 0;
	int checksum = binToDec(binary, CHECKSUM_FIRST_BIT, CHECKSUM_LAST_BIT);
	for (i = 0; i < 8; ++i) {
		sum += binToDec(binary, i*4, i*4 + 3);
	}

	if ((sum & 0x0F) == checksum) {
		return 1;
	}

	return 0;
}

static void parseCode(void) {
	int id;
	int channel;
	int binary[RAW_LENGTH/2];
	double temperature;
	double humidity;

	fillBinary(binary);

	if (!validateChecksum(binary)) {
		return;
	}

        id = binToDecRev(binary, 0, 3);
        channel = binToDecRev(binary, 4, 5);

	temperature = (double)binToDec(binary, TEMPERATURE_FIRST_BIT, TEMPERATURE_LAST_BIT);
	if (binary[TEMPERATURE_SIGN_BIT] == 1) {
		temperature -= 2048.0;
	}
	temperature /= 10.0;
	
	humidity = 10.0*binToDec(binary, HUMIDITY_HIGH_BYTE_FIRST_BIT, HUMIDITY_HIGH_BYTE_LAST_BIT);
        humidity += binToDec(binary, HUMIDITY_LOW_BYTE_FIRST_BIT, HUMIDITY_LOW_BYTE_LAST_BIT);

	struct settings_t *tmp = settings;
	while(tmp) {
		if(fabs(tmp->id-id) < EPSILON && fabs(tmp->channel-channel) < EPSILON){
			temperature += tmp->temp;
			humidity += tmp->humi;
			break;
		}
		tmp = tmp->next;
	}

	mebus_06181->message = json_mkobject();
	json_append_member(mebus_06181->message, "id", json_mknumber(id, 0));
	json_append_member(mebus_06181->message, "channel", json_mknumber(channel, 0));
	json_append_member(mebus_06181->message, "temperature", json_mknumber(temperature, 1));
	json_append_member(mebus_06181->message, "humidity", json_mknumber(humidity, 1));
}

static int checkValues(struct JsonNode *jvalues) {
	struct JsonNode *jid = NULL;

	if((jid = json_find_member(jvalues, "id"))) {
		struct settings_t *snode = NULL;
		struct JsonNode *jchild = NULL;
		struct JsonNode *jchild1 = NULL;
		double id = -1;
		double channel = -1;
		int match = 0;

		jchild = json_first_child(jid);
		while(jchild) {
			jchild1 = json_first_child(jchild);
			while(jchild1) {
				if(strcmp(jchild1->key, "id") == 0) {
					id = jchild1->number_;
				}
				if(strcmp(jchild1->key, "channel") == 0) {
					channel = jchild1->number_;
				}
				jchild1 = jchild1->next;
			}
			jchild = jchild->next;
		}

		struct settings_t *tmp = settings;
		while(tmp) {
			if(fabs(tmp->id-id) < EPSILON && fabs(tmp->channel-channel) < EPSILON) {
				match = 1;
				break;
			}
			tmp = tmp->next;
		}

		if(match == 0) {
			if((snode = MALLOC(sizeof(struct settings_t))) == NULL) {
				fprintf(stderr, "out of memory\n");
				exit(EXIT_FAILURE);
			}
			snode->id = id;
			snode->channel = channel;
			snode->temp = 0;
			snode->humi = 0;

			json_find_number(jvalues, "temperature-offset", &snode->temp);
			json_find_number(jvalues, "humidity-offset", &snode->humi);

			snode->next = settings;
			settings = snode;
		}
	}
	return 0;
}

static void gc(void) {
	struct settings_t *tmp = NULL;
	while(settings) {
		tmp = settings;
		settings = settings->next;
		FREE(tmp);
	}
	if(settings != NULL) {
		FREE(settings);
	}
}

#if !defined(MODULE) && !defined(_WIN32)
__attribute__((weak))
#endif
void mebus06181Init(void) {

	protocol_register(&mebus_06181);
	protocol_set_id(mebus_06181, "mebus_06181");
	protocol_device_add(mebus_06181, "mebus_06181", "RF WIRELESS Temp Sensor");
	mebus_06181->devtype = WEATHER;
	mebus_06181->hwtype = RF433;
	mebus_06181->minrawlen = MIN_RAW_LENGTH;
	mebus_06181->maxrawlen = MAX_RAW_LENGTH;
	mebus_06181->maxgaplen = MAX_PULSE_LENGTH*PULSE_DIV;
	mebus_06181->mingaplen = MIN_PULSE_LENGTH*PULSE_DIV;

	options_add(&mebus_06181->options, 't', "temperature", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "^[0-9]{1,3}$");
        options_add(&mebus_06181->options, 'h', "humidity", OPTION_HAS_VALUE, DEVICES_VALUE, JSON_NUMBER, NULL, "[0-9]");
	options_add(&mebus_06181->options, 'i', "id", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[0-9]");
	options_add(&mebus_06181->options, 'c', "channel", OPTION_HAS_VALUE, DEVICES_ID, JSON_NUMBER, NULL, "[0-9]");

	options_add(&mebus_06181->options, 0, "temperature-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
	options_add(&mebus_06181->options, 0, "temperature-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)0, "[0-9]");
	options_add(&mebus_06181->options, 0, "show-temperature", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");
        options_add(&mebus_06181->options, 0, "humidity-offset", OPTION_HAS_VALUE, DEVICES_SETTING, JSON_NUMBER, (void *)0, "[0-9]");
        options_add(&mebus_06181->options, 0, "humidity-decimals", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "[0-9]");
        options_add(&mebus_06181->options, 0, "show-humidity", OPTION_HAS_VALUE, GUI_SETTING, JSON_NUMBER, (void *)1, "^[10]{1}$");

	mebus_06181->parseCode=&parseCode;
	mebus_06181->checkValues=&checkValues;
	mebus_06181->validate=&validate;
	mebus_06181->gc=&gc;
}

#ifdef MODULAR
void compatibility(const char **version, const char **commit) {
	module->name = "mebus_06181";
	module->version = "1.0";
	module->reqversion = "6.0";
	module->reqcommit = "84";
}

void init(void) {
	mebus06181Init();
}
#endif

