/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

#include "nmea.h"

NMEA::NMEA(const int &fd, struct vehicle_gps_position_s *gps_position):
	_fd(fd),
	_gps_position(gps_position)
{
	decode_init();
	_decode_state = NME_DECODE_UNINIT;
	_rx_buffer_bytes = 0;
	_satellites_count = 0;
	_satellites_visible = 0;
}

NMEA::~NMEA()
{
}


char  NMEA::read_char()
{
	if (_parse_error) {
		return 0;
	}

	_parse_pos++;

	if (*(_parse_pos) == ',' || *(_parse_pos) == '*') {
		return '?';
	}

	return *(_parse_pos++);
}

int32_t  NMEA::read_int()
{
	if (_parse_error) {
		return 0;
	}

	_parse_pos++;

	if (*(_parse_pos) == ',' || *(_parse_pos) == '*') {
		return 0;
	}

	int8_t n = 0;
	int32_t d = 0;
	int8_t neg = 0;
	int8_t n_max_digit = 9;

	if (*_parse_pos == '-') { neg = 1; _parse_pos++; }

	else if (*_parse_pos == '+')     { _parse_pos++; }

	while (*_parse_pos >= '0' && *_parse_pos <= '9') {
		d = d * 10 + (*(_parse_pos++) - '0');
		n++;

		if (n_max_digit > 0 && n == n_max_digit) break;
	}

	if (n == 0 || n > 10) {
		_parse_error = true;
		return 0;
	}

	if (neg) {
		return -d;

	} else {
		return d;
	}
}

double  NMEA::read_float()
{
	if (_parse_error) {
		return 0.0;
	}

	double f = 0.0, div = 1.0;
	int32_t d_int;
	int8_t n = 0, isneg = 0;
	int8_t n_max_int = 9, n_max_frac = 9;

	if (*_parse_pos == '-') {
		isneg = 1;
	}

	d_int = read_int();

	if (_parse_error) {
		return 0.0;
	}

	if (*(_parse_pos) == ',' || *(_parse_pos) == '*') {
		return 0.0;
	}

	if (*(_parse_pos) == '.') {
		_parse_pos++;

		while (*_parse_pos >= '0' && *_parse_pos <= '9') {
			f = f * (10.0) + (float)(*(_parse_pos++) - '0');
			div *= (0.1);
			n++;

			if (n_max_frac > 0 && n == n_max_frac) break;
		}

	} else if (n_max_frac > 0)  {
		_parse_error = true;
		return 0;
	}

	if (isneg) {
		return (double)d_int - f * div;

	} else {
		return (double)d_int + f * div;
	}
}

//All NMEA descriptions are taken from
//http://www.trimble.com/OEM_ReceiverHelp/V4.44/en/NMEA-0183messages_MessageOverview.html
int NMEA::handle_message(int len)
{
	if (len < 7) return 0;

	int commas_count = 0;

	for (int i = 0 ; i < len; i++) {
		if (_rx_buffer[i] == ',')commas_count++;
	}

	_parse_pos = (char *)(_rx_buffer + 6);
	_parse_error = false;

	//warnx((char *)_rx_buffer);

	if ((memcmp(_rx_buffer + 3, "GGA,", 3) == 0) && (commas_count == 14)) {
		/*
		  Time, position, and fix related data
		  An example of the GBS message string is:

		  $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0,0031*4F

		  Note - The data string exceeds the NMEA standard length.
		  GGA message fields
		  Field   Meaning
		  0   Message ID $GPGGA
		  1   UTC of position fix
		  2   Latitude
		  3   Direction of latitude:
		  N: North
		  S: South
		  4   Longitude
		  5   Direction of longitude:
		  E: East
		  W: West
		  6   GPS Quality indicator:
		  0: Fix not valid
		  1: GPS fix
		  2: Differential GPS fix, OmniSTAR VBS
		  4: Real-Time Kinematic, fixed integers
		  5: Real-Time Kinematic, float integers, OmniSTAR XP/HP or Location RTK
		  7   Number of SVs in use, range from 00 through to 24+
		  8   HDOP
		  9   Orthometric height (MSL reference)
		  10  M: unit of measure for orthometric height is meters
		  11  Geoid separation
		  12  M: geoid separation measured in meters
		  13  Age of differential GPS data record, Type 1 or Type 9. Null field when DGPS is not used.
		  14  Reference station ID, range 0000-4095. A null field when any reference station ID is selected and no corrections are received1.
		  15
		  The checksum data, always begins with *
		  Note - If a user-defined geoid model, or an inclined
		*/
		double nmea_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		int num_of_sv = 0, fix_quality = 0, dgps_time = 0, dgps_baseid = 0;
		double hdop = 99.9;
		double height_of_Geoid = 0.0;
		char ns = '?', ew = '?';

		nmea_time   = read_float();
		lat         = read_float();
		ns          = read_char();
		lon         = read_float();
		ew          = read_char();
		fix_quality = read_int();
		num_of_sv   = read_int();
		hdop        = read_float();

		if (_parse_error) {
			return 0;
		}

		if (hdop == 0.0) {
			hdop = 99.9;
		}

		alt         = read_float();

		if (ns == 'S')
			lat = -lat;

		if (ew == 'W')
			lon = -lon;

		_gps_position->lat = (int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position->lon = (int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000;
		_gps_position->alt = alt * 1000;
		_gps_position->fix_quality = fix_quality;

		_rate_count_lat_lon++;

		_gps_position->eph_m = hdop;
		_gps_position->timestamp_position = hrt_absolute_time();

		return 1;

	} else if ((memcmp(_rx_buffer + 3, "GSA,", 3) == 0) && (commas_count == 17)) {
		/*
		  GSA - GPS DOP and active satellites. This sentence provides details on the nature of the fix. It includes the numbers of the satellites
		  being used in the current solution and the DOP. DOP (dilution of precision) is an indication of the effect of satellite geometry
		  on the accuracy of the fix. It is a unitless number where smaller is better. For 3D fixes using 4 satellites a 1.0 would be considered
		  to be a perfect number, however for overdetermined solutions it is possible to see numbers below 1.0. There are differences in the way
		  the PRN's are presented which can effect the ability of some programs to display this data. For example,
		  in the example shown below there are 5 satellites in the solution and the null fields are scattered indicating that the almanac
		  would show satellites in the null positions that are not being used as part of this solution. Other receivers might output
		  all of the satellites used at the beginning of the sentence with the null field all stacked up at the end. This difference accounts
		  for some satellite display programs not always being able to display the satellites being tracked. Some units may show all satellites
		  that have ephemeris data without regard to their use as part of the solution but this is non-standard.

  	  	  $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

			Where:
				 GSA      Satellite status
				 A        Auto selection of 2D or 3D fix (M = manual)
				 3        3D fix - values include: 1 = no fix
												   2 = 2D fix
												   3 = 3D fix
				 04,05... PRNs of satellites used for fix (space for 12)
				 2.5      PDOP (dilution of precision)
				 1.3      Horizontal dilution of precision (HDOP)
				 2.1      Vertical dilution of precision (VDOP)
				 *39      the checksum data, always begins with *
		*/
		char auto_section = '?';
		int fix_type = 0;
		double pdop = 0.0, hdop = 99.9, vdop = 99.9;

		auto_section = read_char();
		fix_type = read_int();
		// skip prn information
		int i;
		for(i = 0; i < 12; i++) {
			read_float();
		}

		hdop        = read_float();
		vdop        = read_float();

		if (_parse_error) {
			return 0;
		}

		if (hdop == 0.0) {
			hdop = 99.9;
		}
		if (vdop == 0.0) {
			vdop = 99.9;
		}

		_gps_position->fix_type = fix_type;
		_gps_position->eph_m = hdop;
		_gps_position->epv_m = vdop;

		return 1;

	} else if ((memcmp(_rx_buffer + 3, "GST,", 3) == 0) && (commas_count == 8)) {
		/*
		  Position error statistics
		  An example of the GST message string is:

		  $GPGST,172814.0,0.006,0.023,0.020,273.6,0.023,0.020,0.031*6A

		  The Talker ID ($--) will vary depending on the satellite system used for the position solution:

		  $GP - GPS only
		  $GL - GLONASS only
		  $GN - Combined
		  GST message fields
		  Field   Meaning
		  0   Message ID $GPGST
		  1   UTC of position fix
		  2   RMS value of the pseudorange residuals; includes carrier phase residuals during periods of RTK (float) and RTK (fixed) processing
		  3   Error ellipse semi-major axis 1 sigma error, in meters
		  4   Error ellipse semi-minor axis 1 sigma error, in meters
		  5   Error ellipse orientation, degrees from true north
		  6   Latitude 1 sigma error, in meters
		  7   Longitude 1 sigma error, in meters
		  8   Height 1 sigma error, in meters
		  9   The checksum data, always begins with *
		*/
		double nmea_time = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;
		double min_err = 0.0, maj_err = 0.0, deg_from_north = 0.0, rms_err = 0.0;

		nmea_time = read_float();
		rms_err = read_float();
		maj_err = read_float();
		min_err = read_float();
		deg_from_north = read_float();
		lat_err = read_float();
		lon_err = read_float();
		alt_err = read_float();

		if (_parse_error) {
			return 0;
		}

		_gps_position->s_variance_m_s = 0; //
		_gps_position->p_variance_m = sqrt(lat_err * lat_err + lon_err * lon_err + alt_err * alt_err);
		_gps_position->timestamp_variance = hrt_absolute_time();

	} else if ((memcmp(_rx_buffer + 3, "VTG,", 3) == 0) && (commas_count == 9)) {
		/*
		  Track made good and speed over ground
		  An example of the VTG message string is:

		  $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

		  VTG message fields
		  Field   Meaning
		  0   Message ID $GPVTG
		  1   Track made good (degrees true)
		  2   T: track made good is relative to true north
		  3   Track made good (degrees magnetic)
		  4   M: track made good is relative to magnetic north
		  5   Speed, in knots
		  6   N: speed is measured in knots
		  7   Speed over ground in kilometers/hour (kph)
		  8   K: speed over ground is measured in kph
		  9   The checksum data, always begins with *
		*/
		double track_true = 0.0, track_magnetic = 0.0, speed = 0.0, ground_speed = 0.0;
		char true_north = '?', magnetic_north = '?', speed_in_knots = '?', speed_in_kph  = '?';

		track_true = read_float();
		true_north = read_char();
		track_magnetic = read_float();
		magnetic_north = read_char();
		speed = read_float();
		speed_in_knots = read_char();
		ground_speed = read_float();
		speed_in_kph = read_char();

		if (_parse_error) {
			return 0;
		}

		double track_rad = track_true * M_PI / 180.0;

		double velocity_ms = ground_speed  / 3.6;
		double velocity_north = velocity_ms * cos(track_rad);
		double velocity_east  = velocity_ms * sin(track_rad);

		_gps_position->vel_m_s = velocity_ms;                                  /**< GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = velocity_north;                                /**< GPS ground speed in m/s */
		_gps_position->vel_e_m_s = velocity_east;                                /**< GPS ground speed in m/s */
		_gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position->cog_rad = track_rad;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = true;                             /**< Flag to indicate if NED speed is valid */
		_gps_position->c_variance_rad = 0.1;
		_gps_position->timestamp_velocity = hrt_absolute_time();

		return 1;

	} else if ((memcmp(_rx_buffer + 3, "GSV,", 3) == 0)) {
		/*
		  The GSV message string identifies the number of SVs in view, the PRN numbers, elevations, azimuths, and SNR values. An example of the GSV message string is:

		  $GPGSV,4,1,13,02,02,213,,03,-3,000,,11,00,121,,14,13,172,05*67

		  GSV message fields
		  Field   Meaning
		  0   Message ID $GPGSV
		  1   Total number of messages of this type in this cycle
		  2   Message number
		  3   Total number of SVs visible
		  4   SV PRN number
		  5   Elevation, in degrees, 90° maximum
		  6   Azimuth, degrees from True North, 000° through 359°
		  7   SNR, 00 through 99 dB (null when not tracking)
		  8-11    Information about second SV, same format as fields 4 through 7
		  12-15   Information about third SV, same format as fields 4 through 7
		  16-19   Information about fourth SV, same format as fields 4 through 7
		  20  The checksum data, always begins with *
		*/
// currently process only gps, because do not know what
// Global satellite ID I should use for non GPS sats

		bool first_part = (memcmp(_rx_buffer, "$GP", 3) == 0);
        bool last_part = (memcmp(_rx_buffer, "$GL", 3) == 0);

		int max_msg_num, this_msg_num, tot_sv_visible;
		struct gsv_sat {
			int prn;
			int elevation;
			int azimuth;
			int snr;
		} sat[4];
		memset(sat, 0, sizeof(sat));

		max_msg_num = read_int();
		this_msg_num = read_int();
		tot_sv_visible = read_int();

		if (_parse_error) {
			return 0;
		}

		if ((this_msg_num < 1) || (this_msg_num > max_msg_num)) {
			return 0;
		}

		if(first_part && (this_msg_num == 1)) {
			// new satellites data
			_satellites_count = 0;
			_satellites_visible = 0;
			_gsv_in_progress = true;
		} else if(!_gsv_in_progress) {
			return 0;
		}

		int sat_in_msg =  this_msg_num == max_msg_num ? tot_sv_visible - (this_msg_num - 1) * 4 : 4;

		for (int i = 0 ; i < sat_in_msg ; i++) {
			sat[i].prn = read_int();
			sat[i].elevation = read_int();
			sat[i].azimuth = read_int();
			sat[i].snr = read_int();

			if (_parse_error) {
				return 0;
			}

			if(sat[i].snr > 0) {
				_satellites_visible++;
			}
		}

		int sat_to_publish = 20 - _satellites_count;
		if(sat_in_msg < sat_to_publish) {
			sat_to_publish = sat_in_msg;
		}
		for (int i = 0 ; i < sat_to_publish; i++) {
			_satellite_prn[_satellites_count + i] = sat[i].prn;
			_satellite_elevation[_satellites_count + i] = sat[i].elevation;
			_satellite_snr[_satellites_count + i] = sat[i].snr;
			_satellite_azimuth[_satellites_count + i] = sat[i].azimuth;
		}

		_satellites_count += sat_to_publish;

		if(last_part && (this_msg_num == max_msg_num)) {
			// satellites data read finished, lets publish it

			for (int i = 0 ; i < _satellites_count ; i++) {
				_gps_position->satellite_prn[i]       = _satellite_prn[i];
				_gps_position->satellite_used[i]      = ((_satellite_snr[i] > 0) ? true : false);
				_gps_position->satellite_snr[i]       = _satellite_snr[i];
				_gps_position->satellite_elevation[i] = _satellite_elevation[i];
				_gps_position->satellite_azimuth[i]   = _satellite_azimuth[i];
			}

			for (int i = _satellites_count; i < 20; i++) {
				/* unused channels have to be set to zero for e.g. MAVLink */
				_gps_position->satellite_prn[i] = 0;
				_gps_position->satellite_used[i] = 0;
				_gps_position->satellite_snr[i] = 0;
				_gps_position->satellite_elevation[i] = 0;
				_gps_position->satellite_azimuth[i] = 0;
			}

			_gps_position->satellite_info_available = true;
			_gps_position->satellites_visible = _satellites_visible;
			_gps_position->timestamp_satellites = hrt_absolute_time();
			_gsv_in_progress = false;
		}

	} else if ((memcmp(_rx_buffer + 3, "ZDA,", 3) == 0) && (commas_count == 6)) {
		/*
		UTC day, month, and year, and local time zone offset
		An example of the ZDA message string is:

		$GPZDA,172809.456,12,07,1996,00,00*45

		ZDA message fields
		Field	Meaning
		0	Message ID $GPZDA
		1	UTC
		2	Day, ranging between 01 and 31
		3	Month, ranging between 01 and 12
		4	Year
		5	Local time zone offset from GMT, ranging from 00 through ±13 hours
		6	Local time zone offset from GMT, ranging from 00 through 59 minutes
		7	The checksum data, always begins with *
		Fields 5 and 6 together yield the total offset. For example, if field 5 is -5 and field 6 is +15, local time is 5 hours and 15 minutes earlier than GMT.
		*/
		double nmea_time = 0.0;
		int32_t year = 0, month = 0, day = 0, local_time_off_hour = 0, local_time_off_min = 0;
		nmea_time = read_float();
		day = read_int();
		month = read_int();
		year = read_int();
		local_time_off_hour = read_int();
		local_time_off_min = read_int();

		if (_parse_error) {
			return 0;
		}

		if (year > 0) {
			int32_t nmea_hour = nmea_time / 10000;
			int32_t nmea_minute = (nmea_time - nmea_hour * 10000) / 100;
			double nmea_sec = nmea_time - nmea_hour * 10000 - nmea_minute * 100;
			//convert to unix timestamp
			struct tm timeinfo;
			timeinfo.tm_year = year - 1900;
			timeinfo.tm_mon = month - 1;
			timeinfo.tm_mday = day;
			timeinfo.tm_hour = nmea_hour;
			timeinfo.tm_min = nmea_minute;
			timeinfo.tm_sec = int(nmea_sec);
			time_t epoch = mktime(&timeinfo);

			_gps_position->time_gps_usec = (uint64_t)epoch * 1000000;
			uint64_t delta = (uint64_t)((nmea_sec - int(nmea_sec)) * 1e6);
			_gps_position->time_gps_usec += (uint64_t)((nmea_sec - int(nmea_sec)) * 1e6);
			_gps_position->timestamp_time = hrt_absolute_time();

			return 1;

		} else {
			// Time is not valid. GPS isn't fixed?
		}
	}

	return 0;
}

int NMEA::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[32];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	int j = 0;
	ssize_t count = 0;

	while (true) {

		/* pass received bytes to the packet decoder */
		while (j < count) {
			int l = 0;

			if ((l = parse_char(buf[j])) > 0) {
				/* return to configure during configuration or to the gps driver during normal work
				 * if a packet has arrived */
				if (handle_message(l) >= 0)
					return 1;
			}

			/* in case we keep trying but only get crap from GPS */
			if (time_started + timeout * 1000 * 2 < hrt_absolute_time()) {
				return -1;
			}

			j++;
		}

		/* everything is read */
		j = count = 0;

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout * 2);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_fd, buf, sizeof(buf));
			}
		}
	}
}

#define HEXDIGIT_CHAR(d) ((char)((d) + (((d) < 0xA) ? '0' : 'A'-0xA)))

int NMEA::parse_char(uint8_t b)
{
	int iRet = 0;

	switch (_decode_state) {
		/* First, look for sync1 */
	case NME_DECODE_UNINIT:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;
			_rx_buffer[_rx_buffer_bytes++] = b;
		}

		break;

	case NME_DECODE_GOT_SYNC1:
		if (b == '$') {
			_decode_state = NME_DECODE_GOT_SYNC1;
			_rx_buffer_bytes = 0;

		} else if (b == '*') {
			_decode_state = NME_DECODE_GOT_ASTERIKS;
		}

		if (_rx_buffer_bytes >= (sizeof(_rx_buffer) - 5)) {
			_decode_state = NME_DECODE_UNINIT;
			_rx_buffer_bytes = 0;

		} else
			_rx_buffer[_rx_buffer_bytes++] = b;

		break;

	case NME_DECODE_GOT_ASTERIKS:
		_rx_buffer[_rx_buffer_bytes++] = b;
		_decode_state = NME_DECODE_GOT_FIRST_CS_BYTE;
		break;

	case NME_DECODE_GOT_FIRST_CS_BYTE:
		_rx_buffer[_rx_buffer_bytes++] = b;
		uint8_t checksum = 0;
		uint8_t *buffer = _rx_buffer + 1;
		uint8_t *bufend = _rx_buffer + _rx_buffer_bytes - 3;

		for (; buffer < bufend; buffer++) checksum ^= *buffer;

		if ((HEXDIGIT_CHAR(checksum >> 4) == *(_rx_buffer + _rx_buffer_bytes - 2)) &&
		    (HEXDIGIT_CHAR(checksum & 0x0F) == *(_rx_buffer + _rx_buffer_bytes - 1))) {
			iRet = _rx_buffer_bytes;
		}

		_decode_state = NME_DECODE_UNINIT;
		_rx_buffer_bytes = 0;
		break;
	}

	return iRet;
}

void NMEA::decode_init(void)
{

}

int NMEA::configure(unsigned &baudrate)
{
	baudrate = 38400;
	set_baudrate(_fd, baudrate);

	return 0;
}
