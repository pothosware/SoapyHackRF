/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015
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

#pragma once
#include <libhackrf/hackrf.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>

#define BUF_LEN			262144
#define BUF_NUM			15
#define BYTES_PER_SAMPLE	2

enum HackRF_Format {
	HACKRF_FORMAT_FLOAT32	=0,
	HACKRF_FORMAT_INT16	=1,
	HACKRF_FORMAT_INT8	=2,
};


class SoapyHackRF : public SoapySDR::Device
{
public:
	SoapyHackRF( const SoapySDR::Kwargs & args );

	~SoapyHackRF( void );


	/*******************************************************************
	 * Identification API
	 ******************************************************************/

	std::string getDriverKey( void ) const;


	std::string getHardwareKey( void ) const;


	SoapySDR::Kwargs getHardwareInfo( void ) const;


	/*******************************************************************
	 * Channels API
	 ******************************************************************/

	size_t getNumChannels( const int ) const;


	bool getFullDuplex( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Stream API
	 ******************************************************************/

	SoapySDR::Stream *setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


	void closeStream( SoapySDR::Stream *stream );


	size_t getStreamMTU( SoapySDR::Stream *stream ) const;


	int activateStream(
		SoapySDR::Stream *stream,
		const int flags = 0,
		const long long timeNs = 0,
		const size_t numElems = 0 );


	int deactivateStream(
		SoapySDR::Stream *stream,
		const int flags = 0,
		const long long timeNs = 0 );


	int readStream(
		SoapySDR::Stream *stream,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs = 100000 );


	int writeStream(
		SoapySDR::Stream *stream,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs = 100000 );

	int readStreamStatus(
			SoapySDR::Stream *stream,
			size_t &chanMask,
			int &flags,
			long long &timeNs,
			const long timeoutUs
	);



	/*******************************************************************
	 * Antenna API
	 ******************************************************************/

	std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;


	void setAntenna( const int direction, const size_t channel, const std::string &name );


	std::string getAntenna( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Frontend corrections API
	 ******************************************************************/

	bool hasDCOffsetMode( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Gain API
	 ******************************************************************/

	std::vector<std::string> listGains( const int direction, const size_t channel ) const;


	void setGainMode( const int direction, const size_t channel, const bool automatic );


	bool getGainMode( const int direction, const size_t channel ) const;


	void setGain( const int direction, const size_t channel, const double value );


	void setGain( const int direction, const size_t channel, const std::string &name, const double value );


	double getGain( const int direction, const size_t channel, const std::string &name ) const;


	SoapySDR::Range getGainRange( const int direction, const size_t channel, const std::string &name ) const;


	/*******************************************************************
	 * Frequency API
	 ******************************************************************/

	void setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


	double getFrequency( const int direction, const size_t channel, const std::string &name ) const;


	std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;


	SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;


	/*******************************************************************
	 * Sample Rate API
	 ******************************************************************/

	void setSampleRate( const int direction, const size_t channel, const double rate );


	double getSampleRate( const int direction, const size_t channel ) const;


	std::vector<double> listSampleRates( const int direction, const size_t channel ) const;


	void setBandwidth( const int direction, const size_t channel, const double bw );


	double getBandwidth( const int direction, const size_t channel ) const;


	std::vector<double> listBandwidths( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * HackRF callback
	 ******************************************************************/
	int hackrf_tx_callback( int8_t *buffer, int32_t length );


	int hackrf_rx_callback( int8_t *buffer, int32_t length );


private:
	bool _running;

	bool _auto_bandwidth;

	hackrf_device * _dev;

	double _frequency;

	uint32_t _rx_vga_gain;

	uint32_t _rx_lna_gain;

	uint32_t _tx_vga_gain;

	double _samplerate;

	uint32_t _bandwidth;

	uint8_t _amp;

	uint32_t _format;

	int32_t _id;

	int8_t		**_buf;
	uint32_t	_buf_num;
	uint32_t	_buf_len;
	uint32_t	_buf_head;
	uint32_t	_buf_tail;
	uint32_t	_buf_count;
	uint32_t	_buf_offset;

	int32_t _samp_avail;
	bool _overflow;
	bool _underflow;
	std::mutex		_buf_mutex;
	std::condition_variable _buf_cond;


};