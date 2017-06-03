/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Wei Jiang
 * Copyright (c) 2015-2017 Josh Blum
 * Copyright (c) 2017 Kevin Mehall
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
#include <hackrf.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <set>

#define BUF_LEN			262144
#define BUF_NUM			15
#define BYTES_PER_SAMPLE	2
#define HACKRF_RX_VGA_MAX_DB 62
#define HACKRF_TX_VGA_MAX_DB 47
#define HACKRF_RX_LNA_MAX_DB 40
#define HACKRF_AMP_MAX_DB 14

enum HackRF_Format {
	HACKRF_FORMAT_FLOAT32	=0,
	HACKRF_FORMAT_INT16	=1,
	HACKRF_FORMAT_INT8	=2,
	HACKRF_FORMAT_FLOAT64 =3,
};

typedef enum {
	HACKRF_TRANSCEIVER_MODE_OFF = 0,
	HACKRF_TRANSCEIVER_MODE_RX = 1,
	HACKRF_TRANSCEIVER_MODE_TX = 2,
} HackRF_transceiver_mode_t;

std::set<std::string> &HackRF_getClaimedSerials(void);

/*!
 * The session object manages hackrf_init/exit
 * with a process-wide reference count.
 */
class SoapyHackRFSession
{
public:
	SoapyHackRFSession(void);
	~SoapyHackRFSession(void);
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

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

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
			const void * const *buffs,
			const size_t numElems,
			int &flags,
			const long long timeNs = 0,
			const long timeoutUs = 100000);

	int readStreamStatus(
			SoapySDR::Stream *stream,
			size_t &chanMask,
			int &flags,
			long long &timeNs,
			const long timeoutUs
	);


	int acquireReadBuffer(
			SoapySDR::Stream *stream,
			size_t &handle,
			const void **buffs,
			int &flags,
			long long &timeNs,
			const long timeoutUs = 100000);

	void releaseReadBuffer(
			SoapySDR::Stream *stream,
			const size_t handle);

	int acquireWriteBuffer(
			SoapySDR::Stream *stream,
			size_t &handle,
			void **buffs,
			const long timeoutUs = 100000);

	void releaseWriteBuffer(
			SoapySDR::Stream *stream,
			const size_t handle,
			const size_t numElems,
			int &flags,
			const long long timeNs = 0);

	size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

	int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

	/*******************************************************************
	 * Settings API
	 ******************************************************************/

	SoapySDR::ArgInfoList getSettingInfo(void) const;


	void writeSetting(const std::string &key, const std::string &value);


	std::string readSetting(const std::string &key) const;


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


	SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;


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

	SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x1;
	SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x2;

	struct Stream {
		Stream(): opened(false), buf_num(BUF_NUM), buf_len(BUF_LEN), buf(nullptr),
				  buf_head(0), buf_tail(0), buf_count(0),
				  remainderHandle(-1), remainderSamps(0), remainderOffset(0), remainderBuff(nullptr),
				  format(HACKRF_FORMAT_INT8) {}

		bool opened;
		uint32_t	buf_num;
		uint32_t	buf_len;
		int8_t		**buf;
		uint32_t	buf_head;
		uint32_t	buf_tail;
		uint32_t	buf_count;

		int32_t remainderHandle;
		size_t remainderSamps;
		size_t remainderOffset;
		int8_t* remainderBuff;
		uint32_t format;

		~Stream() { clear_buffers(); }
		void clear_buffers();
		void allocate_buffers();
	};

	struct RXStream: Stream {
		uint32_t vga_gain;
		uint32_t lna_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;

		bool overflow;
	};

	struct TXStream: Stream {
		uint32_t vga_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;
		bool bias;

		bool underflow;

		bool burst_end;
		int32_t burst_samps;
	} ;

	RXStream _rx_stream;
	TXStream _tx_stream;

	bool _auto_bandwidth;

	hackrf_device * _dev;
	std::string _serial;

	uint64_t _current_frequency;

	double _current_samplerate;

	uint32_t _current_bandwidth;

	uint8_t _current_amp;

	/// Mutex protecting all use of the hackrf device _dev and other instance variables.
	/// Most of the hackrf API is thread-safe because it only calls libusb, however
	/// the activateStream() method in this library can close and re-open the device,
	/// so all use of _dev must be protected
	mutable std::mutex	_device_mutex;
	std::mutex	_buf_mutex;
	std::condition_variable _buf_cond;

	HackRF_transceiver_mode_t _current_mode;

	SoapyHackRFSession _sess;
};
