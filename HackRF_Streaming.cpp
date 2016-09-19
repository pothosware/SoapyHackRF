/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2016 Wei Jiang
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
#include <SoapySDR/Logger.hpp>
#include <chrono>
#include <thread>
#include <algorithm> //min

int _hackrf_rx_callback( hackrf_transfer *transfer )
{
	SoapyHackRF* obj = (SoapyHackRF *) transfer->rx_ctx;
	return(obj->hackrf_rx_callback( (int8_t *) transfer->buffer, transfer->valid_length ) );
}


int _hackrf_tx_callback( hackrf_transfer *transfer )
{
	SoapyHackRF* obj = (SoapyHackRF *) transfer->tx_ctx;
	return(obj->hackrf_tx_callback( (int8_t *) transfer->buffer, transfer->valid_length ) );
}

int SoapyHackRF::hackrf_rx_callback( int8_t *buffer, int32_t length )
{
	std::unique_lock<std::mutex> lock(_buf_mutex);
	_rx_stream->buf_tail = (_rx_stream->buf_head + _rx_stream->buf_count) % _rx_stream->buf_num;
	memcpy(_rx_stream->buf[_rx_stream->buf_tail], buffer, length );

	if ( _rx_stream->buf_count == _rx_stream->buf_num )
	{
		_rx_stream->overflow=true;
		_rx_stream->buf_head = (_rx_stream->buf_head + 1) % _rx_stream->buf_num;
	}else  {
		_rx_stream->buf_count++;
	}
	_buf_cond.notify_one();

	return(0);
}


int SoapyHackRF::hackrf_tx_callback( int8_t *buffer, int32_t length  )
{
	std::unique_lock<std::mutex> lock(_buf_mutex);
	if ( _tx_stream->buf_count == 0 )
	{
		memset( buffer, 0, length );
		_tx_stream->underflow=true;
	}else {
		memcpy( buffer, _tx_stream->buf[_tx_stream->buf_tail], length );
		_tx_stream->buf_tail = (_tx_stream->buf_tail + 1) % _tx_stream->buf_num;

		_tx_stream->buf_count--;

		if(_tx_stream->burst_end)
		{
			_tx_stream->burst_samps -= (length/BYTES_PER_SAMPLE);
			if(_tx_stream->burst_samps < 0 ) {
				_tx_stream->burst_end = false;
				_tx_stream->burst_samps = 0;
				return -1;
			}
		}
	}
	_buf_cond.notify_one();

	return(0);
}

std::vector<std::string> SoapyHackRF::getStreamFormats(const int direction, const size_t channel) const
{
	std::vector<std::string> formats;

	formats.push_back("CS8");
	formats.push_back("CS16");
	formats.push_back("CF32");
	formats.push_back("CF64");

	return formats;
}

std::string SoapyHackRF::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	fullScale = 128;
	return "CS8";
}

SoapySDR::ArgInfoList SoapyHackRF::getStreamArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList streamArgs;

	SoapySDR::ArgInfo buffersArg;
	buffersArg.key="buffers";
	buffersArg.value = std::to_string(BUF_NUM);
	buffersArg.name = "Buffer Count";
	buffersArg.description = "Number of buffers per read.";
	buffersArg.units = "buffers";
	buffersArg.type = SoapySDR::ArgInfo::INT;
	streamArgs.push_back(buffersArg);

	return streamArgs;
}

SoapySDR::Stream *SoapyHackRF::setupStream(
	const int direction,
	const std::string &format,
	const std::vector<size_t> &channels,
	const SoapySDR::Kwargs &args )
{
	if ( channels.size() > 1 or( channels.size() > 0 and channels.at( 0 ) != 0 ) )
	{
		throw std::runtime_error( "setupStream invalid channel selection" );
	}

	SoapyHackRFStream * data=new SoapyHackRFStream();

	if(direction==SOAPY_SDR_RX){

		if ( format == "CS8" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS8." );
			_rx_stream->format = HACKRF_FORMAT_INT8;
		}else if ( format == "CS16" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS16." );
			_rx_stream->format = HACKRF_FORMAT_INT16;
		}else if ( format == "CF32" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF32." );
			_rx_stream->format= HACKRF_FORMAT_FLOAT32;
		}else if(format=="CF64"){
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF64." );
			_rx_stream->format= HACKRF_FORMAT_FLOAT64;
		}else throw std::runtime_error( "setupStream invalid format " + format );

		if ( args.count( "buffers" ) != 0 )
		{
			try
			{
				int numBuffers_in = std::stoi(args.at("buffers"));
				if (numBuffers_in > 0) {
					if ( _rx_stream->buf ) {
						for ( unsigned int i = 0; i < _rx_stream->buf_num; ++i ) {
							if ( _rx_stream->buf[i] ) {
								free( _rx_stream->buf[i] );
							}
						}
						free( _rx_stream->buf );
						_rx_stream->buf = NULL;
					}

					_rx_stream->buf_num = numBuffers_in;
				}
			}
			catch (const std::invalid_argument &){}

		}
		_rx_stream->buf = (int8_t * *) malloc( _rx_stream->buf_num * sizeof(int8_t *) );
		if ( _rx_stream->buf )
		{
			for ( unsigned int i = 0; i < _rx_stream->buf_num; ++i )
				_rx_stream->buf[i] = (int8_t *) malloc( _rx_stream->buf_len );
		}
	}

	if(direction==SOAPY_SDR_TX){

		if ( format == "CS8" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS8." );
			_tx_stream->format = HACKRF_FORMAT_INT8;
		}else if ( format == "CS16" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS16." );
			_tx_stream->format = HACKRF_FORMAT_INT16;
		}else if ( format == "CF32" )
		{
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF32." );
			_tx_stream->format= HACKRF_FORMAT_FLOAT32;
		}else if(format=="CF64"){
			SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF64." );
			_tx_stream->format= HACKRF_FORMAT_FLOAT64;
		}else throw std::runtime_error( "setupStream invalid format " + format );

		if ( args.count( "buffers" ) != 0 )
		{
			try
			{
				int numBuffers_in = std::stoi(args.at("buffers"));
				if (numBuffers_in > 0)
				{
					if ( _tx_stream->buf )
					{
						for ( unsigned int i = 0; i < _tx_stream->buf_num; ++i )
						{
							if ( _tx_stream->buf[i] )
							{
								free( _tx_stream->buf[i] );
							}
						}
						free( _tx_stream->buf );
						_tx_stream->buf = NULL;
					}

					_tx_stream->buf_num = numBuffers_in;
				}
			}
			catch (const std::invalid_argument &){}

		}

		_tx_stream->buf = (int8_t * *) malloc( _tx_stream->buf_num * sizeof(int8_t *) );
		if ( _tx_stream->buf )
		{
			for ( unsigned int i = 0; i < _tx_stream->buf_num; ++i )
				_tx_stream->buf[i] = (int8_t *) malloc( _tx_stream->buf_len );
		}

	}

	data->direction=direction;


	return ((SoapySDR::Stream *) data );
}


void SoapyHackRF::closeStream( SoapySDR::Stream *stream )
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	delete data;;
}


size_t SoapyHackRF::getStreamMTU( SoapySDR::Stream *stream ) const
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;
	size_t mtu;
	if(data->direction==SOAPY_SDR_RX){
		mtu =_rx_stream->buf_len/BYTES_PER_SAMPLE;

	}
	if(data->direction==SOAPY_SDR_TX){

		mtu=_tx_stream->buf_len/BYTES_PER_SAMPLE;
	}
	return mtu;
}

int SoapyHackRF::activateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs,
	const size_t numElems )
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction==SOAPY_SDR_RX){

		std::lock_guard<std::mutex> lock(_activate_mutex);


		if(_current_mode==TRANSCEIVER_MODE_RX)
			return 0;

		if(_current_mode==TRANSCEIVER_MODE_TX){

			if(_tx_stream->burst_end){

				while(hackrf_is_streaming(_dev)==HACKRF_TRUE)
					std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}

			hackrf_stop_tx(_dev);
		}

		SoapySDR_logf(SOAPY_SDR_DEBUG, "Start RX");

		//reset buffer tracking before streaming
		{
			_rx_stream->buf_count = 0;
			_rx_stream->buf_head = 0;
			_rx_stream->buf_tail = 0;
		}

		int ret = hackrf_start_rx(_dev, _hackrf_rx_callback, (void *) this);
		if (ret != HACKRF_SUCCESS) {
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_start_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		ret=hackrf_is_streaming(_dev);

		if (ret==HACKRF_ERROR_STREAMING_EXIT_CALLED){

			hackrf_close(_dev);

			if (_id<0){
				hackrf_open(&_dev);
			}else {
				hackrf_device_list_open(_list,_id,&_dev);
			}
			_current_frequency=_rx_stream->frequecy;
			hackrf_set_freq(_dev,_current_frequency);
			_current_samplerate=_rx_stream->samplerate;
			hackrf_set_sample_rate(_dev,_current_samplerate);
			_current_bandwidth=_rx_stream->bandwidth;
			hackrf_set_baseband_filter_bandwidth(_dev,_current_bandwidth);
			_current_amp=_rx_stream->amp_gain;
			hackrf_set_amp_enable(_dev,_current_amp);
			hackrf_set_lna_gain(_dev,_rx_stream->lna_gain);
			hackrf_set_vga_gain(_dev,_rx_stream->vga_gain);
			hackrf_start_rx(_dev,_hackrf_rx_callback,(void *) this);
			ret=hackrf_is_streaming(_dev);
		}
		if(ret!=HACKRF_TRUE){
			SoapySDR_logf(SOAPY_SDR_ERROR,"Activate RX Stream Failed.");
			return SOAPY_SDR_STREAM_ERROR;

		}
			_current_mode = TRANSCEIVER_MODE_RX;

	}

	if(data->direction==SOAPY_SDR_TX ){

		std::lock_guard<std::mutex> lock(_activate_mutex);

		if((flags & SOAPY_SDR_END_BURST)!=0 and numElems!=0) {
			if(_current_mode==TRANSCEIVER_MODE_RX){
				_tx_stream->buf_head=0;
				_tx_stream->buf_tail=0;
				_tx_stream->burst_end = true;
				_tx_stream->burst_samps = numElems;
			}
		}

		if(_current_mode==TRANSCEIVER_MODE_TX)
			return 0;

		if(_current_mode==TRANSCEIVER_MODE_RX){

			hackrf_stop_rx(_dev);
		}

		SoapySDR_logf( SOAPY_SDR_DEBUG, "Start TX" );

		int ret = hackrf_start_tx( _dev, _hackrf_tx_callback, (void *) this );
		if (ret != HACKRF_SUCCESS)
		{
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_start_tx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		ret=hackrf_is_streaming(_dev);

		if (ret==HACKRF_ERROR_STREAMING_EXIT_CALLED){


			hackrf_close(_dev);

			if (_id<0){
				hackrf_open(&_dev);
			}else {
				hackrf_device_list_open(_list,_id,&_dev);
			}
			_current_frequency=_tx_stream->frequecy;
			hackrf_set_freq(_dev,_current_frequency);
			_current_samplerate=_tx_stream->samplerate;
			hackrf_set_sample_rate(_dev,_current_samplerate);
			_current_bandwidth=_tx_stream->bandwidth;
			hackrf_set_baseband_filter_bandwidth(_dev,_current_bandwidth);
			_current_amp=_rx_stream->amp_gain;
			hackrf_set_amp_enable(_dev,_current_amp);
			hackrf_set_txvga_gain(_dev,_tx_stream->vga_gain);
			hackrf_set_antenna_enable(_dev,_tx_stream->bias);
			hackrf_start_tx(_dev,_hackrf_tx_callback,(void *) this);
			ret=hackrf_is_streaming(_dev);
		}
		if(ret!=HACKRF_TRUE){

			SoapySDR_logf(SOAPY_SDR_ERROR,"Activate TX Stream Failed.");
			return SOAPY_SDR_STREAM_ERROR;
		}
			_current_mode = TRANSCEIVER_MODE_TX;

	}

	return(0);
}


int SoapyHackRF::deactivateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs )
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction==SOAPY_SDR_RX){

		std::lock_guard<std::mutex> lock(_activate_mutex);

		if(_current_mode==TRANSCEIVER_MODE_RX) {

			int ret = hackrf_stop_rx(_dev);
			if (ret != HACKRF_SUCCESS) {
				SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
			}
			_current_mode = TRANSCEIVER_MODE_OFF;
		}
	}

	if(data->direction==SOAPY_SDR_TX){

		std::lock_guard<std::mutex> lock(_activate_mutex);

		if(_current_mode==TRANSCEIVER_MODE_TX) {
			int ret = hackrf_stop_tx(_dev);
			if (ret != HACKRF_SUCCESS) {
				SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_tx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
			}
			_current_mode = TRANSCEIVER_MODE_OFF;
		}

	}
	return(0);
}

void readbuf(int8_t * src, void * dst, uint32_t len,uint32_t format,size_t offset){

	if(format==HACKRF_FORMAT_INT8){
		int8_t *samples_cs8=(int8_t *) dst+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			samples_cs8[i*BYTES_PER_SAMPLE] = src[i*BYTES_PER_SAMPLE];
			samples_cs8[i*BYTES_PER_SAMPLE+1] = src[i*BYTES_PER_SAMPLE+1];
		}

	}else if(format==HACKRF_FORMAT_INT16){

		int16_t *samples_cs16=(int16_t *) dst+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			samples_cs16[i*BYTES_PER_SAMPLE] = (int16_t)(src[i*BYTES_PER_SAMPLE]<<8);
			samples_cs16[i*BYTES_PER_SAMPLE+1] = (int16_t)(src[i*BYTES_PER_SAMPLE+1]<<8);
		}
	}else if(format==HACKRF_FORMAT_FLOAT32){
		float *samples_cf32=(float *) dst+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			samples_cf32[i*BYTES_PER_SAMPLE] = (float)(src[i*BYTES_PER_SAMPLE]/127.0);
			samples_cf32[i*BYTES_PER_SAMPLE+1] = (float)(src[i*BYTES_PER_SAMPLE+1]/127.0);
		}
	}else if(format==HACKRF_FORMAT_FLOAT64){
		double *samples_cf64=(double *) dst+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			samples_cf64[i*BYTES_PER_SAMPLE] = (double)(src[i*BYTES_PER_SAMPLE]/127.0);
			samples_cf64[i*BYTES_PER_SAMPLE+1] = (double)(src[i*BYTES_PER_SAMPLE+1]/127.0);
		}
	} else {
		SoapySDR_log( SOAPY_SDR_ERROR, "read format not support" );
	}
}


void writebuf(const void * src, int8_t* dst, uint32_t len,uint32_t format,size_t offset) {
	if(format==HACKRF_FORMAT_INT8){
		int8_t *samples_cs8=(int8_t *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = samples_cs8[i*BYTES_PER_SAMPLE];
			dst[i*BYTES_PER_SAMPLE+1] = samples_cs8[i*BYTES_PER_SAMPLE+1];
		}

	}else if(format==HACKRF_FORMAT_INT16){
		int16_t *samples_cs16=(int16_t *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (int8_t) (samples_cs16[i*BYTES_PER_SAMPLE] >> 8);
			dst[i*BYTES_PER_SAMPLE+1] = (int8_t) (samples_cs16[i*BYTES_PER_SAMPLE+1] >> 8);
		}
	}else if(format==HACKRF_FORMAT_FLOAT32){
		float *samples_cf32=(float *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (int8_t) (samples_cf32[i*BYTES_PER_SAMPLE] * 127.0);
			dst[i*BYTES_PER_SAMPLE+1] = (int8_t) (samples_cf32[i*BYTES_PER_SAMPLE+1] * 127.0);
		}
	}else if(format==HACKRF_FORMAT_FLOAT64){
		double *samples_cf64=(double *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (int8_t) (samples_cf64[i*BYTES_PER_SAMPLE] * 127.0);
			dst[i*BYTES_PER_SAMPLE+1] = (int8_t) (samples_cf64[i*BYTES_PER_SAMPLE+1] * 127.0);
		}

	}else {
		SoapySDR_log( SOAPY_SDR_ERROR, "write format not support" );

	}
}


int SoapyHackRF::readStream(
	SoapySDR::Stream *stream,
	void * const *buffs,
	const size_t numElems,
	int &flags,
	long long &timeNs,
	const long timeoutUs )
{
	/* this is the user's buffer for channel 0 */

	size_t returnedElems = std::min(numElems,this->getStreamMTU(stream));

	size_t samp_avail=0;

	if(_rx_stream->remainderHandle >= 0){

		const size_t n =std::min(_rx_stream->remainderSamps,returnedElems);

		if(n<returnedElems){
			samp_avail=n;
		}

		readbuf(_rx_stream->remainderBuff+_rx_stream->remainderOffset*BYTES_PER_SAMPLE,buffs[0],n,_rx_stream->format,0);

		_rx_stream->remainderOffset+=n;
		_rx_stream->remainderSamps -=n;

		if(_rx_stream->remainderSamps==0){

			this->releaseReadBuffer(stream,_rx_stream->remainderHandle);
			_rx_stream->remainderHandle=-1;
			_rx_stream->remainderOffset=0;
		}

		if(n==returnedElems)
			return returnedElems;
	}

	size_t handle;
	int ret = this->acquireReadBuffer(stream, handle, (const void **)&_rx_stream->remainderBuff, flags, timeNs, timeoutUs);

	if (ret < 0)
		return ret;

	_rx_stream->remainderHandle=handle;
	_rx_stream->remainderSamps=ret;


	const size_t n =std::min((returnedElems-samp_avail),_rx_stream->remainderSamps);

	readbuf(_rx_stream->remainderBuff,buffs[0],n,_rx_stream->format,samp_avail);
	_rx_stream->remainderSamps -=n;
	_rx_stream->remainderOffset +=n;

	if(_rx_stream->remainderSamps==0){
		this->releaseReadBuffer(stream,_rx_stream->remainderHandle);
		_rx_stream->remainderHandle=-1;
		_rx_stream->remainderOffset=0;
	}

	return(returnedElems);
}


int SoapyHackRF::writeStream(
		SoapySDR::Stream *stream,
		const void * const *buffs,
		const size_t numElems,
		int &flags,
		const long long timeNs,
		const long timeoutUs )
{

	size_t returnedElems = std::min(numElems,this->getStreamMTU(stream));

	size_t samp_avail = 0;

	if(_tx_stream->remainderHandle>=0){

		const size_t n =std::min(_tx_stream->remainderSamps,returnedElems);

		if(n<returnedElems){
			samp_avail=n;
		}

		writebuf(buffs[0],_tx_stream->remainderBuff+_tx_stream->remainderOffset*BYTES_PER_SAMPLE,n,_tx_stream->format,0);
		_tx_stream->remainderSamps -=n;
		_tx_stream->remainderOffset +=n;

		if(_tx_stream->remainderSamps==0){
			this->releaseWriteBuffer(stream,_tx_stream->remainderHandle,_tx_stream->remainderOffset,flags,timeNs);
			_tx_stream->remainderHandle=-1;
			_tx_stream->remainderOffset=0;
		}

		if(n==returnedElems)
			return returnedElems;

	}

	size_t handle;

	int ret=this->acquireWriteBuffer(stream,handle,(void **)&_tx_stream->remainderBuff,timeoutUs);
	if (ret<0)return ret;

	_tx_stream->remainderHandle=handle;
	_tx_stream->remainderSamps=ret;

	const size_t n =std::min((returnedElems-samp_avail),_tx_stream->remainderSamps);

	writebuf(buffs[0],_tx_stream->remainderBuff,n,_tx_stream->format,samp_avail);
	_tx_stream->remainderSamps -=n;
	_tx_stream->remainderOffset +=n;

	if(_tx_stream->remainderSamps==0){
		this->releaseWriteBuffer(stream,_tx_stream->remainderHandle,_tx_stream->remainderOffset,flags,timeNs);
		_tx_stream->remainderHandle=-1;
		_tx_stream->remainderOffset=0;
	}

	return returnedElems;

}


int SoapyHackRF::readStreamStatus(
		SoapySDR::Stream *stream,
		size_t &chanMask,
		int &flags,
		long long &timeNs,
		const long timeoutUs
){
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction!=SOAPY_SDR_TX){

		return SOAPY_SDR_NOT_SUPPORTED;
	}

	//calculate when the loop should exit
	const auto timeout = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(timeoutUs));
	const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

	//poll for status events until the timeout expires
	while (true)
	{
		if(_tx_stream->underflow){
			_tx_stream->underflow=false;
			SoapySDR::log(SOAPY_SDR_SSI, "U");
			return SOAPY_SDR_UNDERFLOW;
		}

		//sleep for a fraction of the total timeout
		const auto sleepTimeUs = std::min<long>(1000, timeoutUs/10);
		std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeUs));

		//check for timeout expired
		const auto timeNow = std::chrono::high_resolution_clock::now();
		if (exitTime < timeNow) return SOAPY_SDR_TIMEOUT;
	}
}

int SoapyHackRF::acquireReadBuffer(
		SoapySDR::Stream *stream,
		size_t &handle,
		const void **buffs,
		int &flags,
		long long &timeNs,
		const long timeoutUs)
{

	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction!=SOAPY_SDR_RX){
		return SOAPY_SDR_NOT_SUPPORTED;
	}

	if ( _current_mode!=TRANSCEIVER_MODE_RX ) {

		//wait for tx to be consumed before switching
		const auto exitTime = std::chrono::high_resolution_clock::now() + std::chrono::microseconds(timeoutUs);
		while (true)
		{
			std::unique_lock <std::mutex> lock( _buf_mutex );
			if (_tx_stream->buf_count == 0) break;
			if (std::chrono::high_resolution_clock::now() > exitTime) return SOAPY_SDR_TIMEOUT;
		}

		int ret=this->activateStream(stream);
		if(ret<0) return ret;
	}

	std::unique_lock <std::mutex> lock( _buf_mutex );

	while (_rx_stream->buf_count == 0)
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_rx_stream->buf_count == 0) return SOAPY_SDR_TIMEOUT;
	}

	if(_rx_stream->overflow) {
		flags|=SOAPY_SDR_END_ABRUPT;
		_rx_stream->overflow=false;
		SoapySDR::log(SOAPY_SDR_SSI,"O");
		return  SOAPY_SDR_OVERFLOW;
	}

	handle=_rx_stream->buf_head;
	_rx_stream->buf_head = (_rx_stream->buf_head + 1) % _rx_stream->buf_num;
	this->getDirectAccessBufferAddrs(stream,handle,(void **)buffs);

	return this->getStreamMTU(stream);
}

void SoapyHackRF::releaseReadBuffer(
		SoapySDR::Stream *stream,
		const size_t handle)
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(!_tx_stream->burst_end){

		std::unique_lock <std::mutex> lock( _buf_mutex );

		_rx_stream->buf_count--;
	}

}

int SoapyHackRF::acquireWriteBuffer(
		SoapySDR::Stream *stream,
		size_t &handle,
		void **buffs,
		const long timeoutUs)
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction!=SOAPY_SDR_TX){

		return SOAPY_SDR_NOT_SUPPORTED;
	}

	if(_current_mode!=TRANSCEIVER_MODE_TX) {
		int ret=this->activateStream(stream);
		if(ret<0) return ret;
	}

	std::unique_lock <std::mutex> lock( _buf_mutex );

	while ( _tx_stream->buf_count == _tx_stream->buf_num )
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_tx_stream->buf_count == _tx_stream->buf_num) return SOAPY_SDR_TIMEOUT;
	}

	handle=_tx_stream->buf_head;
	_tx_stream->buf_head = (_tx_stream->buf_head + 1) % _tx_stream->buf_num;

	this->getDirectAccessBufferAddrs(stream,handle,buffs);

	if(_tx_stream->burst_end){
		if((_tx_stream->burst_samps - int32_t(this->getStreamMTU(stream))) < 0){
			memset(buffs[0],0,this->getStreamMTU(stream));
			return _tx_stream->burst_samps;
		}
	}
	return this->getStreamMTU(stream);

}

void SoapyHackRF::releaseWriteBuffer(
		SoapySDR::Stream *stream,
		const size_t handle,
		const size_t numElems,
		int &flags,
		const long long timeNs)
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	std::unique_lock <std::mutex> lock( _buf_mutex );

	_tx_stream->buf_count++;
}

size_t SoapyHackRF::getNumDirectAccessBuffers(
		SoapySDR::Stream *stream)
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	size_t buffers;
	if(data->direction==SOAPY_SDR_RX){
		buffers =_rx_stream->buf_num;

	}
	if(data->direction==SOAPY_SDR_TX){

		buffers=_tx_stream->buf_num;
	}

	return buffers;
}

int SoapyHackRF::getDirectAccessBufferAddrs(
		SoapySDR::Stream *stream,
		const size_t handle,
		void **buffs)
{
	SoapyHackRFStream * data = (SoapyHackRFStream*)stream;

	if(data->direction==SOAPY_SDR_RX){
		buffs[0]=(void *)_rx_stream->buf[handle];

	}
	if(data->direction==SOAPY_SDR_TX){

		buffs[0]=(void *)_tx_stream->buf[handle];
	}

	return 0;
}
