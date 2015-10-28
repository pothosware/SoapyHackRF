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

	_buf_tail = (_buf_head + _buf_count) % _buf_num;
	memcpy( _buf[_buf_tail], buffer, length );
	if ( _buf_count == _buf_num )
	{
		_overflow=true;
		_buf_head = (_buf_head + 1) % _buf_num;
	}else  {
		_buf_count++;
	}
	_buf_cond.notify_one();


	return(0);
}

int SoapyHackRF::hackrf_tx_callback( int8_t *buffer, int32_t length )
{
	std::unique_lock<std::mutex> lock(_buf_mutex);
	if ( _buf_count == 0 )
	{
		memset( buffer, 0, length );
		_underflow=true;
	}else{
		memcpy( buffer, _buf[_buf_tail], length );
		_buf_tail = (_buf_tail + 1) % _buf_num;
		_buf_count--;
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
	buffersArg.value = std::to_string(_buf_num);
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

	if ( format == "CS8" )
	{
		SoapySDR_log( SOAPY_SDR_INFO, "Using format CS8." );
		_format = HACKRF_FORMAT_INT8;
	}else if ( format == "CS16" )
	{
		SoapySDR_log( SOAPY_SDR_INFO, "Using format CS16." );
		_format = HACKRF_FORMAT_INT16;
	}else if ( format == "CF32" )
	{
		SoapySDR_log( SOAPY_SDR_INFO, "Using format CF32." );
		_format = HACKRF_FORMAT_FLOAT32;
	}else throw std::runtime_error( "setupStream invalid format " + format );


	if ( _running )
	{
		std::runtime_error( "setupStream invalid format " );
	}


	if ( args.count( "buffers" ) != 0 )
	{
		try
		{
			int numBuffers_in = std::stoi(args.at("buffers"));
			if (numBuffers_in > 0)
			{
				_buf_num = numBuffers_in;
			}
		}
		catch (const std::invalid_argument &){}

	}

	_buf_tail = _buf_count = _buf_head = _remainderSamps =_remainderOffset=0;

	_remainderBuff= nullptr;

	_remainderHandle=-1;

	_buf = (int8_t * *) malloc( _buf_num * sizeof(int8_t *) );
	if ( _buf )
	{
		for ( unsigned int i = 0; i < _buf_num; ++i )
			_buf[i] = (int8_t *) malloc( _buf_len );
	}

	if ( direction == SOAPY_SDR_RX )
	{
		SoapySDR_logf( SOAPY_SDR_INFO, "Start RX" );

		int ret = hackrf_start_rx( _dev, _hackrf_rx_callback, (void *) this );
		if (ret != HACKRF_SUCCESS)
		{
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_start_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		_running = (hackrf_is_streaming( _dev ) == HACKRF_TRUE);

	}
	if ( direction == SOAPY_SDR_TX )
	{

		SoapySDR_logf( SOAPY_SDR_INFO, "Start TX" );

		int ret = hackrf_start_tx( _dev, _hackrf_tx_callback, (void *) this );
		if (ret != HACKRF_SUCCESS)
		{
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_start_tx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		_running = (hackrf_is_streaming( _dev ) == HACKRF_TRUE);

	}


	return( (SoapySDR::Stream *) (new int(direction) ) );
}


void SoapyHackRF::closeStream( SoapySDR::Stream *stream )
{
	const int direction = *reinterpret_cast<int *>(stream);

	if ( direction == SOAPY_SDR_RX )
	{
		int ret = hackrf_stop_rx( _dev );
		if (ret != HACKRF_SUCCESS)
		{
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		_running=false;
	}

	if ( direction == SOAPY_SDR_TX )
	{
		int ret = hackrf_stop_tx( _dev );
		if (ret != HACKRF_SUCCESS)
		{
			SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_tx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
		}

		_running=false;
	}

	if ( _buf )
	{
		for ( unsigned int i = 0; i < _buf_num; ++i )
		{
			if ( _buf[i] )
			{
				free( _buf[i] );
			}
		}
		free( _buf );
		_buf = NULL;
	}

	delete reinterpret_cast<int *>(stream);
}


size_t SoapyHackRF::getStreamMTU( SoapySDR::Stream *stream ) const
{

	return(_buf_len / BYTES_PER_SAMPLE);
}


int SoapyHackRF::activateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs,
	const size_t numElems )
{
	if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
	return(0);
}


int SoapyHackRF::deactivateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs )
{
	if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
	//const int direction = *reinterpret_cast<int *>(stream);


	/*
	 * same idea as activateStream,
	 * but disable streaming in the hardware
	 */
	return(0);
}

void readbuf(int8_t * src, void * dst, uint32_t len,uint32_t format,uint32_t offset){

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
	}else {
		SoapySDR_log( SOAPY_SDR_ERROR, "read format not support" );
	}
}


void writebuf(void * src, int8_t* dst, uint32_t len,uint32_t format,uint32_t offset) {
	if(format==HACKRF_FORMAT_INT8){
		int8_t *samples_cs8=(int8_t *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = samples_cs8[i*BYTES_PER_SAMPLE];
			dst[i*BYTES_PER_SAMPLE+1] = samples_cs8[i*BYTES_PER_SAMPLE+1];

		}

	}else if(format==HACKRF_FORMAT_INT16){

		int16_t *samples_cs16=(int16_t *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (int16_t) (samples_cs16[i*BYTES_PER_SAMPLE] >> 8);
			dst[i*BYTES_PER_SAMPLE+1] = (int16_t) (samples_cs16[i*BYTES_PER_SAMPLE+1] >> 8);
		}
	}else if(format==HACKRF_FORMAT_FLOAT32){
		float *samples_cf32=(float *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (float) (samples_cf32[i*BYTES_PER_SAMPLE] * 127.0);
			dst[i*BYTES_PER_SAMPLE+1] = (float) (samples_cf32[i*BYTES_PER_SAMPLE+1] * 127.0);
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

	if(_remainderHandle >= 0){

		const size_t n =std::min(_remainderSamps,returnedElems);

		if(n<returnedElems){
			samp_avail=n;
		}

		readbuf(_remainderBuff+_remainderOffset*BYTES_PER_SAMPLE,buffs[0],n,_format,0);

		_remainderOffset+=n;
		_remainderSamps -=n;

		if(_remainderSamps==0){

			this->releaseReadBuffer(stream,_remainderHandle);
			_remainderHandle=-1;
			_remainderOffset=0;
		}

		if(n==returnedElems)
			return returnedElems;
	}

	size_t handle;
	int ret = this->acquireReadBuffer(stream, handle, (const void **)&_remainderBuff, flags, timeNs, timeoutUs);

	if (ret < 0)
		return ret;

	_remainderHandle=handle;
	_remainderSamps=ret;

	const size_t n =std::min((returnedElems-samp_avail),_remainderSamps);

	readbuf(_remainderBuff,buffs[0],n,_format,samp_avail);
	_remainderSamps -=n;
	_remainderOffset +=n;

	if(_remainderSamps==0){
		this->releaseReadBuffer(stream,_remainderHandle);
		_remainderHandle=-1;
		_remainderOffset=0;
	}

	return(returnedElems);
}


int SoapyHackRF::writeStream(
	SoapySDR::Stream *stream,
	void * const *buffs,
	const size_t numElems,
	int &flags,
	long long &timeNs,
	const long timeoutUs )
{

	size_t returnedElems = std::min(numElems,this->getStreamMTU(stream));

	size_t samp_avail = 0;

	if(_remainderHandle>=0){

		const size_t n =std::min(_remainderSamps,returnedElems);

		if(n<returnedElems){
			samp_avail=n;
		}

		writebuf(buffs[0],_remainderBuff+_remainderOffset*BYTES_PER_SAMPLE,n,_format,0);
		_remainderSamps -=n;
		_remainderOffset +=n;

		if(_remainderSamps==0){
			this->releaseWriteBuffer(stream,_remainderHandle,_remainderOffset,flags,timeNs);
			_remainderHandle=-1;
			_remainderOffset=0;
		}

		if(n==returnedElems)
			return returnedElems;

	}

	size_t handle;

	int ret=this->acquireWriteBuffer(stream,handle,(void **)&_remainderBuff,timeoutUs);
	if (ret<0)return ret;

	_remainderHandle=handle;
	_remainderSamps=ret;

	const size_t n =std::min((returnedElems-samp_avail),_remainderSamps);

	writebuf(buffs[0],_remainderBuff,n,_format,samp_avail);
	_remainderSamps -=n;
	_remainderOffset +=n;


	if(_remainderSamps==0){
		this->releaseWriteBuffer(stream,_remainderHandle,_remainderOffset,flags,timeNs);
		_remainderHandle=-1;
		_remainderOffset=0;
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
	const int direction = *reinterpret_cast<int *>(stream);

	if (direction == SOAPY_SDR_RX) return SOAPY_SDR_NOT_SUPPORTED;

	//calculate when the loop should exit
	const auto timeout = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(timeoutUs));
	const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

	//poll for status events until the timeout expires
	while (true)
	{
		if(_underflow){
			_underflow=false;
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
	std::unique_lock <std::mutex> lock( _buf_mutex );

	flags=0;

	if ( !_running )
		return(SOAPY_SDR_STREAM_ERROR);

	while (_buf_count == 0)
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_buf_count == 0) return SOAPY_SDR_TIMEOUT;
	}

	if(_overflow) {
		flags|=SOAPY_SDR_END_ABRUPT;
		_overflow=false;
		SoapySDR::log(SOAPY_SDR_SSI,"O");
		return  SOAPY_SDR_OVERFLOW;
	}

	handle=_buf_head;
	_buf_head = (_buf_head + 1) % _buf_num;
	this->getDirectAccessBufferAddrs(stream,handle,(void **)buffs);

	return this->getStreamMTU(stream);
}

void SoapyHackRF::releaseReadBuffer(
		SoapySDR::Stream *stream,
		const size_t handle)
{
	std::unique_lock <std::mutex> lock( _buf_mutex );

	_buf_count--;
}

int SoapyHackRF::acquireWriteBuffer(
		SoapySDR::Stream *stream,
		size_t &handle,
		void **buffs,
		const long timeoutUs)
{
	std::unique_lock <std::mutex> lock( _buf_mutex );

	if(!_running)
		return SOAPY_SDR_STREAM_ERROR;

	while ( _buf_count == _buf_num )
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_buf_count == _buf_num) return SOAPY_SDR_TIMEOUT;
	}

	handle=_buf_head;
	_buf_head = (_buf_head + 1) % _buf_num;

	this->getDirectAccessBufferAddrs(stream,handle,buffs);

	return this->getStreamMTU(stream);

}

void SoapyHackRF::releaseWriteBuffer(
		SoapySDR::Stream *stream,
		const size_t handle,
		const size_t numElems,
		int &flags,
		const long long timeNs)
{
	std::unique_lock <std::mutex> lock( _buf_mutex );

	if (numElems<this->getStreamMTU(stream))flags &=~SOAPY_SDR_END_BURST;

	_buf_count++;
}

size_t SoapyHackRF::getNumDirectAccessBuffers(
		SoapySDR::Stream *stream)
{

	return(_buf_num);

}

int SoapyHackRF::getDirectAccessBufferAddrs(
		SoapySDR::Stream *stream,
		const size_t handle,
		void **buffs)
{
	buffs[0]=(void *)_buf[handle];
	return 0;
}
