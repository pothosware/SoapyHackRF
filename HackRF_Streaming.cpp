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

	_buf_tail = _buf_count = _buf_head = _buf_offset = 0;

	_samp_avail = _buf_len / BYTES_PER_SAMPLE;

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

		_running = (hackrf_is_streaming( _dev ) == HACKRF_TRUE);

	}
	if ( direction == SOAPY_SDR_TX )
	{

		SoapySDR_logf( SOAPY_SDR_INFO, "Start TX" );

		int ret = hackrf_start_tx( _dev, _hackrf_tx_callback, (void *) this );

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

		_running=false;
	}

	if ( direction == SOAPY_SDR_TX )
	{
		int ret = hackrf_stop_tx( _dev );

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
	return(0);
}


int SoapyHackRF::deactivateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs )
{
	const int direction = *reinterpret_cast<int *>(stream);


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
		float_t *samples_cf32=(float_t *) dst+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			samples_cf32[i*BYTES_PER_SAMPLE] = (float_t)(src[i*BYTES_PER_SAMPLE]/127.0);
			samples_cf32[i*BYTES_PER_SAMPLE+1] = (float_t)(src[i*BYTES_PER_SAMPLE+1]/127.0);
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
		float_t *samples_cf32=(float_t *) src+offset*BYTES_PER_SAMPLE;
		for (uint32_t i=0;i<len;++i){
			dst[i*BYTES_PER_SAMPLE] = (float_t) (samples_cf32[i*BYTES_PER_SAMPLE] * 127.0);
			dst[i*BYTES_PER_SAMPLE+1] = (float_t) (samples_cf32[i*BYTES_PER_SAMPLE+1] * 127.0);
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

	size_t returnedElems = numElems< (_buf_len / BYTES_PER_SAMPLE)?numElems:(_buf_len / BYTES_PER_SAMPLE);


	//_buf_mutex.lock();
	std::unique_lock <std::mutex> lock( _buf_mutex );

	while ( _buf_count < 3 )
		_buf_cond.wait( lock );

	if ( !_running )
		return(SOAPY_SDR_STREAM_ERROR);

	if(_overflow) {
		_overflow=false;
		return SOAPY_SDR_OVERFLOW;
	}

	int8_t *buf = _buf[_buf_head] + _buf_offset * BYTES_PER_SAMPLE;

	if ( returnedElems <= _samp_avail )
	{
		readbuf(buf,buffs[0],returnedElems,_format,0);
		_buf_offset	+= returnedElems;
		_samp_avail	-= returnedElems;
	}else  {

		readbuf(buf,buffs[0],_samp_avail,_format,0);

		_buf_head = (_buf_head + 1) % _buf_num;
		_buf_count--;

		buf = _buf[_buf_head];

		int remaining = returnedElems - _samp_avail;

		readbuf(buf,buffs[0],remaining,_format,_samp_avail);

		_buf_offset	= remaining;
		_samp_avail	= (_buf_len / BYTES_PER_SAMPLE) - remaining;
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

	size_t returnedElems = numElems< (_buf_len / BYTES_PER_SAMPLE)?numElems:(_buf_len / BYTES_PER_SAMPLE);


	std::unique_lock <std::mutex> lock( _buf_mutex );

	while ( _buf_count == _buf_num )
	{
		_buf_cond.wait( lock );
	}


	if(!_running)
		return SOAPY_SDR_STREAM_ERROR;

	int8_t *buf = _buf[_buf_head] + _buf_offset * BYTES_PER_SAMPLE;


	if ( returnedElems <= _samp_avail )
	{
		writebuf(buffs[0],buf,returnedElems,_format,0);

		_buf_offset	+= returnedElems;
		_samp_avail	-= returnedElems;
	}else {

		writebuf(buffs[0],buf,_samp_avail,_format,0);

		_buf_head = (_buf_head + 1) % _buf_num;

		_buf_count++;

		buf = _buf[_buf_head];

		int remaining = returnedElems - _samp_avail;

		writebuf(buffs[0],buf,remaining,_format,_samp_avail);

		_buf_offset	= remaining;
		_samp_avail	= (_buf_len / BYTES_PER_SAMPLE) - remaining;
	}


	return(returnedElems);
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

	int event_code =0;

	if(_underflow){
		_underflow=false;
		event_code =SOAPY_SDR_UNDERFLOW;
	}

	return event_code;

}

