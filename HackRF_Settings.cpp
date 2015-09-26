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

SoapyHackRF::SoapyHackRF( const SoapySDR::Kwargs &args )
{
	_running	= false;
	
	_dev		= NULL;

	_rx_vga_gain = 0;

	_rx_lna_gain = 0;

	_tx_vga_gain = 0;

	_amp = 0;

	_frequency = 1000000;

	_samplerate = _bandwidth = 8000000;

	_id = -1;

	_buf_num	= BUF_NUM;
	_buf_len	= BUF_LEN;

	hackrf_init();


	hackrf_device_list_t	* list = hackrf_device_list();

	if ( args.count( "hackrf" ) != 0 )
	{
		int _id = std::stoi( args.at( "hackrf" ) );
		
		if ( _id < 0 && _id > list->devicecount )
		{
			throw std::runtime_error( "hackrf out of range [0 .. " + std::to_string( list->devicecount ) + "]." );
		}
		int ret = hackrf_device_list_open(list, _id, &_dev );
		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR_logf( SOAPY_SDR_DEBUG, "Could not Open HackRF Device by Index:%d", _id );
		}
	} else if ( _id == -1 )
	{
		int ret = hackrf_open( &_dev );
		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR_logf( SOAPY_SDR_DEBUG, "Could not Open HackRF Device" );
		}
	}

	if ( args.count( "buffers" ) != 0 )
	{
		_buf_num = std::stoi( args.at( "buffers" ) );

	} else {
		_buf_num = BUF_NUM;
	}
	
	hackrf_device_list_free(list);

}


SoapyHackRF::~SoapyHackRF( void )
{
	if ( _dev )
	{
		hackrf_close( _dev );
		hackrf_exit();
	}
	

	/* cleanup device handles */
}


/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyHackRF::getDriverKey( void ) const
{

	return("HackRF");
}


std::string SoapyHackRF::getHardwareKey( void ) const
{
	uint8_t board_id=BOARD_ID_INVALID;

	if(_dev!=NULL){
		hackrf_board_id_read(_dev,&board_id);
	}
	return(hackrf_board_id_name((hackrf_board_id)board_id));
}


SoapySDR::Kwargs SoapyHackRF::getHardwareInfo( void ) const
{
	SoapySDR::Kwargs info;
	info["Buffer Size"] = std::to_string( _buf_len * _buf_num * 1.0 / 1024 / 1024 ) + "MB";

	return(info);

}


/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyHackRF::getNumChannels( const int dir ) const
{
	return(1);
}


bool SoapyHackRF::getFullDuplex( const int direction, const size_t channel ) const
{
	return(false);
}


/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyHackRF::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	if ( direction == SOAPY_SDR_TX )
		options.push_back( "TX" );
	if ( direction == SOAPY_SDR_RX )
		options.push_back( "RX" );
	return(options);
}


void SoapyHackRF::setAntenna( const int direction, const size_t channel, const std::string &name )
{
	/* TODO delete this function or throw if name != RX... */
}


std::string SoapyHackRF::getAntenna( const int direction, const size_t channel ) const
{
	if ( direction == SOAPY_SDR_TX )
		return("TX");
	if ( direction == SOAPY_SDR_RX )
		return("RX");
	return("");
}


/*******************************************************************
 * Frontend corrections API
 ******************************************************************/


bool SoapyHackRF::hasDCOffsetMode( const int direction, const size_t channel ) const
{
	return(false);
}


/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyHackRF::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	if ( direction == SOAPY_SDR_RX )
	{
		options.push_back( "LNA" );
	}
	options.push_back( "VGA" );
	options.push_back( "AMP" );

	return(options);
	/*
	 * list available gain elements,
	 * the functions below have a "name" parameter
	 */
}


void SoapyHackRF::setGainMode( const int direction, const size_t channel, const bool automatic )
{
	/* enable AGC if the hardware supports it, or remove this function */
}


bool SoapyHackRF::getGainMode( const int direction, const size_t channel ) const
{
	return(false);
	/* ditto for the AGC */
}


void SoapyHackRF::setGain( const int direction, const size_t channel, const double value )
{
	if ( direction == SOAPY_SDR_RX )
	{
		_rx_vga_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_vga_gain( _dev, _rx_vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_vga_gain(%f) returned %s", _rx_vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_TX )
	{
		_tx_vga_gain = (uint32_t)value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_txvga_gain( _dev, _tx_vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_txvga_gain(%f) returned %s", _tx_vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}
}


void SoapyHackRF::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	if ( name == "AMP" )
	{
		_amp = value > 0 ? 1 : 0;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_amp_enable( _dev, _amp );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_amp_enable(%f) returned %s", _amp, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_RX and name == "LNA" )
	{
		_rx_lna_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_lna_gain( _dev, _rx_lna_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_lna_gain(%f) returned %s", _rx_lna_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_RX and name == "VGA" )
	{
		_rx_vga_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_vga_gain( _dev, _rx_vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_vga_gain(%f) returned %s", _rx_vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_TX and name == "VGA" )
	{
		_tx_vga_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_txvga_gain( _dev, _tx_vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_txvga_gain(%f) returned %s", _tx_vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}


	/* set individual gain element by name */
}


double SoapyHackRF::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	double gain = 0.0;
	if ( name == "AMP" )
	{
		gain = _amp;
	}else if ( direction == SOAPY_SDR_RX and name == "LNA" )
	{
		gain = _rx_lna_gain;
	}else if ( direction == SOAPY_SDR_RX and name == "VGA" )
	{
		gain = _rx_vga_gain;
	}else if ( direction == SOAPY_SDR_TX and name == "VGA" )
	{
		gain = _tx_vga_gain;
	}

	return(gain);
}


SoapySDR::Range SoapyHackRF::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	if ( name == "AMP" )
		return(SoapySDR::Range( 0, 1 ) );
	if ( direction == SOAPY_SDR_RX and name == "LNA" )
		return(SoapySDR::Range( 0, 40 ) );
	if ( direction == SOAPY_SDR_RX and name == "VGA" )
		return(SoapySDR::Range( 0, 62 ) );
	if ( direction == SOAPY_SDR_TX and name == "VGA" )
		return(SoapySDR::Range( 0, 47 ) );
	return(SoapySDR::Range( 0, 0 ) );
}


/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyHackRF::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	if ( name == "BB" )
		return;
	if ( name != "RF" )
		throw std::runtime_error( "setFrequency(" + name + ") unknown name" );

	_frequency = frequency;

	if ( _dev != NULL )
	{
		int ret = hackrf_set_freq( _dev, _frequency );

		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_freq(%f) returned %s", _frequency, hackrf_error_name( (hackrf_error) ret ) );
		}
	}
}


double SoapyHackRF::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
	if ( name == "BB" )
		return(0.0);
	if ( name != "RF" )
		throw std::runtime_error( "getFrequency(" + name + ") unknown name" );
	return(_frequency);
}


std::vector<std::string> SoapyHackRF::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}


SoapySDR::RangeList SoapyHackRF::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
	if ( name == "BB" )
		return(SoapySDR::RangeList( 1, SoapySDR::Range( 0.0, 0.0 ) ) );
	if ( name != "RF" )
		throw std::runtime_error( "getFrequencyRange(" + name + ") unknown name" );
	return(SoapySDR::RangeList( 1, SoapySDR::Range( 0, 7250000000ull ) ) );
}


/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyHackRF::setSampleRate( const int direction, const size_t channel, const double rate )
{
	_samplerate = rate;

	if ( _dev != NULL )
	{
		int ret = hackrf_set_sample_rate( _dev, _samplerate );
		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_sample_rate(%f) returned %s", _samplerate, hackrf_error_name( (hackrf_error) ret ) );
			throw std::runtime_error( "setSampleRate()" );
		}
	}
}


double SoapyHackRF::getSampleRate( const int direction, const size_t channel ) const
{
	return(_samplerate);
}


std::vector<double> SoapyHackRF::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	for ( double r = 1e6; r <= 20e6; r += 1e6 )
	{
		options.push_back( r );
	}
	return(options);
}


void SoapyHackRF::setBandwidth( const int direction, const size_t channel, const double bw )
{
	_bandwidth = bw;
	if ( _dev != NULL )
	{
		int ret = hackrf_set_baseband_filter_bandwidth( _dev, _bandwidth );
		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_baseband_filter_bandwidth(%f) returned %s", _bandwidth, hackrf_error_name( (hackrf_error) ret ) );
			throw std::runtime_error( "setBandwidth()" );
		}
	}
}


double SoapyHackRF::getBandwidth( const int direction, const size_t channel ) const
{
	return(_bandwidth);
}


std::vector<double> SoapyHackRF::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	options.push_back( 1750000 );
	options.push_back( 2500000 );
	options.push_back( 3500000 );
	options.push_back( 5000000 );
	options.push_back( 5500000 );
	options.push_back( 6000000 );
	options.push_back( 7000000 );
	options.push_back( 8000000 );
	options.push_back( 9000000 );
	options.push_back( 10000000 );
	options.push_back( 12000000 );
	options.push_back( 14000000 );
	options.push_back( 15000000 );
	options.push_back( 20000000 );
	options.push_back( 24000000 );
	options.push_back( 28000000 );
	return(options);
}


