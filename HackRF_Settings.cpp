/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2016 Wei Jiang
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

#include "SoapyHackRF.hpp"

std::set<std::string> &HackRF_getClaimedSerials(void)
{
	static std::set<std::string> serials;
	return serials;
}

SoapyHackRF::SoapyHackRF( const SoapySDR::Kwargs &args )
{
	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

	_rx_stream.vga_gain=16;
	_rx_stream.lna_gain=16;
	_rx_stream.amp_gain=0;
	_rx_stream.frequency=0;
	_rx_stream.samplerate=0;
	_rx_stream.bandwidth=0;
	_rx_stream.overflow = false;

	_tx_stream.vga_gain=0;
	_tx_stream.amp_gain=0;
	_tx_stream.frequency=0;
	_tx_stream.samplerate=0;
	_tx_stream.bandwidth=0;
	_tx_stream.burst_samps=0;
	_tx_stream.burst_end=false;
	_tx_stream.underflow = false;

	_current_mode=HACKRF_TRANSCEIVER_MODE_OFF;

	_auto_bandwidth=true;

	_dev		= nullptr;

	if (args.count("serial") == 0)
		throw std::runtime_error("no hackrf device matches");
	_serial = args.at("serial");

	_current_amp = 0;

	_current_frequency = 0;

	_current_samplerate = 0;

	_current_bandwidth=0;

	int ret = hackrf_open_by_serial(_serial.c_str(), &_dev);
	if ( ret != HACKRF_SUCCESS )
	{
		SoapySDR_logf( SOAPY_SDR_INFO, "Could not Open HackRF Device" );
		throw std::runtime_error("hackrf open failed");
	}

	HackRF_getClaimedSerials().insert(_serial);
}


SoapyHackRF::~SoapyHackRF( void )
{
	HackRF_getClaimedSerials().erase(_serial);

	if ( _dev )
	{
		hackrf_close( _dev );
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
	std::lock_guard<std::mutex> lock(_device_mutex);
	uint8_t board_id=BOARD_ID_INVALID;

	hackrf_board_id_read(_dev,&board_id);

	return(hackrf_board_id_name((hackrf_board_id)board_id));
}


SoapySDR::Kwargs SoapyHackRF::getHardwareInfo( void ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	SoapySDR::Kwargs info;

	char version_str[100];

	hackrf_version_string_read(_dev, &version_str[0], 100);

	info["version"] = version_str;

	read_partid_serialno_t read_partid_serialno;

	hackrf_board_partid_serialno_read(_dev, &read_partid_serialno);

	char part_id_str[100];

	sprintf(part_id_str, "%08x%08x", read_partid_serialno.part_id[0], read_partid_serialno.part_id[1]);

	info["part id"] = part_id_str;

	char serial_str[100];
	sprintf(serial_str, "%08x%08x%08x%08x", read_partid_serialno.serial_no[0], read_partid_serialno.serial_no[1], read_partid_serialno.serial_no[2], read_partid_serialno.serial_no[3]);
	info["serial"] = serial_str;

	uint16_t clock;

	hackrf_si5351c_read(_dev,0,&clock);

	info["clock source"]=(clock==0x51)?"internal":"external";

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
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyHackRF::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList setArgs;

	SoapySDR::ArgInfo biastxArg;
	biastxArg.key="bias_tx";
	biastxArg.value="false";
	biastxArg.name="Antenna Bias";
	biastxArg.description="Antenna port power control.";
	biastxArg.type=SoapySDR::ArgInfo::BOOL;
	setArgs.push_back(biastxArg);

	return setArgs;
}

void SoapyHackRF::writeSetting(const std::string &key, const std::string &value)
{
	if(key=="bias_tx"){
		std::lock_guard<std::mutex> lock(_device_mutex);
		_tx_stream.bias=(value=="true") ? true : false;
		int ret=hackrf_set_antenna_enable(_dev,_tx_stream.bias);
		if(ret!=HACKRF_SUCCESS){

			SoapySDR_logf(SOAPY_SDR_INFO,"Failed to apply antenna bias voltage");

		}
	}

}

std::string SoapyHackRF::readSetting(const std::string &key) const
{
	if (key == "bias_tx") {
		return _tx_stream.bias?"true":"false";
	}
	return "";
}
/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyHackRF::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back( "TX/RX" );
	return(options);
}


void SoapyHackRF::setAntenna( const int direction, const size_t channel, const std::string &name )
{
	/* TODO delete this function or throw if name != RX... */
}


std::string SoapyHackRF::getAntenna( const int direction, const size_t channel ) const
{
	return("TX/RX");
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
	// in gr-osmosdr/lib/soapy/ soapy_sink_c.cc and soapy_source_c.cc expect if_gain at front and bb_gain at back
		options.push_back( "LNA" );						// RX: if_gain
		options.push_back( "AMP" );						// RX: rf_gain
		options.push_back( "VGA" );						// RX: bb_gain
	}
	else
	{
		options.push_back( "VGA" );						// TX: if_gain
		options.push_back( "AMP" );						// TX: rf_gain
	}

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
	std::lock_guard<std::mutex> lock(_device_mutex);
	int32_t ret(0), gain(0);
	gain = value;
	SoapySDR_logf(SOAPY_SDR_DEBUG,"setGain RF %s, channel %d, gain %d", direction == SOAPY_SDR_RX ? "RX" : "TX", channel, gain);

	if ( direction == SOAPY_SDR_RX )
	{
		if ( gain <= 0 )
		{
			_rx_stream.lna_gain	= 0;
			_rx_stream.vga_gain	= 0;
			_current_amp		= 0;
		}else if ( gain <= (HACKRF_RX_LNA_MAX_DB / 2) + (HACKRF_RX_VGA_MAX_DB / 2) )
		{
			_rx_stream.vga_gain	= (gain / 3) & ~0x1;
			_rx_stream.lna_gain	= gain - _rx_stream.vga_gain;
			_current_amp		= 0;
		}else if ( gain <= ( (HACKRF_RX_LNA_MAX_DB / 2) + (HACKRF_RX_VGA_MAX_DB / 2) + HACKRF_AMP_MAX_DB) )
		{
			_current_amp		= HACKRF_AMP_MAX_DB;
			_rx_stream.vga_gain	= ( (gain - _current_amp) / 3) & ~0x1;
			_rx_stream.lna_gain	= gain -_current_amp - _rx_stream.vga_gain;
		}else if ( gain <= HACKRF_RX_LNA_MAX_DB + HACKRF_RX_VGA_MAX_DB + HACKRF_AMP_MAX_DB )
		{
			_current_amp		= HACKRF_AMP_MAX_DB;
			_rx_stream.vga_gain	= (gain - _current_amp) * double(HACKRF_RX_LNA_MAX_DB) / double(HACKRF_RX_VGA_MAX_DB);
			_rx_stream.lna_gain	= gain - _current_amp - _rx_stream.vga_gain;
		}

		_rx_stream.amp_gain=_current_amp;

		ret	= hackrf_set_lna_gain( _dev, _rx_stream.lna_gain );
		ret	|= hackrf_set_vga_gain( _dev, _rx_stream.vga_gain );
		ret	|= hackrf_set_amp_enable( _dev, (_current_amp > 0) ? 1 : 0 );
	}else if ( direction == SOAPY_SDR_TX )
	{
		if ( gain <= 0 )
		{
			_current_amp		= 0;
			_tx_stream.vga_gain	= 0;
		}else if ( gain <= (HACKRF_TX_VGA_MAX_DB / 2) )
		{
			_current_amp		= 0;
			_tx_stream.vga_gain	= gain;
		}else if ( gain <= HACKRF_TX_VGA_MAX_DB + HACKRF_AMP_MAX_DB )
		{
			_current_amp		= HACKRF_AMP_MAX_DB;
			_tx_stream.vga_gain	= gain - HACKRF_AMP_MAX_DB;
		}

		_tx_stream.amp_gain=_current_amp;

		ret	= hackrf_set_txvga_gain( _dev, _tx_stream.vga_gain );
		ret	|= hackrf_set_amp_enable( _dev, (_current_amp > 0) ? 1 : 0 );
	}
	if ( ret != HACKRF_SUCCESS )
	{
		SoapySDR::logf( SOAPY_SDR_ERROR, "setGain(%f) returned %s", value, hackrf_error_name( (hackrf_error) ret ) );
	}
}


void SoapyHackRF::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	SoapySDR_logf(SOAPY_SDR_DEBUG,"setGain %s %s, channel %d, gain %d", name.c_str(), direction == SOAPY_SDR_RX ? "RX" : "TX", channel, (int)value);
	if ( name == "AMP" )
	{
		_current_amp = value;
		_current_amp = (_current_amp > 0)?HACKRF_AMP_MAX_DB : 0; //clip to possible values

		if(direction == SOAPY_SDR_RX){
			_rx_stream.amp_gain=_current_amp;
		}else if (direction ==SOAPY_SDR_TX){
			_tx_stream.amp_gain=_current_amp;
		}

		if ( _dev != NULL )
		{
			int ret = hackrf_set_amp_enable( _dev, (_current_amp > 0)?1 : 0 );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_amp_enable(%f) returned %s", _current_amp, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_RX and name == "LNA" )
	{
		_rx_stream.lna_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_lna_gain( _dev, _rx_stream.lna_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_lna_gain(%f) returned %s", _rx_stream.lna_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_RX and name == "VGA" )
	{
		_rx_stream.vga_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_vga_gain( _dev, _rx_stream.vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_vga_gain(%f) returned %s", _rx_stream.vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}else if ( direction == SOAPY_SDR_TX and name == "VGA" )
	{
		_tx_stream.vga_gain = value;
		if ( _dev != NULL )
		{
			int ret = hackrf_set_txvga_gain( _dev, _tx_stream.vga_gain );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_txvga_gain(%f) returned %s", _tx_stream.vga_gain, hackrf_error_name( (hackrf_error) ret ) );
			}
		}
	}


	/* set individual gain element by name */
}


double SoapyHackRF::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double gain = 0.0;
	if ( direction == SOAPY_SDR_RX and name == "AMP" )
	{
		gain = _rx_stream.amp_gain;
	}else if ( direction == SOAPY_SDR_TX and name == "AMP" )
	{
		gain = _tx_stream.amp_gain;
	}else if ( direction == SOAPY_SDR_RX and name == "LNA" )
	{
		gain = _rx_stream.lna_gain;
	}else if ( direction == SOAPY_SDR_RX and name == "VGA" )
	{
		gain = _rx_stream.vga_gain;
	}else if ( direction == SOAPY_SDR_TX and name == "VGA" )
	{
		gain = _tx_stream.vga_gain;
	}

	return(gain);
}


SoapySDR::Range SoapyHackRF::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	if ( name == "AMP" )
		return(SoapySDR::Range( 0, HACKRF_AMP_MAX_DB,  HACKRF_AMP_MAX_DB) );
	if ( direction == SOAPY_SDR_RX and name == "LNA" )
		return(SoapySDR::Range( 0, HACKRF_RX_LNA_MAX_DB, 8.0 ) );
	if ( direction == SOAPY_SDR_RX and name == "VGA")
		return(SoapySDR::Range( 0, HACKRF_RX_VGA_MAX_DB, 2.0 ) );
	if ( direction == SOAPY_SDR_TX and name == "VGA" )
		return(SoapySDR::Range( 0, HACKRF_TX_VGA_MAX_DB, 1.0 ) );
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

	std::lock_guard<std::mutex> lock(_device_mutex);
	_current_frequency = frequency;


	if(direction==SOAPY_SDR_RX){

		_rx_stream.frequency=_current_frequency;
	}
	if(direction==SOAPY_SDR_TX){

		_tx_stream.frequency=_current_frequency;
	}

	if ( _dev != NULL )
	{
		int ret = hackrf_set_freq( _dev, _current_frequency );

		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_freq(%f) returned %s", _current_frequency, hackrf_error_name( (hackrf_error) ret ) );
		}
	}
}


double SoapyHackRF::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
	if ( name == "BB" )
		return(0.0);
	if ( name != "RF" )
		throw std::runtime_error( "getFrequency(" + name + ") unknown name" );

	std::lock_guard<std::mutex> lock(_device_mutex);
	double freq(0.0);

	if(direction==SOAPY_SDR_RX){

		freq = _rx_stream.frequency;
	}
	if(direction==SOAPY_SDR_TX){

		freq = _tx_stream.frequency;
	}
	return(freq);
}

SoapySDR::ArgInfoList SoapyHackRF::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList freqArgs;
	// TODO: frequency arguments
	return freqArgs;
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
	std::lock_guard<std::mutex> lock(_device_mutex);
	_current_samplerate = rate;

	if(direction==SOAPY_SDR_RX){

		_rx_stream.samplerate=_current_samplerate;
	}
	if(direction==SOAPY_SDR_TX){

		_tx_stream.samplerate=_current_samplerate;
	}

	if ( _dev != NULL )
	{
		int ret = hackrf_set_sample_rate( _dev, _current_samplerate );

		if ( ret != HACKRF_SUCCESS )
		{
			SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_sample_rate(%f) returned %s", _current_samplerate, hackrf_error_name( (hackrf_error) ret ) );
			throw std::runtime_error( "setSampleRate()" );
		}
	}
}


double SoapyHackRF::getSampleRate( const int direction, const size_t channel ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double samp(0.0);
	if(direction==SOAPY_SDR_RX){

		samp= _rx_stream.samplerate;
	}
	if(direction==SOAPY_SDR_TX){

		samp= _tx_stream.samplerate;
	}

	return(samp);
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
	std::lock_guard<std::mutex> lock(_device_mutex);
	_current_bandwidth = bw;

	if(direction==SOAPY_SDR_RX){

		_rx_stream.bandwidth=_current_bandwidth;
	}
	if(direction==SOAPY_SDR_TX){

		_tx_stream.bandwidth=_current_bandwidth;
	}

	if(_current_bandwidth > 0){
		_auto_bandwidth=false;

		if ( _dev != NULL )
		{
			int ret = hackrf_set_baseband_filter_bandwidth( _dev, _current_bandwidth );
			if ( ret != HACKRF_SUCCESS )
			{
				SoapySDR::logf( SOAPY_SDR_ERROR, "hackrf_set_baseband_filter_bandwidth(%f) returned %s", _current_bandwidth, hackrf_error_name( (hackrf_error) ret ) );
				throw std::runtime_error( "setBandwidth()" );
			}
		}

	}else{
		_auto_bandwidth=true;
	}

}


double SoapyHackRF::getBandwidth( const int direction, const size_t channel ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double bw(0.0);
	if(direction==SOAPY_SDR_RX){

		bw = _rx_stream.bandwidth;
	}
	if(direction==SOAPY_SDR_TX){

		bw = _tx_stream.bandwidth;
	}

	return (bw);
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
