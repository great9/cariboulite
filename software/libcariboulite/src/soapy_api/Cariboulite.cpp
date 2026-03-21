#include <cmath>
#include "Cariboulite.hpp"
#include "cariboulite_config_default.h"

SoapyCaribouliteSession Cariboulite::sess;

// TX power base offset in dBm (AT86RF215 PA characteristic)
static constexpr double TX_POWER_BASE_DBM = -18.0;

// AT86RF215 gain ranges
static constexpr double RX_GAIN_MIN = 0.0;
static constexpr double RX_GAIN_MAX = 69.0;  // 23 steps * 3 dB
static constexpr double TX_GAIN_MIN = 0.0;
static constexpr double TX_GAIN_MAX = 31.0;

/*******************************************************************
 * Constructor / Destructor
 ******************************************************************/
Cariboulite::Cariboulite(const SoapySDR::Kwargs &args)
{
	SoapySDR_logf(SOAPY_SDR_INFO, "Initializing DeviceID: %s, Label: %s, ChannelType: %s",
					args.at("device_id").c_str(),
					args.at("label").c_str(),
					args.at("channel").c_str());

	if (!args.at("channel").compare ("HiF"))
	{
        radio = &sess.sys.radio_high;
	}
	else if (!args.at("channel").compare ("S1G"))
	{
        radio = &sess.sys.radio_low;
	}
	else
	{
		throw std::runtime_error( "Channel type is not specified correctly" );
	}

	stream = new SoapySDR::Stream(radio);
    if (stream == NULL)
    {
        throw std::runtime_error( "Stream allocation failed" );
    }
}

//========================================================
Cariboulite::~Cariboulite()
{
    if (stream)
    {
        delete stream;
        stream = nullptr;
    }
}

/*******************************************************************
 * Identification API
 ******************************************************************/
SoapySDR::Kwargs Cariboulite::getHardwareInfo() const
{
    SoapySDR::Kwargs args;

    uint32_t serial_number = 0;
    uint32_t deviceId = 0;
    int count = 0;
    cariboulite_get_serial_number((sys_st*)&sess.sys, &serial_number, &count) ;

    args["device_id"] = std::to_string(deviceId);
    args["serial_number"] = std::to_string(serial_number);
    args["hardware_revision"] = sess.sys.board_info.product_version;
    args["fpga_revision"] = std::to_string(1);
    args["vendor_name"] = sess.sys.board_info.product_vendor;
    args["product_name"] = sess.sys.board_info.product_name;

    return args;
}

std::string Cariboulite::getHardwareKey() const
{
    return "Cariboulite Rev2.8";
}

/*******************************************************************
 * Antenna API
 ******************************************************************/
std::vector<std::string> Cariboulite::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
    if (radio->type == cariboulite_channel_s1g)
    {
        options.push_back( "TX/RX Sub1GHz" );
    }
    else if (radio->type == cariboulite_channel_hif)
    {
        options.push_back( "TX/RX 30MHz-6GHz" );
    }
	return(options);
}

//========================================================
void Cariboulite::setAntenna( const int direction, const size_t channel, const std::string &name )
{
    // Each channel has a dedicated antenna — validate but nothing to switch
    auto antennas = listAntennas(direction, channel);
    if (std::find(antennas.begin(), antennas.end(), name) == antennas.end())
    {
        throw std::runtime_error("setAntenna: unknown antenna " + name);
    }
}

//========================================================
std::string Cariboulite::getAntenna( const int direction, const size_t channel ) const
{
	if (radio->type == cariboulite_channel_s1g) return "TX/RX Sub1GHz";
    else if (radio->type == cariboulite_channel_hif) return "TX/RX 30MHz-6GHz";
    return "";
}

/*******************************************************************
 * Gain API
 ******************************************************************/
std::vector<std::string> Cariboulite::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> gains;
    if (direction == SOAPY_SDR_RX)
    {
        gains.push_back("Modem AGC");
    }
    else if (direction == SOAPY_SDR_TX)
    {
        gains.push_back("Modem PA");
    }

    return (gains);
}

//========================================================
void Cariboulite::setGain(const int direction, const size_t channel, const double value)
{
    if (direction == SOAPY_SDR_RX)
    {
        // When the user explicitly sets gain, disable AGC so the
        // manual value takes effect.  AGC can be re-enabled via
        // setGainMode(true).
        double clamped = std::max(RX_GAIN_MIN, std::min(value, RX_GAIN_MAX));
        cariboulite_radio_set_rx_gain_control(radio, false, clamped);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        double clamped = std::max(TX_GAIN_MIN, std::min(value, TX_GAIN_MAX));
        cariboulite_radio_set_tx_power(radio, clamped + TX_POWER_BASE_DBM);
    }
}

//========================================================
void Cariboulite::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    Cariboulite::setGain(direction, channel, value);
}

//========================================================
double Cariboulite::getGain(const int direction, const size_t channel) const
{
    int value = 0;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_radio_get_rx_gain_control((cariboulite_radio_state_st*)radio, NULL, &value);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        int temp = 0;
        cariboulite_radio_get_tx_power((cariboulite_radio_state_st*)radio, &temp);
        value = temp - TX_POWER_BASE_DBM;
    }
    return (double)value;
}

//========================================================
double Cariboulite::getGain(const int direction, const size_t channel, const std::string &name) const
{
    return getGain(direction, channel);
}

//========================================================
SoapySDR::Range Cariboulite::getGainRange(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_RX)
    {
        return SoapySDR::Range(RX_GAIN_MIN, RX_GAIN_MAX);
    }
    return SoapySDR::Range(TX_GAIN_MIN, TX_GAIN_MAX);
}

//========================================================
SoapySDR::Range Cariboulite::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    return getGainRange(direction, channel);
}


//========================================================
bool Cariboulite::hasGainMode(const int direction, const size_t channel) const
{
    return (direction == SOAPY_SDR_RX);
}

//========================================================
void Cariboulite::setGainMode( const int direction, const size_t channel, const bool automatic )
{
    if (direction == SOAPY_SDR_RX)
    {
        // Preserve current gain value (as double, not bool)
        double current_gain = (double)radio->rx_gain_value_db;
        cariboulite_radio_set_rx_gain_control(radio, automatic, current_gain);
    }
}

//========================================================
bool Cariboulite::getGainMode( const int direction, const size_t channel ) const
{
    bool mode = false;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_radio_get_rx_gain_control((cariboulite_radio_state_st*)radio, &mode, NULL);
        return mode;
    }

    return (false);
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

// Sample rate tolerance for matching (Hz)
static constexpr double RATE_TOLERANCE = 100.0;

void Cariboulite::setSampleRate( const int direction, const size_t channel, const double rate )
{
    cariboulite_radio_sample_rate_en fs;
    bool matched = true;
    cariboulite_radio_f_cut_en rx_cuttof = radio->rx_fcut;
    cariboulite_radio_f_cut_en tx_cuttof = radio->tx_fcut;

    if      (std::fabs(rate - 4000000.0) < RATE_TOLERANCE)     fs = cariboulite_radio_rx_sample_rate_4000khz;
    else if (std::fabs(rate - 2000000.0) < RATE_TOLERANCE)     fs = cariboulite_radio_rx_sample_rate_2000khz;
    else if (std::fabs(rate - 4000000.0/3) < RATE_TOLERANCE)   fs = cariboulite_radio_rx_sample_rate_1333khz;
    else if (std::fabs(rate - 1000000.0) < RATE_TOLERANCE)     fs = cariboulite_radio_rx_sample_rate_1000khz;
    else if (std::fabs(rate - 800000.0) < RATE_TOLERANCE)      fs = cariboulite_radio_rx_sample_rate_800khz;
    else if (std::fabs(rate - 2000000.0/3) < RATE_TOLERANCE)   fs = cariboulite_radio_rx_sample_rate_666khz;
    else if (std::fabs(rate - 500000.0) < RATE_TOLERANCE)      fs = cariboulite_radio_rx_sample_rate_500khz;
    else if (std::fabs(rate - 400000.0) < RATE_TOLERANCE)      fs = cariboulite_radio_rx_sample_rate_400khz;
    // Deprecated rounded rates — map to correct values with warning
    else if (std::fabs(rate - 1333000.0) < RATE_TOLERANCE)
    {
        fs = cariboulite_radio_rx_sample_rate_1333khz;
        SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: 1333000 is deprecated; use 4e6/3 or 1333333.3");
    }
    else if (std::fabs(rate - 666000.0) < RATE_TOLERANCE)
    {
        fs = cariboulite_radio_rx_sample_rate_666khz;
        SoapySDR_logf(SOAPY_SDR_WARNING, "setSampleRate: 666000 is deprecated; use 2e6/3 or 666666.7");
    }
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setSampleRate: unsupported rate %.0f Hz", rate);
        throw std::runtime_error("Unsupported sample rate: " + std::to_string(rate));
    }

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_radio_set_rx_samp_cutoff((cariboulite_radio_state_st*)radio, fs, rx_cuttof);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_radio_set_tx_samp_cutoff((cariboulite_radio_state_st*)radio, fs, tx_cuttof);
    }
}

//========================================================
double Cariboulite::getSampleRate( const int direction, const size_t channel ) const
{
    cariboulite_radio_sample_rate_en fs = cariboulite_radio_rx_sample_rate_4000khz;

    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_radio_get_rx_samp_cutoff((cariboulite_radio_state_st*)radio, &fs, NULL);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_radio_get_tx_samp_cutoff((cariboulite_radio_state_st*)radio, &fs, NULL);
    }

    switch(fs)
    {
        case cariboulite_radio_rx_sample_rate_4000khz: return 4000000.0;
        case cariboulite_radio_rx_sample_rate_2000khz: return 2000000.0;
        case cariboulite_radio_rx_sample_rate_1333khz: return 4000000.0/3;
        case cariboulite_radio_rx_sample_rate_1000khz: return 1000000.0;
        case cariboulite_radio_rx_sample_rate_800khz: return 800000.0;
        case cariboulite_radio_rx_sample_rate_666khz: return 2000000.0/3;
        case cariboulite_radio_rx_sample_rate_500khz: return 500000.0;
        case cariboulite_radio_rx_sample_rate_400khz: return 400000.0;
    }
    return 4000000;
}

//========================================================
std::vector<double> Cariboulite::listSampleRates( const int direction, const size_t channel ) const
{
    std::vector<double> options;
	options.push_back( 4000000.0 );
    options.push_back( 2000000.0 );
    options.push_back( 4000000.0/3 );
    options.push_back( 1000000.0 );
    options.push_back( 800000.0 );
    options.push_back( 2000000.0/3 );
    options.push_back( 500000.0 );
    options.push_back( 400000.0 );
	return(options);
}

//========================================================
static cariboulite_radio_rx_bw_en convertRxBandwidth(double bw_numeric)
{
    if (std::fabs(bw_numeric - 160000.0) < 1) return cariboulite_radio_rx_bw_160KHz;
    if (std::fabs(bw_numeric - 200000.0) < 1) return cariboulite_radio_rx_bw_200KHz;
    if (std::fabs(bw_numeric - 250000.0) < 1) return cariboulite_radio_rx_bw_250KHz;
    if (std::fabs(bw_numeric - 320000.0) < 1) return cariboulite_radio_rx_bw_320KHz;
    if (std::fabs(bw_numeric - 400000.0) < 1) return cariboulite_radio_rx_bw_400KHz;
    if (std::fabs(bw_numeric - 500000.0) < 1) return cariboulite_radio_rx_bw_500KHz;
    if (std::fabs(bw_numeric - 630000.0) < 1) return cariboulite_radio_rx_bw_630KHz;
    if (std::fabs(bw_numeric - 800000.0) < 1) return cariboulite_radio_rx_bw_800KHz;
    if (std::fabs(bw_numeric - 1000000.0) < 1) return cariboulite_radio_rx_bw_1000KHz;
    if (std::fabs(bw_numeric - 1250000.0) < 1) return cariboulite_radio_rx_bw_1250KHz;
    if (std::fabs(bw_numeric - 1600000.0) < 1) return cariboulite_radio_rx_bw_1600KHz;
    if (std::fabs(bw_numeric - 2000000.0) < 1) return cariboulite_radio_rx_bw_2000KHz;

    return cariboulite_radio_rx_bw_2000KHz;
}

//========================================================
static double convertRxBandwidth(cariboulite_radio_rx_bw_en bw_en)
{
    if (cariboulite_radio_rx_bw_160KHz == bw_en) return 160000.0;
    if (cariboulite_radio_rx_bw_200KHz == bw_en) return 200000.0;
    if (cariboulite_radio_rx_bw_250KHz == bw_en) return 250000.0;
    if (cariboulite_radio_rx_bw_320KHz == bw_en) return 320000.0;
    if (cariboulite_radio_rx_bw_400KHz == bw_en) return 400000.0;
    if (cariboulite_radio_rx_bw_500KHz == bw_en) return 500000.0;
    if (cariboulite_radio_rx_bw_630KHz == bw_en) return 630000.0;
    if (cariboulite_radio_rx_bw_800KHz == bw_en) return 800000.0;
    if (cariboulite_radio_rx_bw_1000KHz == bw_en) return 1000000.0;
    if (cariboulite_radio_rx_bw_1250KHz == bw_en) return 1250000.0;
    if (cariboulite_radio_rx_bw_1600KHz == bw_en) return 1600000.0;
    if (cariboulite_radio_rx_bw_2000KHz == bw_en) return 2000000.0;

    return 2000000.0;
}

//========================================================
static cariboulite_radio_tx_cut_off_en convertTxBandwidth(double bw_numeric)
{
    if (std::fabs(bw_numeric - 80000.0) < 1) return cariboulite_radio_tx_cut_off_80khz;
    if (std::fabs(bw_numeric - 100000.0) < 1) return cariboulite_radio_tx_cut_off_100khz;
    if (std::fabs(bw_numeric - 125000.0) < 1) return cariboulite_radio_tx_cut_off_125khz;
    if (std::fabs(bw_numeric - 160000.0) < 1) return cariboulite_radio_tx_cut_off_160khz;
    if (std::fabs(bw_numeric - 200000.0) < 1) return cariboulite_radio_tx_cut_off_200khz;
    if (std::fabs(bw_numeric - 250000.0) < 1) return cariboulite_radio_tx_cut_off_250khz;
    if (std::fabs(bw_numeric - 315000.0) < 1) return cariboulite_radio_tx_cut_off_315khz;
    if (std::fabs(bw_numeric - 400000.0) < 1) return cariboulite_radio_tx_cut_off_400khz;
    if (std::fabs(bw_numeric - 500000.0) < 1) return cariboulite_radio_tx_cut_off_500khz;
    if (std::fabs(bw_numeric - 625000.0) < 1) return cariboulite_radio_tx_cut_off_625khz;
    if (std::fabs(bw_numeric - 800000.0) < 1) return cariboulite_radio_tx_cut_off_800khz;
    if (std::fabs(bw_numeric - 1000000.0) < 1) return cariboulite_radio_tx_cut_off_1000khz;
    return cariboulite_radio_tx_cut_off_1000khz;
}

//========================================================
static double convertTxBandwidth(cariboulite_radio_tx_cut_off_en bw_en)
{
    if (cariboulite_radio_tx_cut_off_80khz == bw_en) return 80000.0;
    if (cariboulite_radio_tx_cut_off_100khz == bw_en) return 100000.0;
    if (cariboulite_radio_tx_cut_off_125khz == bw_en) return 125000.0;
    if (cariboulite_radio_tx_cut_off_160khz == bw_en) return 160000.0;
    if (cariboulite_radio_tx_cut_off_200khz == bw_en) return 200000.0;
    if (cariboulite_radio_tx_cut_off_250khz == bw_en) return 250000.0;
    if (cariboulite_radio_tx_cut_off_315khz == bw_en) return 315000.0;
    if (cariboulite_radio_tx_cut_off_400khz == bw_en) return 400000.0;
    if (cariboulite_radio_tx_cut_off_500khz == bw_en) return 500000.0;
    if (cariboulite_radio_tx_cut_off_625khz == bw_en) return 625000.0;
    if (cariboulite_radio_tx_cut_off_800khz == bw_en) return 800000.0;
    if (cariboulite_radio_tx_cut_off_1000khz == bw_en) return 1000000.0;
    return 1000000.0;
}


//========================================================
void Cariboulite::setBandwidth( const int direction, const size_t channel, const double bw )
{
	double modem_bw = bw;

    if (direction == SOAPY_SDR_RX)
    {
		if (modem_bw < 160000.0 )
		{
			modem_bw = 160000.0;
			if (bw <= 20000.0) stream->setDigitalFilter(SoapySDR::Stream::DigitalFilter_20KHz);
			else if (bw <= 50000.0) stream->setDigitalFilter(SoapySDR::Stream::DigitalFilter_50KHz);
			else if (bw <= 100000.0) stream->setDigitalFilter(SoapySDR::Stream::DigitalFilter_100KHz);
			else stream->setDigitalFilter(SoapySDR::Stream::DigitalFilter_None);
		}
		else stream->setDigitalFilter(SoapySDR::Stream::DigitalFilter_None);

        // Validate bandwidth <= current sample rate
        double sr = getSampleRate(direction, channel);
        if (modem_bw > sr)
        {
            SoapySDR_logf(SOAPY_SDR_WARNING, "setBandwidth: %.0f exceeds sample rate %.0f, clamping", modem_bw, sr);
            modem_bw = sr;
        }

		cariboulite_radio_set_rx_bandwidth(radio, convertRxBandwidth(modem_bw));
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_radio_set_tx_bandwidth(radio, convertTxBandwidth(modem_bw));
    }
}

//========================================================
double Cariboulite::getBandwidth( const int direction, const size_t channel ) const
{
    if (direction == SOAPY_SDR_RX)
    {
        cariboulite_radio_rx_bw_en bw;
        cariboulite_radio_get_rx_bandwidth((cariboulite_radio_state_st*)radio, &bw);
        auto filter_type = stream->getDigitalFilter();
        if (filter_type == SoapySDR::Stream::DigitalFilter_20KHz) return 20000.0;
        else if (filter_type == SoapySDR::Stream::DigitalFilter_50KHz) return 50000.0;
        else if (filter_type == SoapySDR::Stream::DigitalFilter_100KHz) return 100000.0;
        return convertRxBandwidth(bw);
    }
    else if (direction == SOAPY_SDR_TX)
    {
        cariboulite_radio_tx_cut_off_en bw;
        cariboulite_radio_get_tx_bandwidth((cariboulite_radio_state_st*)radio, &bw);
        return convertTxBandwidth(bw);
    }
    return 0.0;
}

//========================================================
std::vector<double> Cariboulite::listBandwidths( const int direction, const size_t channel ) const
{
    std::vector<double> options;
    if (direction == SOAPY_SDR_RX)
    {
        options.push_back( 20000.0 );
        options.push_back( 50000.0 );
        options.push_back( 100000.0 );
        options.push_back( 160000.0 );
        options.push_back( 200000.0 );
        options.push_back( 250000.0 );
        options.push_back( 320000.0 );
        options.push_back( 400000.0 );
        options.push_back( 500000.0 );
        options.push_back( 630000.0 );
        options.push_back( 800000.0 );
        options.push_back( 1000000.0 );
        options.push_back( 1250000.0 );
        options.push_back( 1600000.0 );
        options.push_back( 2000000.0 );
    }
    else
    {
        options.push_back( 80000.0 );
        options.push_back( 100000.0 );
        options.push_back( 125000.0 );
        options.push_back( 160000.0 );
        options.push_back( 200000.0 );
        options.push_back( 250000.0 );
        options.push_back( 315000.0 );
        options.push_back( 400000.0 );
        options.push_back( 500000.0 );
        options.push_back( 625000.0 );
        options.push_back( 800000.0 );
        options.push_back( 1000000.0 );
    }
    return(options);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/
void Cariboulite::setFrequency( const int direction, const size_t channel, const std::string &name,
                                const double frequency, const SoapySDR::Kwargs &args )
{
    if (name != "RF")
    {
        throw std::runtime_error("setFrequency: unknown frequency name " + name);
    }

    int err = cariboulite_radio_set_frequency(radio, true, (double *)&frequency);
    if (err == 0) SoapySDR_logf(SOAPY_SDR_INFO, "setFrequency dir: %d, channel: %ld, freq: %.2f", direction, channel, frequency);
    else
    {
        SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency dir: %d, channel: %ld, freq: %.2f FAILED", direction, channel, frequency);
        throw std::runtime_error("setFrequency failed for " + std::to_string(frequency) + " Hz");
    }
}

//========================================================
double Cariboulite::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
    double freq;
    if (name != "RF")
    {
        return 0.0;
    }

    cariboulite_radio_get_frequency((cariboulite_radio_state_st*)radio, &freq, NULL, NULL);
    return freq;
}

//========================================================
SoapySDR::ArgInfoList Cariboulite::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    // No additional frequency arguments supported beyond the center frequency
    SoapySDR::ArgInfoList freqArgs;
	return freqArgs;
}

//========================================================
std::vector<std::string> Cariboulite::listFrequencies( const int direction, const size_t channel ) const
{
    std::vector<std::string> names;
	names.push_back( "RF" );
	return(names);
}

//========================================================
SoapySDR::RangeList Cariboulite::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
    if (name != "RF" )
    {
		throw std::runtime_error( "getFrequencyRange(" + name + ") unknown name" );
    }

    if (radio->type == cariboulite_channel_s1g)
    {
        SoapySDR::RangeList list;
        list.push_back(SoapySDR::Range( 389.5e6, 510e6 ));
        list.push_back(SoapySDR::Range( 779e6, 1020e6 ));
        return list;
    }
    else if (radio->type == cariboulite_channel_hif)
    {
        return (SoapySDR::RangeList( 1, SoapySDR::Range( 1e6, 6000e6 ) ) );
    }
    throw std::runtime_error( "getFrequencyRange - unknown channel" );
}

/*******************************************************************
 * Frontend corrections API (stubs)
 ******************************************************************/
bool Cariboulite::hasFrequencyCorrection(const int direction, const size_t channel) const
{
    return false;
}

void Cariboulite::setFrequencyCorrection(const int direction, const size_t channel, const double value)
{
    // Not supported by AT86RF215
}

double Cariboulite::getFrequencyCorrection(const int direction, const size_t channel) const
{
    return 0.0;
}

bool Cariboulite::hasIQBalance(const int direction, const size_t channel) const
{
    return false;
}
