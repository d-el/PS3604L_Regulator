typedef enum {
	// Target
	Nversion_major = 0u,
	Nversion_minor = 1u,
	Nversion_revision = 2u,
	Nversion_build = 3u,
	Nbodbus_address = 4u,

	// Target
	Nvoltage_set = 5u,
	Ncurrent_set = 6u,
	Nvdac = 7u,
	Nidac = 8u,
	Nmode = 9u,
	Ntime_set = 10u,
	Nenable = 11u,

	// Measurement
	Nvoltage = 12u,
	Ncurrent = 13u,
	Npower = 14u,
	Nresistance = 15u,
	Ntime = 16u,
	Ncapacity = 17u,
	Ninput_voltage = 18u,
	Ntemperature = 19u,
	Nstate = 20u,
	Nvadc = 21u,
	Niadc = 22u,
	Niexternaladc = 23u,

	// calibration
	Nv0_u = 24u,
	Nv1_u = 25u,
	Nv2_u = 26u,
	Nv3_u = 27u,
	Ni0_i = 28u,
	Ni1_i = 29u,
	Ni2_i = 30u,
	Ni3_i = 31u,

	// calibration
	Niext0_i = 32u,
	Niext1_i = 33u,
	Niext2_i = 34u,
	Niext3_i = 35u,
	Nv0_adc = 36u,
	Nv1_adc = 37u,
	Nv2_adc = 38u,
	Nv3_adc = 39u,
	Nv0_dac = 40u,
	Nv1_dac = 41u,
	Nv2_dac = 42u,
	Nv3_dac = 43u,
	Ni0_adc = 44u,
	Ni1_adc = 45u,
	Ni2_adc = 46u,
	Ni3_adc = 47u,
	Ni0_dac = 48u,
	Ni1_dac = 49u,
	Ni2_dac = 50u,
	Ni3_dac = 51u,
	Niext0_adc = 52u,
	Niext1_adc = 53u,
	Niext2_adc = 54u,
	Niext3_adc = 55u,

	endOfNumberPrm = 56u
} parametresNum_type;

typedef enum {
	m_overCurrent = 1,
	m_limitation = 2,
	m_externaIDac = 4,
	m_overheated = 8,
	m_errorTemperatureSensor = 16,
	m_lowInputVoltage = 32,
	m_reverseVoltage = 64,
	m_notCalibrated = 128,
} maskstate_type;

typedef enum {
	overcurrentShutdown = 0,
	limitation = 1,
	timeShutdown = 2,
	lowCurrentShutdown = 3,
	dacMode = 4,
} maskmode_type;

typedef struct {
	// Target
	typeu16Frmt version_major;
	typeu16Frmt version_minor;
	typeu16Frmt version_revision;
	typeu16Frmt version_build;
	typeu16Frmt bodbus_address;

	// Target
	typeu32Frmt voltage_set;
	typeu32Frmt current_set;
	typeu16Frmt vdac;
	typeu16Frmt idac;
	typeu16Frmt mode;
	typeu32Frmt time_set;
	typeboolFrmt enable;

	// Measurement
	typeu32Frmt voltage;
	typeu32Frmt current;
	typeu32Frmt power;
	typeu32Frmt resistance;
	typeu32Frmt time;
	typeu32Frmt capacity;
	typeu32Frmt input_voltage;
	typeu16Frmt temperature;
	typeu16Frmt state;
	typeu16Frmt vadc;
	typeu16Frmt iadc;
	types16Frmt iexternaladc;

	// calibration
	typeu32Frmt v0_u;
	typeu32Frmt v1_u;
	typeu32Frmt v2_u;
	typeu32Frmt v3_u;
	typeu32Frmt i0_i;
	typeu32Frmt i1_i;
	typeu32Frmt i2_i;
	typeu32Frmt i3_i;

	// calibration
	typeu32Frmt iext0_i;
	typeu32Frmt iext1_i;
	typeu32Frmt iext2_i;
	typeu32Frmt iext3_i;
	typeu16Frmt v0_adc;
	typeu16Frmt v1_adc;
	typeu16Frmt v2_adc;
	typeu16Frmt v3_adc;
	typeu16Frmt v0_dac;
	typeu16Frmt v1_dac;
	typeu16Frmt v2_dac;
	typeu16Frmt v3_dac;
	typeu16Frmt i0_adc;
	typeu16Frmt i1_adc;
	typeu16Frmt i2_adc;
	typeu16Frmt i3_adc;
	typeu16Frmt i0_dac;
	typeu16Frmt i1_dac;
	typeu16Frmt i2_dac;
	typeu16Frmt i3_dac;
	types16Frmt iext0_adc;
	types16Frmt iext1_adc;
	types16Frmt iext2_adc;
	types16Frmt iext3_adc;
} prmData_type;

