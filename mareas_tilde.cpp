#include "c74_msp.h"

#include "stmlib/dsp/hysteresis_quantizer.h"
#include "stmlib/dsp/units.h"
#include "tides2/poly_slope_generator.h"
#include "tides2/ramp_extractor.h"
#include "tides2/io_buffer.h"

using namespace c74::max;

static t_class* this_class = nullptr;

static const float kRootScaled[3] = {
	0.125f,
	2.0f,
	130.81f
};

static const tides2::Ratio kRatios[20] = {
	{ 0.0625f, 16 },
	{ 0.125f, 8 },
	{ 0.1666666f, 6 },
	{ 0.25f, 4 },
	{ 0.3333333f, 3 },
	{ 0.5f, 2 },
	{ 0.6666666f, 3 },
	{ 0.75f, 4 },
	{ 0.8f, 5 },
	{ 1, 1 },
	{ 1, 1 },
	{ 1.25f, 4 },
	{ 1.3333333f, 3 },
	{ 1.5f, 2 },
	{ 2.0f, 1 },
	{ 3.0f, 1 },
	{ 4.0f, 1 },
	{ 6.0f, 1 },
	{ 8.0f, 1 },
	{ 16.0f, 1 },
};

inline float min(float a, float b) {
	return (a < b) ? a : b;
}

inline float max(float a, float b) {
	return (a > b) ? a : b;
}

inline float clamp(float x, float a, float b) {
	return min(max(x, a), b);
}

struct t_mareas {
	t_pxobject x_obj;

	double range_param;
	double mode_param;
	double ramp_param;
	double frequency_param;
	double shape_param;
	double smoothness_param;
	double slope_param;
	double shift_param;
	double trigger;
	double cvinput;

	t_mareas();
	void init();
	void step();
	void changeRate();

	long sf = 0;
	long sr = 48000.0f;

	tides2::PolySlopeGenerator poly_slope_generator;
	tides2::RampExtractor ramp_extractor;
	stmlib::HysteresisQuantizer ratio_index_quantizer;

	// State
	int range;
	tides2::OutputMode output_mode;
	tides2::RampMode ramp_mode;

	// Buffers
	tides2::PolySlopeGenerator::OutputSample out[tides2::kBlockSize] = {};
	stmlib::GateFlags trig_flags[tides2::kBlockSize] = {};
	stmlib::GateFlags clock_flags[tides2::kBlockSize] = {};
	stmlib::GateFlags previous_trig_flag = stmlib::GATE_FLAG_LOW;
	stmlib::GateFlags previous_clock_flag = stmlib::GATE_FLAG_LOW;

	bool must_reset_ramp_extractor = true;
	tides2::OutputMode previous_output_mode = tides2::OUTPUT_MODE_GATES;
	uint8_t frame = 0;


};

t_mareas::t_mareas() {
}

void t_mareas::init() {

	range_param = 0.;
	mode_param = 0.;
	ramp_param = 0.;
	frequency_param = 0.;
	shape_param = 0.;
	smoothness_param = 0.5;
	slope_param = 0.5;
	shift_param = 0.5;

	poly_slope_generator.Init();
	ratio_index_quantizer.Init();

	range = 1;
	trigger = 0.;
	cvinput = 0.;
	output_mode = tides2::OUTPUT_MODE_GATES;
	ramp_mode = tides2::RAMP_MODE_LOOPING;

	changeRate();

}

void t_mareas::changeRate() {
	ramp_extractor.Init(sr, 40.f / sr);
}

void* mareas_new(void) {

	t_mareas* self = (t_mareas*) object_alloc(this_class);

	dsp_setup((t_pxobject*)self, 0);
	outlet_new(self, "signal");
	outlet_new(self, "signal");
	outlet_new(self, "signal");
	outlet_new(self, "signal");

	self->x_obj.z_misc = Z_NO_INPLACE;
	self->init();

	return (void *)self;
}


void mareas_perform64(t_mareas* self, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam) {

    double    *out = outs[0];
    double    *out2 = outs[1];
    double    *out3 = outs[2];
    double    *out4 = outs[3];

	if (self->sf!=sampleframes){
		self->sf=sampleframes;
		self->changeRate();
	}

	if (self->output_mode != self->previous_output_mode) {
		self->poly_slope_generator.Reset();
		self->previous_output_mode = self->output_mode;
	}

	self->poly_slope_generator.Render(self->ramp_mode, self->output_mode, tides2::RANGE_AUDIO, self->frequency_param / self->sr, self->slope_param,
			self->shape_param,
			self->smoothness_param, self->shift_param, self->trig_flags,
			NULL, self->out, tides2::kBlockSize);

	for (int j = 0; j < tides2::kBlockSize; j++) {
		*out++  = self->out[j].channel[0];
		*out2++ = self->out[j].channel[1];
		*out3++ = self->out[j].channel[2];
		*out4++ = self->out[j].channel[3];
	}
}

void mareas_free(t_mareas* self) {
	dsp_free((t_pxobject*)self);
}

void mareas_dsp64(t_mareas* self, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags) {
	object_method_direct(void, (t_object*, t_object*, t_perfroutine64, long, void*),
						 dsp64, gensym("dsp_add64"), (t_object*)self, (t_perfroutine64)mareas_perform64, 0, NULL);

	if (self->sr!=samplerate)
	{
		self->sr=samplerate;
		self->changeRate();
	}
}


void mareas_assist(t_mareas* self, void* unused, t_assist_function io, long index, char* string_dest) {
	if (io == ASSIST_INLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"(parameters) IN", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
	else if (io == ASSIST_OUTLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"(signal) 1", ASSIST_STRING_MAXSIZE); 
				break;
			case 1: 
				strncpy(string_dest,"(signal) 2", ASSIST_STRING_MAXSIZE); 
				break;
			case 2: 
				strncpy(string_dest,"(signal) 3", ASSIST_STRING_MAXSIZE); 
				break;
			case 3: 
				strncpy(string_dest,"(signal) 4", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
}

void mareas_range(t_mareas *x, double f){
	x->range_param = f;
};

void mareas_mode(t_mareas *x, double f){
	int fint = f;
	switch (fint) {
		case 1: 
			x->output_mode = tides2::OUTPUT_MODE_GATES;
			break;
		case 2:
			x->output_mode = tides2::OUTPUT_MODE_AMPLITUDE;
			break;
		case 3:
			x->output_mode = tides2::OUTPUT_MODE_SLOPE_PHASE;
			break;
		case 4:
			x->output_mode = tides2::OUTPUT_MODE_FREQUENCY;
			break;
		case 5:
			x->output_mode = tides2::OUTPUT_MODE_LAST;
			break;
		default:
			x->output_mode = tides2::OUTPUT_MODE_AMPLITUDE;
			break;
	}
};

void mareas_ramp(t_mareas *x, double f){
	x->ramp_param = f;
};

void mareas_frequency(t_mareas *x, double f){
	x->frequency_param = f;
};

void mareas_shape(t_mareas *x, double f){
	x->shape_param = f;
};

void mareas_smoothness(t_mareas *x, double f){
	x->smoothness_param = f;
};

void mareas_slope(t_mareas *x, double f){
	x->slope_param = f;
};

void mareas_shift(t_mareas *x, double f){
	x->shift_param = f;
};

void mareas_cvinput(t_mareas *x, double f){
	x->cvinput = f;
};

void mareas_trigger(t_mareas *x, double f){
	x->trigger = f;
};

void ext_main(void* r) {
	this_class = class_new("mareas~", (method)mareas_new, (method)mareas_free, sizeof(t_mareas), NULL, A_GIMME, 0);

	class_addmethod(this_class,(method) mareas_dsp64, "dsp64",	A_CANT,		0);

	class_addmethod(this_class,(method) mareas_range,"range",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_mode,"mode",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_ramp,"ramp",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_frequency,"frequency",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_shape,"shape",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_smoothness,"smoothness",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_slope,"slope",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_shift,"shift",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_trigger,"trigger",A_DEFFLOAT,0);
	class_addmethod(this_class,(method) mareas_cvinput,"cvinput",A_DEFFLOAT,0);

	class_addmethod(this_class,(method) mareas_assist, "assist",	A_CANT,		0);

	class_dspinit(this_class);
	class_register(CLASS_BOX, this_class);
}