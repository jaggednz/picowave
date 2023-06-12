#ifndef FILTER_H
#define FILTER_H

// Some DSP type aliases
typedef uint32_t integrator;
typedef integrator filter1pole;

// A 32-bit overflow/underflow-safe digital integrator
static inline integrator integrator_feed(integrator *i, uint32_t x)
{
	return *i += x;
}

//! A 1 pole filter based on the above integrator
//! \see integrator
static inline uint32_t filter1pole_feed(filter1pole *f, uint8_t k, uint16_t x)
{
	integrator_feed(f, ((x - (*f >> 8)) * k));
	return *f >> 8;
}

#endif