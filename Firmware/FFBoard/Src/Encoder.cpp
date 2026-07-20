/*
 * Encoder.cpp
 *
 *  Created on: 25.01.2020
 *      Author: Yannick
 */

#include "Encoder.h"
#include "ClassChooser.h"
#include "cppmain.h"
#include "cpp_target_config.h"

ClassIdentifier Encoder::info ={.name = "None" , .id=CLSID_ENCODER_NONE, .visibility = ClassVisibility::visible};


const ClassIdentifier Encoder::getInfo(){
	return info;
}

Encoder::Encoder() {

}

Encoder::~Encoder() {

}

/**
 * Returns the type of the encoder. Must override this and NOT return NONE in other classes
 */
EncoderType Encoder::getEncoderType(){
	return EncoderType::NONE;
}

/**
 * Gets the amount of counts per full rotation of the encoder
 */
uint32_t Encoder::getCpr(){
	return this->cpr;
}


// --- Hardware hooks (base defaults) ---

int32_t Encoder::getPosHardware(){
	return 0;
}

int32_t Encoder::getPosAbsHardware(){
	return getPosHardware();
}


// --- Cached accessors ---

/**
 * Returns the encoder position as raw counts.
 * If sampler is active, returns the ISR-updated cache.
 * Otherwise falls through to getPosHardware() (non-TMC encoders).
 */
int32_t Encoder::getPos() {
	return sampler_active ? cached_pos : getPosHardware();
}

/**
 * Returns absolute positions without offsets for absolute encoders.
 * If sampler is active, returns the ISR-updated cache.
 * Otherwise falls through to getPosAbsHardware().
 */
int32_t Encoder::getPosAbs() {
	return sampler_active ? cached_pos_abs : getPosAbsHardware();
}


/**
 * Returns a float position in rotations (modulo 1.0 turn).
 * If sampler is active, returns the ISR-updated cache.
 */
float Encoder::getPos_f() {
	if (sampler_active) return cached_pos_f;
	int32_t icpr = getCpr();
	if (icpr == 0) return 0.0f;
	int32_t pos = getPosHardware();
	int32_t turns = pos / icpr;
	int32_t remainder = pos % icpr;
	return (float)turns + ((float)remainder / (float)icpr);
}

float Encoder::getPosAbs_f() {
	if (sampler_active) return cached_pos_abs_f;
	int32_t icpr = getCpr();
	if (icpr == 0) return 0.0f;
	return (float)getPosAbsHardware() / (float)icpr;
}

float    Encoder::getVelocity()    const { return cached_velocity_turns_s; }
float    Encoder::getVelocityRpm() const { return cached_velocity_rpm; }
uint32_t Encoder::getPosMicros()   const { return cached_pos_micros; }


// --- Sampler (called from driver's sampler thread, notified by hardware-timer ISR) ---

/**
 * Runs in THREAD context (NOT ISR). The driver's sampler thread calls this
 * after being woken by the hardware-timer ISR. getPosHardware() may do
 * blocking SPI/CAN, which is only legal from thread context.
 *
 * dt is measured via micros() between successive calls to absorb thread-wake
 * jitter (~5–20 µs at prio 34). First sample uses TIM_TMC_ARR as nominal dt.
 */
void Encoder::sampleNow() {
	int32_t icpr = getCpr();
	if (icpr == 0) return;
	int32_t raw = getPosHardware();     // THE ONLY HARDWARE READ
	uint32_t now = micros();

	int32_t rem = raw % icpr;
	int32_t turns = raw / icpr;
	// C++ / and % truncate toward zero, not floor. For negative raw the
	// remainder is negative; adjusting it to positive WITHOUT decrementing
	// turns shifts the multi-turn position by +1 rev, causing a full 360°
	// discontinuity every time raw crosses zero. Correct to floor semantics.
	if (rem < 0) {
		rem += icpr;
		turns -= 1;
	}
	float pos_f = (float)rem / (float)icpr;
	float pos_abs_f = (float)turns + pos_f;

	// dt = actual elapsed microseconds (absorbs thread-wake jitter).
	// First sample: use TIM_TMC_ARR as nominal dt.
	float dt = (last_sample_us == 0) ? ((float)TIM_TMC_ARR / 1.0e6f)
	                                 : ((float)(now - last_sample_us) / 1.0e6f);
	if (dt <= 0.0f || dt > 0.01f) dt = (float)TIM_TMC_ARR / 1.0e6f; // sanity clamp

	float delta = pos_f - prev_sample_pos_f;
	if (delta > 0.5f)  delta -= 1.0f;
	if (delta < -0.5f) delta += 1.0f;
	float v = delta / dt;

	cached_pos              = raw;
	cached_pos_abs          = raw;
	cached_pos_f            = pos_abs_f;    // multi-turn — matches getPos_f() fallback
	cached_pos_abs_f        = pos_abs_f;
	cached_velocity_turns_s = v;
	cached_velocity_rpm     = fabsf(v) * 60.0f;
	cached_pos_micros       = now;
	prev_sample_pos_f       = pos_f;
	last_sample_us          = now;
	cache_valid             = true;
}

/**
 * Change the position of the encoder
 * Can be used to reset the center
 */
void Encoder::setPos(int32_t pos){

}




