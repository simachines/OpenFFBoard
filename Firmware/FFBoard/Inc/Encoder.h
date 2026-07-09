/*
 * Encoder.h
 *
 *  Created on: 25.01.2020
 *      Author: Yannick
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "FFBoardMain.h"
#include "ChoosableClass.h"

/*
 * Note:
 * Encoders should count UP when turned counterclockwise
 * This is not the default gamepad direction but matches most motors
 * If direction is not fixed the encoder class should provide a reverse option
 */

/*
 * Info encoder type:
 * Incremental: relative position with no homing
 * Incremental with index: Absolute position available after homing
 * Absolute: Absolute position always available
 */
enum class EncoderType : uint8_t {NONE=0,incremental=1,incrementalIndex=2,absolute=3};

class Encoder : public ChoosableClass {
public:
	Encoder();
	virtual ~Encoder();
	static ClassIdentifier info;
	const ClassIdentifier getInfo();

	virtual EncoderType getEncoderType();

	// --- Cached accessors (non-virtual — subclasses cannot override) ---
	// Returns the latest sampler value if sampler is active, else falls
	// through to getPosHardware() for non-TMC encoders.
	int32_t getPos();
	int32_t getPosAbs();
	virtual float   getPos_f();
	float   getPosAbs_f();
	float   getVelocity() const;      // turns/s, signed
	float   getVelocityRpm() const;   // RPM, absolute
	uint32_t getPosMicros() const;    // timestamp of last sample (micros())

	// --- Hardware hooks (virtual — subclasses override these) ---
	// Body = exactly today's getPos() / getPosAbs() body, just renamed.
	virtual int32_t getPosHardware();
	virtual int32_t getPosAbsHardware();

	// Called by the driver's sampler thread (notification-driven from the
	// hardware-timer ISR) to update the cache. Runs in THREAD context, NOT ISR,
	// because getPosHardware() may do blocking SPI/CAN. The ISR signals a
	// high-priority thread that calls this; thread-wake jitter is absorbed by
	// measuring actual dt via micros().
	void sampleNow();

	// Driver calls this to activate sampling. After this, getPos() returns
	// the cache instead of calling getPosHardware() directly.
	void activateSampler() { sampler_active = true; }
	bool isSamplerActive() const { return sampler_active; }

	virtual void setPos(int32_t pos);

	virtual uint32_t getCpr(); // Encoder counts per rotation


	static const std::vector<class_entry<Encoder> > all_encoders;
	virtual const ClassType getClassType() override {return ClassType::Encoder;};

protected:
	uint32_t cpr = 0;

	// --- Cache members (volatile: writer = ISR, readers = any thread) ---
	volatile int32_t cached_pos        = 0;   // raw counts
	volatile int32_t cached_pos_abs    = 0;
	volatile float  cached_pos_f       = 0.0f;     // modulo 1.0 turns
	volatile float  cached_pos_abs_f   = 0.0f;     // multi-turn
	volatile float  cached_velocity_turns_s = 0.0f;
	volatile float  cached_velocity_rpm     = 0.0f;
	volatile uint32_t cached_pos_micros     = 0;
	volatile bool   cache_valid       = false;
	bool            sampler_active    = false;   // set by activateSampler()
	float           prev_sample_pos_f = 0.0f;    // sampler-local
	uint32_t        last_sample_us    = 0;       // for actual-dt velocity (thread-context sampler)
};


#endif /* ENCODER_H_ */
