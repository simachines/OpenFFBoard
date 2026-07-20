/*
 * TMC4671.cpp
 *
 *  Created on: Feb 1, 2020
 *	  Author: Yannick, Vincent
 */

#include "TMC4671.h"
#include <cstdarg>
#include <new>
#ifdef TMC4671DRIVER
#include "ledEffects.h"
#include "voltagesense.h"
#include <math.h>
#include <assert.h>
#include "ErrorHandler.h"
#include "cpp_target_config.h"
#define MAX_TMC_DRIVERS 3

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
volatile TMC4671CoggingDebugVars g_tmc4671_cogging_debug{};
#endif

ClassIdentifier TMC_1::info = {
	.name = "TMC4671 (CS 1)",
	.id=CLSID_MOT_TMC0, // 1
};


bool TMC_1::isCreatable() {
	return motor_spi.isPinFree(*motor_spi.getCsPin(0));
}


ClassIdentifier TMC_2::info = {
	.name = "TMC4671 (CS 2)" ,
	.id=CLSID_MOT_TMC1,
};


bool TMC_2::isCreatable() {
	return motor_spi.isPinFree(*motor_spi.getCsPin(1));
}




ClassIdentifier TMC4671::info = {
	.name = "TMC4671" ,
	.id=CLSID_MOT_TMC0,
};



TMC4671::TMC4671(SPIPort& spiport,OutputPin cspin,uint8_t address) :
		CommandHandler("tmc", CLSID_MOT_TMC0,address-1), SPIDevice{motor_spi,cspin},Thread("TMC", TMC_THREAD_MEM, TMC_THREAD_PRIO)
{
	this->drv_address = address;
	CommandHandler::setCommandsEnabled(false);
	setAddress(address);
	registerCommands();
	spiConfig.peripheral = motor_spi.getPortHandle()->Init;
	spiConfig.peripheral.Mode = SPI_MODE_MASTER;
	spiConfig.peripheral.Direction = SPI_DIRECTION_2LINES;
	spiConfig.peripheral.DataSize = SPI_DATASIZE_8BIT;
	spiConfig.peripheral.CLKPolarity = SPI_POLARITY_HIGH;
	spiConfig.peripheral.CLKPhase = SPI_PHASE_2EDGE;
	spiConfig.peripheral.NSS = SPI_NSS_SOFT;
	spiConfig.peripheral.BaudRatePrescaler = spiPort.getClosestPrescaler(8e6,0,10e6).first; // 8 target, 10MHz max
	spiConfig.peripheral.FirstBit = SPI_FIRSTBIT_MSB;
	spiConfig.peripheral.TIMode = SPI_TIMODE_DISABLE;
	spiConfig.peripheral.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spiConfig.cspol = true;

#ifdef TMC4671_SPI_DATA_IDLENESS
	spiConfig.peripheral.MasterInterDataIdleness = TMC4671_SPI_DATA_IDLENESS;
#endif

	spiPort.takeSemaphore();
	spiPort.configurePort(&spiConfig.peripheral);
	spiPort.giveSemaphore();

	this->restoreFlash();
	CommandHandler::setCommandsEnabled(true);

	// Start the sampler thread (woken by TIM_TMC ISR, does SPI in thread context).
	if (samplerThread == nullptr)
		samplerThread = std::make_unique<TMC_SamplerThread>(this);
}


TMC4671::~TMC4671() {
	enablePin.reset();
	//recordSpiAddrUsed(0);
}

// --- Sampler thread: ISR-safe position sampling ---
// TIM_TMC ISR calls triggerFromIsr() which does NotifyFromISR().
// This wakes the thread, which calls Encoder::sampleNow() in thread
// context where blocking SPI (readReg → takeSemaphore) is legal.

TMC4671::TMC_SamplerThread::TMC_SamplerThread(TMC4671* tmc)
	: cpp_freertos::Thread("TMCSAMP", 80, 24), tmc(tmc) {
	this->Start();
}

void TMC4671::TMC_SamplerThread::triggerFromIsr() {
	// Pause sampling during encoder alignment — the alignment sequence
	// reads/writes TMC registers on the same SPI bus and the sampler's
	// interleaved reads would disrupt the timing/sequence.
	if (tmc->initialized && tmc->encoderAligned) this->NotifyFromISR();
}

void TMC4671::TMC_SamplerThread::Run() {
	while (true) {
		this->WaitForNotification();
		// Double-check: encoderAligned may have gone false between
		// triggerFromIsr and now (race is harmless — just one extra sample).
		if (!tmc->encoderAligned) continue;
		Encoder* enc = tmc->getEncoder();
		if (enc != nullptr) {
			if (!enc->isSamplerActive()) enc->activateSampler();
			enc->sampleNow();

		}
	}
}

const ClassIdentifier TMC4671::getInfo() {

	return info;
}


void TMC4671::setAddress(uint8_t address){
	if (address == 1){
		this->flashAddrs = TMC4671FlashAddrs({ADR_TMC1_MOTCONF, ADR_TMC1_CPR, ADR_TMC1_ENCA, ADR_TMC1_OFFSETFLUX, ADR_TMC1_TORQUE_P, ADR_TMC1_TORQUE_I, ADR_TMC1_FLUX_P, ADR_TMC1_FLUX_I,ADR_TMC1_ADC_I0_OFS,ADR_TMC1_ADC_I1_OFS,ADR_TMC1_ENC_OFFSET,ADR_TMC1_PHIE_OFS,ADR_TMC1_TRQ_FILT
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
			,ADR_TMC1_COGGING_CAL, ADR_TMC1_COGGING_SCALE, ADR_TMC1_COGGING_DYN_OFS, ADR_TMC1_SCALE_CURVE_BASE, ADR_TMC1_PHASEADV_CURVE_BASE, ADR_TMC1_H3_AMP, ADR_TMC1_H3_PHASE, ADR_TMC1_H3_ORDER, ADR_TMC1_COGGING_BLEND_RPM2, ADR_TMC1_COGGING_RPM2_VALID, ADR_TMC1_COGGING_BLEND_RPM3, ADR_TMC1_COGGING_RPM3_VALID
#endif
			});
	}else if (address == 2)
	{
		this->flashAddrs = TMC4671FlashAddrs({ADR_TMC2_MOTCONF, ADR_TMC2_CPR, ADR_TMC2_ENCA, ADR_TMC2_OFFSETFLUX, ADR_TMC2_TORQUE_P, ADR_TMC2_TORQUE_I, ADR_TMC2_FLUX_P, ADR_TMC2_FLUX_I,ADR_TMC2_ADC_I0_OFS,ADR_TMC2_ADC_I1_OFS,ADR_TMC2_ENC_OFFSET,ADR_TMC2_PHIE_OFS,ADR_TMC2_TRQ_FILT
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	,ADR_TMC2_COGGING_CAL, ADR_TMC2_COGGING_SCALE, ADR_TMC2_COGGING_DYN_OFS, ADR_TMC2_SCALE_CURVE_BASE, ADR_TMC2_PHASEADV_CURVE_BASE, ADR_TMC2_H3_AMP, ADR_TMC2_H3_PHASE, ADR_TMC2_H3_ORDER, ADR_TMC2_COGGING_BLEND_RPM2, ADR_TMC2_COGGING_RPM2_VALID, ADR_TMC2_COGGING_BLEND_RPM3, ADR_TMC2_COGGING_RPM3_VALID
#endif
		});
	}else if (address == 3)
	{
		this->flashAddrs = TMC4671FlashAddrs({ADR_TMC3_MOTCONF, ADR_TMC3_CPR, ADR_TMC3_ENCA, ADR_TMC3_OFFSETFLUX, ADR_TMC3_TORQUE_P, ADR_TMC3_TORQUE_I, ADR_TMC3_FLUX_P, ADR_TMC3_FLUX_I,ADR_TMC3_ADC_I0_OFS,ADR_TMC3_ADC_I1_OFS,ADR_TMC3_ENC_OFFSET,ADR_TMC3_PHIE_OFS,ADR_TMC3_TRQ_FILT
#ifdef COGGING_TABLE_FLASH_START_ADDRESS			
	,ADR_TMC3_COGGING_CAL, ADR_TMC3_COGGING_SCALE, ADR_TMC3_COGGING_DYN_OFS, ADR_TMC3_SCALE_CURVE_BASE, ADR_TMC3_PHASEADV_CURVE_BASE, ADR_TMC3_H3_AMP, ADR_TMC3_H3_PHASE, ADR_TMC3_H3_ORDER, ADR_TMC3_COGGING_BLEND_RPM2, ADR_TMC3_COGGING_RPM2_VALID, ADR_TMC3_COGGING_BLEND_RPM3, ADR_TMC3_COGGING_RPM3_VALID
#endif
		});
	}
	//this->setAxis((char)('W'+address));
}


void TMC4671::saveFlash(){
	uint16_t mconfint = TMC4671::encodeMotToInt(this->conf.motconf);
	uint16_t abncpr = this->conf.motconf.enctype == EncoderType_TMC::abn ? this->abnconf.cpr : this->aencconf.cpr;
	// Save flash
	Flash_Write(flashAddrs.mconf, mconfint);
	Flash_Write(flashAddrs.cpr, abncpr);
	Flash_Write(flashAddrs.offsetFlux,maxOffsetFlux);
	Flash_Write(flashAddrs.encA,encodeEncHallMisc());

	Flash_Write(flashAddrs.torque_p, curPids.torqueP);
	Flash_Write(flashAddrs.torque_i, curPids.torqueI);
	Flash_Write(flashAddrs.flux_p, curPids.fluxP);
	Flash_Write(flashAddrs.flux_i, curPids.fluxI);
	Flash_Write(flashAddrs.encOffset,(uint16_t)abnconf.posOffsetFromIndex);

	// If encoder is ABN and uses index save the last configured offset
//	if(this->conf.motconf.enctype == EncoderType_TMC::abn && this->abnconf.useIndex && encoderAligned){
//		Flash_Write(flashAddrs.phieOffset, abnconf.phiEoffset);
//	}

	uint16_t filterval = (torqueFilterConf.params.freq & 0x1fff) | ((uint8_t)(torqueFilterConf.mode) << 13);
	Flash_Write(flashAddrs.torqueFilter, filterval);

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	// Save cogging state
	uint16_t coggingFlash = (cogging_enabled ? 1 : 0);
	Flash_Write(flashAddrs.coggingEnable, coggingFlash);
	
	int16_t scale_int = clip<float>(this->cogging_scale * 10000.0f, -32767, 32767);
	Flash_Write(flashAddrs.coggingScale, (uint16_t)scale_int);

	// Save speed-dependent scale curve (24 points, *1000)
	if (scale_curve_valid) {
		for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
			int16_t curve_val = (int16_t)clip<float>(scale_curve_values[i] * 1000.0f, 0, 32767);
			Flash_Write(flashAddrs.scaleCurveBase + i, (uint16_t)curve_val);
		}
	}

	// Save velocity-based phase-advance curve (24 points, degrees * 100)
	if (phase_adv_curve_valid) {
		for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
			int16_t curve_val = (int16_t)clip<float>(phase_advance_curve_values[i] * 100.0f, -32767, 32767);
			Flash_Write(flashAddrs.phaseAdvCurveBase + i, (uint16_t)curve_val);
		}
	}

	// Save cogging waveshaping ("3rd harmonic" tab): shaping *1000, phase trim millirad, mult.
	Flash_Write(flashAddrs.h3Shaping, (uint16_t)(int16_t)clip<float>(this->h3_shaping * 1000.0f, -32767, 32767));
	Flash_Write(flashAddrs.h3PhaseTrim, (uint16_t)(int16_t)clip<float>(this->h3_phase_trim * 1000.0f, -32767, 32767));
	Flash_Write(flashAddrs.h3Mult, this->h3_mult);

	// Multi-RPM blend anchors (RPM*10) and validity flags.
	Flash_Write(flashAddrs.coggingBlendrpm2, (uint16_t)clip<float>(this->blend_rpm2 * 10.0f, 0.0f, 65535.0f));
	Flash_Write(flashAddrs.coggingrpm2Valid, this->rpm2_table_valid ? 1 : 0);
	Flash_Write(flashAddrs.coggingBlendrpm3, (uint16_t)clip<float>(this->blend_rpm3 * 10.0f, 0.0f, 65535.0f));
	Flash_Write(flashAddrs.coggingrpm3Valid, this->rpm3_table_valid ? 1 : 0);
#endif
}

/**
 * Writes ADC offsets into flash
 */
void TMC4671::saveAdcParams(){
	Flash_Write(flashAddrs.ADC_i0_ofs, conf.adc_I0_offset);
	Flash_Write(flashAddrs.ADC_i1_ofs, conf.adc_I1_offset);
	adcSettingsStored = true;
}

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
void TMC4671::saveCoggingTable(){
		Flash_WriteCoggingTable(this->drv_address - 1, (void*)this->cogging_harmonics);}

// RPM#2 and RPM#3 maps live in the second and third per-driver flash banks:
//   low  : index (drv_address-1) + 0*COGGING_DRIVER_COUNT   (slots 0..2)
//   rpm2 : index (drv_address-1) + 1*COGGING_DRIVER_COUNT   (slots 3..5)
//   rpm3 : index (drv_address-1) + 2*COGGING_DRIVER_COUNT   (slots 6..8)
void TMC4671::saveCoggingTableRpm2(){
		Flash_WriteCoggingTable((this->drv_address - 1) + 1*COGGING_DRIVER_COUNT, (void*)this->cogging_harmonics_rpm2);}

void TMC4671::saveCoggingTableRpm3(){
		Flash_WriteCoggingTable((this->drv_address - 1) + 2*COGGING_DRIVER_COUNT, (void*)this->cogging_harmonics_rpm3);}

void TMC4671::clearCoggingTable(){
	memset(cogging_harmonics, 0, sizeof(cogging_harmonics));
	cogging_enabled = false;
	saveCoggingTable();
}
#endif

/**
 * Restores saved parameters
 * Call initialize() to apply some of the settings
 */
void TMC4671::restoreFlash(){
	uint16_t mconfint;
	uint16_t abncpr = 0;

	// Read flash
	if(Flash_Read(flashAddrs.mconf, &mconfint))
		this->conf.motconf = TMC4671::decodeMotFromInt(mconfint);

	if(Flash_Read(flashAddrs.cpr, &abncpr))
		setCpr(abncpr);

	// Pids
	Flash_Read(flashAddrs.torque_p, &this->curPids.torqueP);
	Flash_Read(flashAddrs.torque_i, &this->curPids.torqueI);
	Flash_Read(flashAddrs.flux_p, &this->curPids.fluxP);
	Flash_Read(flashAddrs.flux_i, &this->curPids.fluxI);
	uint16_t encofs;
	Flash_Read(flashAddrs.encOffset,&encofs);
	this->abnconf.posOffsetFromIndex = (int16_t)encofs;

	Flash_Read(flashAddrs.offsetFlux, (uint16_t*)&this->maxOffsetFlux);

	// Restore ADC settings
	if(Flash_Read(flashAddrs.ADC_i0_ofs,&conf.adc_I0_offset) &&	Flash_Read(flashAddrs.ADC_i1_ofs,&conf.adc_I1_offset)){
		adcSettingsStored = true; // Previous adc settings restored
	}else{
		recalibrationRequired = true; // Never stored
	}

//	abnconf.phiEoffset = Flash_ReadDefault(flashAddrs.phieOffset,0);
//	if(abnconf.phiEoffset != 0){
//		phiErestored = true;
//	}

	uint16_t miscval;
	if(Flash_Read(flashAddrs.encA, &miscval)){
		restoreEncHallMisc(miscval);
		encHallRestored = true;
	}else{
		// set first hwconf if we can't restore
		this->setHwType(TMC4671::tmc4671_hw_configs[0].hwVersion);
	}
	uint16_t filterval;
	if(Flash_Read(flashAddrs.torqueFilter, &filterval)){
		torqueFilterConf.params.freq = filterval & 0x1fff;
		torqueFilterConf.mode = static_cast<TMCbiquadpreset>((filterval >> 13) & 0x7);
	}

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	uint16_t cogging = 0; // Initialize to avoid random stack data
	if(Flash_Read(flashAddrs.coggingEnable, &cogging)) {
		cogging_enabled = cogging & 0x01;
	} else {
		cogging_enabled = false;
	}
	
	uint16_t scale_flash = 0;
	if(Flash_Read(flashAddrs.coggingScale, &scale_flash)) {
		int16_t scale_int = (int16_t)scale_flash;
		this->cogging_scale = (float)scale_int / 10000.0f;
	} else {
		this->cogging_scale = 0.0f;
	}

	Flash_ReadCoggingTable(this->drv_address - 1, (int16_t*)this->cogging_harmonics);

	// Restore the RPM#2/RPM#3 maps, their validity flags and blend anchors so the
	// multi-RPM gain scheduling survives a reboot.
	Flash_ReadCoggingTable((this->drv_address - 1) + 1*COGGING_DRIVER_COUNT, (int16_t*)this->cogging_harmonics_rpm2);
	Flash_ReadCoggingTable((this->drv_address - 1) + 2*COGGING_DRIVER_COUNT, (int16_t*)this->cogging_harmonics_rpm3);
	uint16_t rpm2_valid_flash = 0;
	if (Flash_Read(flashAddrs.coggingrpm2Valid, &rpm2_valid_flash)) {
		this->rpm2_table_valid = (rpm2_valid_flash != 0);
	} else { this->rpm2_table_valid = false; }
	uint16_t rpm3_valid_flash = 0;
	if (Flash_Read(flashAddrs.coggingrpm3Valid, &rpm3_valid_flash)) {
		this->rpm3_table_valid = (rpm3_valid_flash != 0);
	} else { this->rpm3_table_valid = false; }
	uint16_t blend2_flash = 0;
	if (Flash_Read(flashAddrs.coggingBlendrpm2, &blend2_flash)) {
		this->blend_rpm2 = (float)(uint16_t)blend2_flash / 10.0f;
		if (this->blend_rpm2 < 1.0f) this->blend_rpm2 = 10.0f;
	} else { this->blend_rpm2 = 10.0f; }
	uint16_t blend3_flash = 0;
	if (Flash_Read(flashAddrs.coggingBlendrpm3, &blend3_flash)) {
		this->blend_rpm3 = (float)(uint16_t)blend3_flash / 10.0f;
		if (this->blend_rpm3 < 1.0f) this->blend_rpm3 = 20.0f;
	} else { this->blend_rpm3 = 20.0f; }
	// Populate calib RPM targets from persisted blend values so the configurator
	// remembers the last calibrated speeds on reboot.
	if (this->rpm2_table_valid && this->blend_rpm2 > 0.0f)
		this->cogging_calib_rpm[1] = this->blend_rpm2;
	if (this->rpm3_table_valid && this->blend_rpm3 > 0.0f)
		this->cogging_calib_rpm[2] = this->blend_rpm3;
	// Largest single harmonic amplitude as tanh reference.
	// Using sum-of-all or RMS overestimates, keeping tanh in linear zone.
	// Restore speed-dependent scale curve (24 points, *1000)
	scale_curve_valid = false;
	scale_curve_count = SCALE_CURVE_POINTS;
	uint16_t curve_flash = 0;
	bool any_valid = false;
	for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
		if (Flash_Read(flashAddrs.scaleCurveBase + i, &curve_flash)) {
			scale_curve_values[i] = (float)(int16_t)curve_flash / 1000.0f;
			any_valid = true;
		} else {
			scale_curve_values[i] = 0.0f;
		}
	}
	// Validate: index 1 is the first calibration point (e.g. 3 RPM); its scale
	// must be near 1.0.  Reject corrupt curves from old code.
	// Index 0 is the plateau and is clamped to 1.0 unconditionally.
	scale_curve_values[0] = 1.0f;
#ifndef COGGING_DISABLE_SCALE_CURVE
	if (any_valid && scale_curve_values[1] >= 1.0f)
		scale_curve_valid = true;
#endif

	// Restore velocity-based phase-advance curve (24 points, degrees * 100)
	phase_adv_curve_valid = false;
	uint16_t padv_flash = 0;
	bool padv_any_valid = false;
	for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
		if (Flash_Read(flashAddrs.phaseAdvCurveBase + i, &padv_flash)) {
			phase_advance_curve_values[i] = (float)(int16_t)padv_flash / 100.0f;
			padv_any_valid = true;
		} else {
			phase_advance_curve_values[i] = 0.0f;
		}
	}
	// Consider valid as soon as any point has ever been written to flash.
	// Index 0 is the plateau and must always be 0.0°.
	phase_advance_curve_values[0] = 0.0f;
	if (padv_any_valid)
		phase_adv_curve_valid = true;

	// Fill all 24 RPM breakpoints using linear interpolation between the
	// sparse calibrated points so the configurator shows a smooth curve.
	if (scale_curve_valid || phase_adv_curve_valid) {
		// Find indices of calibrated (non-zero) points.
		// When COGGING_DISABLE_SCALE_CURVE is defined, use phase values as anchors.
		uint8_t calib_idx[SCALE_CURVE_POINTS];
		uint8_t calib_n = 0;
		for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
#ifdef COGGING_DISABLE_SCALE_CURVE
			if (phase_advance_curve_values[i] != 0.0f || i == 0)
#else
			if (scale_curve_values[i] > 0.0f)
#endif
				calib_idx[calib_n++] = i;
		}
		// Interpolate every breakpoint from nearest calibrated neighbours
		if (calib_n >= 2) {
			uint8_t lo = 0;
			for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
				while (lo + 1 < calib_n && calib_idx[lo + 1] <= i)
					lo++;
				uint8_t hi = (lo + 1 < calib_n) ? lo + 1 : lo;
				if (hi == lo) {
					if (calib_n >= 2) {
						uint8_t a = calib_idx[calib_n - 2], b = calib_idx[calib_n - 1];
						float drpm_i = scale_curve_rpm_points[i] - scale_curve_rpm_points[b];
						float denom = scale_curve_rpm_points[b] - scale_curve_rpm_points[a];
						if (denom > 0.01f) {
#ifndef COGGING_DISABLE_SCALE_CURVE
							float ss = (scale_curve_values[b] - scale_curve_values[a]) / denom;
							scale_curve_values[i] = scale_curve_values[b] + ss * drpm_i;
#endif
							float ps = (phase_advance_curve_values[b] - phase_advance_curve_values[a]) / denom;
							phase_advance_curve_values[i] = phase_advance_curve_values[b] + ps * drpm_i;
						}
					}
				} else {
					float denom = scale_curve_rpm_points[calib_idx[hi]] - scale_curve_rpm_points[calib_idx[lo]];
					if (denom > 0.01f) {
						float t = (scale_curve_rpm_points[i] - scale_curve_rpm_points[calib_idx[lo]]) / denom;
#ifndef COGGING_DISABLE_SCALE_CURVE
						scale_curve_values[i] = scale_curve_values[calib_idx[lo]] + t * (scale_curve_values[calib_idx[hi]] - scale_curve_values[calib_idx[lo]]);
#endif
						phase_advance_curve_values[i] = phase_advance_curve_values[calib_idx[lo]] + t * (phase_advance_curve_values[calib_idx[hi]] - phase_advance_curve_values[calib_idx[lo]]);
					}
				}
			}
#ifdef COGGING_DISABLE_SCALE_CURVE
			for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++)
				scale_curve_values[i] = 1.0f;
#else
			scale_curve_values[0] = 1.0f;
#endif
			phase_advance_curve_values[0] = 0.0f;
		}
	}

	// Restore cogging waveshaping ("3rd harmonic" tab)
	uint16_t h3_flash = 0;
	if (Flash_Read(flashAddrs.h3Shaping, &h3_flash))
		this->h3_shaping = (float)(int16_t)h3_flash / 1000.0f;
	if (Flash_Read(flashAddrs.h3PhaseTrim, &h3_flash))
		this->h3_phase_trim = (float)(int16_t)h3_flash / 1000.0f;
	if (Flash_Read(flashAddrs.h3Mult, &h3_flash)) {
		uint16_t m = h3_flash;
		if (m >= 1 && m <= 31) this->h3_mult = m;
	}
#endif
}

bool TMC4671::hasPower(){
	int32_t intV = getIntV();
	return (intV > 10000) && (getExtV() > 10000) && (intV < 78000);
}

// Checks if important parameters are set to valid values
bool TMC4671::isSetUp(){

	if(this->conf.motconf.motor_type == MotorType::NONE ||!adcCalibrated || !initialized || !powerInitialized){
		return false;
	}

	// Encoder
	if(this->conf.motconf.phiEsource == PhiE::abn && abnconf.cpr == 0){
		return false;
	}
	if(this->conf.motconf.phiEsource == PhiE::abn || this->conf.motconf.phiEsource == PhiE::aenc){
		if(!encoderAligned){
			return false;
		}
	}
	if(this->conf.motconf.phiEsource == PhiE::ext && drvEncoder->getEncoderType() == EncoderType::NONE && !encoderAligned){
		return false;
	}

	return true;
}

/**
 * Check if driver is responding
 */
bool TMC4671::pingDriver(){
	writeReg(1, 0);
	return(readReg(0) == 0x34363731);
}

/**
 * Returns estimated VM in mV measured by TMC
 */
int32_t TMC4671::getTmcVM(){
	writeReg(0x03, 1); // adc raw data to VM/agpiA
	uint32_t agpiA_VM = readReg(0x02);
	agpiA_VM = (agpiA_VM & 0xFFFF) - 0x7FFF - conf.hwconf.adcOffset;

	return ((float)agpiA_VM * conf.hwconf.vmScaler) * 1000;
}

void TMC4671::setupDriver() {
	// Configuration specific to TMC4671 upon starting the motor
	// The power limit is set by the Axis class via setPowerLimit during initialization.

	this->setExternalEncoderAllowed(true);
	this->restoreFlash();
	this->setLimits(curLimits);

	// Start driver
	this->setMotionMode(MotionMode::torque);
	this->Start(); // Start thread
}

/**
 * Sets all parameters of the driver at startup. Only has to be called once when the driver is detected
 * restoreFlash() should be called before this to restore settings!
 */
bool TMC4671::initialize(){
//	active = true;
//	if(state == TMC_ControlState::uninitialized){
//		state = TMC_ControlState::Init_wait;
//	}
	// Check if a TMC4671 is active and replies correctly
	if(!pingDriver()){
		ErrorHandler::addError(COMMUNICATION_ERROR);
		return false;
	}

	writeReg(1, 1);
	if(readReg(0) == 0x00010000 && allowSlowSPI){
		/* Slow down SPI if old TMC engineering sample is detected
		 * The first version has a high chance of glitches of the MSB
		 * when high spi speeds are used.
		 * This can cause problems for some operations.
		 */
		pulseClipLed();

		this->spiConfig.peripheral.BaudRatePrescaler = spiPort.getClosestPrescaler(1e6).first; // 1MHz target
		spiPort.configurePort(&this->spiConfig.peripheral);
		ES_TMCdetected = true;
	}

	if(!ES_TMCdetected){
		this->setPidPrecision(pidPrecision);
	}

	// Detect if tmc was previously uninitialized
	if(startupType == TMC_StartupType::NONE){
		if(getMotionMode() != MotionMode::stop){
			startupType = TMC_StartupType::warmStart;
		}else{
			startupType = TMC_StartupType::coldStart;
		}
	}

	// Write main constants

	writeReg(0x64, 0); // No flux/torque
	setPwm(0,conf.pwmcnt,conf.bbmL,conf.bbmH); // Set FOC @ 50khz but turn off pwm for now
	setMotorType(conf.motconf.motor_type,conf.motconf.pole_pairs);
	setPhiEtype(conf.motconf.phiEsource);
	setup_HALL(hallconf); // Enables hall filter and masking

	initAdc(conf.mdecA,conf.mdecB,conf.mclkA,conf.mclkB);
	setAdcOffset(conf.adc_I0_offset, conf.adc_I1_offset);
	setAdcScale(conf.adc_I0_scale, conf.adc_I1_scale);
	setTorqueFilter(torqueFilterConf);
	setBiquadFlux(TMC4671Biquad(Biquad(BiquadType::lowpass, FLUX_FILTER_FREQ / getPwmFreq(), 0.7,0.0), true)); // Create flux filter

	// Initial adc calibration and check without PWM if power off to get basic offsets. PWM is off!
	if(!hasPower()){
		if(!calibrateAdcOffset(150)){
			changeState(TMC_ControlState::HardError); // ADC or shunt amp is broken!
			enablePin.reset();
			return false;
		}
	}
	// brake res failsafe.
//	/*
//	 * Single ended input raw value
//	 * 0V = 0x7fff
//	 * 4.7k / (360k+4.7k) Divider on old board.
//	 * 1.5k / (71.5k+1.5k) 16.121 counts 60V new. 100V VM => 2V
//	 * 13106 counts/V input.
//	 */
	setBrakeLimits(this->conf.hwconf.brakeLimLow,this->conf.hwconf.brakeLimHigh); // update limit from previously loaded constants or defaults

	// Status mask
	if(ES_TMCdetected){
		setStatusMask(0); // ES Version status output is broken
	}else{
		/*
		 * Enable adc clipping and pll errors
		 */
		statusMask.asInt = 0;
		statusMask.flags.adc_i_clipped = 1;
		statusMask.flags.not_PLL_locked = 1;
		setStatusMask(statusMask);
	}

	setPids(curPids); // Write basic pids

//	if(hasPower()){
//		enablePin.set();
//		setPwm(TMC_PwmMode::PWM_FOC);
//		calibrateAdcOffset(400); // Calibrate ADC again with power
//		motorEnabledRequested = true;
//	}
	//setEncoderType(conf.motconf.enctype);

	// Update flags
	readFlags(false); // Read all flags

	initialized = true;
	initTime = HAL_GetTick();
	return initialized;
}

/**
 * Reads a temperature from a thermistor connected to AGPI_B
 * Not calibrated perfectly!
 */
float TMC4671::getTemp(){
	if(!this->conf.hwconf.thermistorSettings.temperatureEnabled){
		return 0;
	}
	TMC4671HardwareTypeConf* hwconf = &conf.hwconf;

	writeReg(0x03, 2);
	int32_t adcval = ((readReg(0x02)) & 0xffff) - 0x7fff; // Center offset
	adcval -= hwconf->adcOffset;
	if(adcval <= 0){
		return 0.0;
	}
	float r = hwconf->thermistorSettings.thermistor_R2 * (((float)43252 / (float)adcval)); //43252 equivalent ADC count if it was 3.3V and not 2.5V

	// Beta
	r = (1.0 / 298.15) + log(r / hwconf->thermistorSettings.thermistor_R) / hwconf->thermistorSettings.thermistor_Beta;
	r = 1.0 / r;
	r -= 273.15;
	return r;

}

/**
 * Samples the adc and checks if it returns a neutral value
 */
bool TMC4671::checkAdc(){
	setFluxTorque(0, 0);
	int32_t total = 0;
	for(uint8_t i = 0;i<50;i++){
		std::pair<int32_t,int32_t> ft = getActualTorqueFlux();
		total += (ft.first+ft.second);
		Delay(2);
	}
	return(abs(total / 50) < 100); // Check if average has a low bias
}

void TMC4671::initializeWithPower(){
	if(powerInitialized){
		return;
	}
	powerInitialized = true;
	// Load ADC settings
	if(Flash_Read(flashAddrs.ADC_i0_ofs,&conf.adc_I0_offset) &&	Flash_Read(flashAddrs.ADC_i1_ofs,&conf.adc_I1_offset)){
		adcSettingsStored = true; // Previous adc settings restored
		setAdcOffset(conf.adc_I0_offset, conf.adc_I1_offset);
	}

	if(adcSettingsStored && checkAdc()){
		adcCalibrated = true;
	}else{
		if(!calibrateAdcOffset(300)){
			powerInitialized = false;
			return; // Abort
		}
	}

	// got power long enough. proceed to set up encoder
	// if encoder not set up
	enablePin.set();
	setPwm(TMC_PwmMode::PWM_FOC); // enable foc
	if(!encoderAligned){
		setEncoderType(conf.motconf.enctype);
	}else{
		//last state
		if(!emergency){
			allowStateChange = true;
			changeState(requestedState);
			setMotionMode(lastMotionMode,true);
			ErrorHandler::clearError(ErrorCode::undervoltage);
		}
	}

}

bool TMC4671::motorReady(){
	return this->state == TMC_ControlState::Running && powerInitialized && adcCalibrated && encoderAligned;
}

void TMC4671::Run(){
	// Main state machine
	while(1){

		if(!initialized && state != TMC_ControlState::HardError){
			changeState(TMC_ControlState::uninitialized,true);
		}

		// check if we are in a privileged state otherwise use requested state as new state
		if(allowStateChange){
			state = requestedState;
		}

		switch(this->state){

		case TMC_ControlState::uninitialized:
			allowStateChange = false;
			// check communication and write constants
			if(!pingDriver() || emergency){ // driver not available or emergency was set before startup
				initialized = false; // Assume driver is not initialized if we can not detect it
				Delay(250);
				break;
			}
			// Driver available. Write constants and go to next step
			if(!initialized){
				initialize();
			}
			changeState(TMC_ControlState::waitPower,true);
			break;

		case TMC_ControlState::waitPower:
			handleStateWaitPower();
			break;

		case TMC_ControlState::FullCalibration:
			handleStateFullCalibration();
			break;

		case TMC_ControlState::Pidautotune:
			handleStatePidAutoTune();
			break;

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
		case TMC_ControlState::CoggingCalibration:
			handleStateCoggingCalibration();
			break;
#endif
		case TMC_ControlState::IndexSearch:
			autohome();
			changeState(laststate);

			break;

		case TMC_ControlState::Running:
			handleStateRunning();
			break;

		case TMC_ControlState::Shutdown:
			Delay(100);
			if(estopTriggered){
				uint32_t pins = readReg(0x76);
				bool tmc_en = ((pins >> 15) & 0x01) && pins != 0xffffffff;
				if(tmc_en){
					// Emergency stop reset
					ErrorHandler::clearError(ESTOP_ERROR);
					this->estopTriggered = false; // TODO resume correctly
					changeState(TMC_ControlState::uninitialized,true);
				}
			}
			break;

		case TMC_ControlState::EncoderInit:
			if(powerInitialized && hasPower() && conf.motconf.motor_type != MotorType::NONE)
				encoderInit();
		break;

		case TMC_ControlState::ExternalEncoderInit:
			if(powerInitialized && hasPower() && drvEncoder != nullptr && conf.motconf.motor_type != MotorType::NONE)
				encoderInit();
			break;

		case TMC_ControlState::HardError:

		break; // Broken

		case TMC_ControlState::OverTemp:
			this->stopMotor();
			changeState(TMC_ControlState::HardError); // Block
		break;

		case TMC_ControlState::EncoderFinished: // Startup sequence done
			//setEncoderIndexFlagEnabled(false); // TODO
//			curFilters.flux.params.enable = true;
//			setBiquadFlux(curFilters.flux); // Enable flux filter
			encoderAligned = true;
			if(motorEnabledRequested && isSetUp()){
				startMotor();
				changeState(TMC_ControlState::Running);
			}else{
				stopMotor();
				laststate = TMC_ControlState::Running; // Go to running when starting again
			}

			if(fullCalibrationInProgress){
				Flash_Write(flashAddrs.encA,encodeEncHallMisc()); // Save encoder settings
			}


		break;

		default:

		break;

		}


		// Optional update methods for safety

		if(!hasPower() && state != TMC_ControlState::waitPower && state != TMC_ControlState::CoggingCalibration && initialized && powerInitialized){ // low voltage or overvoltage

			requestedState = state;
			ErrorHandler::addError(LOW_VOLTAGE_ERROR);
			setMotionMode(MotionMode::stop,true); // Disable tmc
			changeState(TMC_ControlState::waitPower,true);
			allowStateChange = false;
		}

		if(flagCheckInProgress){ // cause some delay until reenabling the status interrupt checking
			setStatusFlags(0);
			flagCheckInProgress = false;
		}
		Delay(10);

		if(emergency && !motorReady()){
			this->Suspend(); // we can not safely run. wait until resumed by estop
		}
	} // End while
}
void TMC4671::calibrateEncoder(){
	if(conf.motconf.enctype == EncoderType_TMC::abn) {
		estimateABNparams();
		// Report changes
		CommandHandler::broadcastCommandReply(CommandReply(abnconf.npol ? 1 : 0), (uint32_t)TMC4671_commands::encpol, CMDtype::get);
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		if(!conf.hwconf.flags.analog_enc_skip_cal){
			calibrateAenc();
		}
	}else if(conf.motconf.enctype == EncoderType_TMC::ext){
		estimateExtEnc();
	}
	changeState(TMC_ControlState::EncoderInit);


}

bool TMC4671::autohome(){
	// Moves motor to index
	if(findEncoderIndex(abnconf.posOffsetFromIndex < 0 ? 10 : -10,bangInitPower/2,false,false)){
		// Load position offset
		if(abnconf.useIndex)
			setTmcPos(getPosAbs() - abnconf.posOffsetFromIndex);
		return true;
	}
	return false;
}

/*
 * Returns the current state of the driver controller
 */
TMC_ControlState TMC4671::getState(){
	return this->state;
}

inline void TMC4671::changeState(TMC_ControlState newState,bool force){
	if(newState != this->state){
		this->laststate = this->state; // save last state if new state wants to jump back
	}
	if(!force){
		this->requestedState = newState;
	}else{
		state = newState;
	}

}

bool TMC4671::reachedPosition(uint16_t tolerance){
	int32_t actualPos = readReg(0x6B);
	int32_t targetPos = readReg(0x68);
	if( abs(targetPos - actualPos) < tolerance){
		return true;
	}else{
		return false;
	}
}

void TMC4671::zeroAbnUsingPhiM(bool offsetPhiE){
	int32_t npos = (int32_t)readReg(0x28); // raw encoder counts at index hit
	int32_t npos_M = (npos * 0xffff) / abnconf.cpr; // Scaled encoder angle at index
	abnconf.phiMoffset = -npos_M;
	if(offsetPhiE){
		abnconf.phiEoffset += npos_M*conf.motconf.pole_pairs;
		// change index to zero phiM
		uint32_t phiEphiM = readReg(0x29);
		int16_t phiE = ((phiEphiM >> 16) & 0xffff); // Write back phiE offset
		int16_t phiM = phiEphiM & 0xffff;
		//updateReg(0x29, abnconf.phiMoffset, 0xffff, 0);
		writeReg(0x29,(phiE << 16) | phiM);
	}else{
		updateReg(0x29, abnconf.phiMoffset, 0xffff, 0);
	}
	setTmcPos(getPosAbs()); // Set position to absolute position = ~zero
}

/**
 * Rotates motor until the ABN index is found
 */
bool TMC4671::findEncoderIndex(int32_t speed, uint16_t power,bool offsetPhiM,bool zeroCount){

	if(conf.motconf.enctype != EncoderType_TMC::abn){
		return false; // Only valid for ABN encoders
	}

	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	curFilters.flux.params.enable = false;
	setBiquadFlux(curFilters.flux);
	setFluxTorque(0, 0);
//	setPhiE_ext(getPhiE());
//	setPhiEtype(PhiE::openloop);

//	abnconf.clear_on_N = true;
//	setup_ABN_Enc(abnconf);

	// Arm encoder signal
	setEncoderIndexFlagEnabled(true,zeroCount);
	// Rotate

	//uint32_t mposStart = readReg(0x2A);
	int32_t timeout = 1000; // 10s
	rampFlux(power, 500);
	runOpenLoop(power, 0, speed, 10, true);
	while(!encoderIndexHitFlag && timeout-- > 0){
		Delay(10);
	}
	//int32_t speed = 10;
	rampFlux(0, 100);
	runOpenLoop(0, 0, 0, 10, true);
	if(!encoderIndexHitFlag){
		pulseErrLed();
		ErrorHandler::addError(INDEX_NOT_HIT_ERROR);
	}

	// If zero count on index write a phiM offset so that phiM is 0 on index and we don't need to change the raw encoder count (possible timing danger)
	if(offsetPhiM){
		zeroAbnUsingPhiM(false);
	}

//	abnconf.clear_on_N = false;
//	setup_ABN_Enc(abnconf);
	curFilters.flux.params.enable = true;
	setBiquadFlux(curFilters.flux);

	setMotionMode(lastmode,true);
	setPhiEtype(lastphie);
	return encoderIndexHitFlag;
}

/**
 * Enables or disables the encoder index interruption on the flag pin depending on the selected encoder
 */
void TMC4671::setEncoderIndexFlagEnabled(bool enabled,bool zeroEncoder){
	//zeroEncoderOnIndexHit = zeroEncoder;

	updateReg(0x25, zeroEncoder ? 1 : 0, 0x1, 9); // Enable encoder clearing
	if(zeroEncoder){
		writeReg(0x28,0); // Preload 0 into n register
	}

	if(enabled)
		encoderIndexHitFlag = false;
	setStatusFlags(0); // Reset flags
	this->statusMask.flags.AENC_N = this->conf.motconf.enctype == EncoderType_TMC::sincos && enabled;
	this->statusMask.flags.ENC_N = this->conf.motconf.enctype == EncoderType_TMC::abn && enabled;
	setStatusMask(statusMask); // Enable flag output for encoder
}

/**
 * Enables position mode and sets a target position
 */
void TMC4671::setTargetPos(int32_t pos){
	if(curMotionMode != MotionMode::position){
		setMotionMode(MotionMode::position,true);
		setPhiEtype(this->conf.motconf.phiEsource);
	}
	writeReg(0x68,pos);
}
int32_t TMC4671::getTargetPos(){

	return readReg(0x68);
}


/**
 * Enables velocity mode and sets a velocity target
 */
void TMC4671::setTargetVelocity(int32_t vel){
	if(curMotionMode != MotionMode::velocity){
		setMotionMode(MotionMode::velocity,true);
		setPhiEtype(this->conf.motconf.phiEsource);
	}
	writeReg(0x66,vel);
}
int32_t TMC4671::getTargetVelocity(){
	return readReg(0x66);
}
int32_t TMC4671::getVelocity(){
	return readReg(0x6A);
}

void TMC4671::setPositionExt(int32_t pos){
	writeReg(0x1E, pos);
}

void TMC4671::setPhiE_ext(int16_t phiE){
	writeReg(0x1C, phiE);
}

int16_t TMC4671::getPhiEfromExternalEncoder(){
	int64_t phiE_t = (int64_t)drvEncoder->getPosAbs() * 0xffff;
	if(this->conf.encoderReversed){
		phiE_t = -phiE_t;
	}
	int32_t phiE = (phiE_t / (int64_t)drvEncoder->getCpr());
	phiE = (phiE * conf.motconf.pole_pairs) & 0xffff; // scale to pole pairs
	//int16_t phiE = (drvEncoder->getPosAbs_f() * (float)0xffff) * conf.motconf.pole_pairs + externalEncoderPhieOffset;
	return(phiE+externalEncoderPhieOffset);
}

// PhiE is read only
int16_t TMC4671::getPhiE(){
	return readReg(0x53);
}



/**
 * Aligns ABN encoders by forcing an angle with high current and calculating the offset
 * Will start at the current phiE to minimize any extra movements (useful if motor was turned in openloop mode before already)
 * @param power Maximum current reached during flux ramp
 */
void TMC4671::bangInitEnc(int16_t power){
	if(!hasPower() || (this->conf.motconf.motor_type != MotorType::STEPPER && this->conf.motconf.motor_type != MotorType::BLDC)){ // If not stepper or bldc return
		return;
	}
	blinkClipLed(50, 0);
	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	setFluxTorque(0, 0);

	uint8_t phiEoffsetReg = 0;
	if(conf.motconf.enctype == EncoderType_TMC::abn){
		phiEoffsetReg = 0x29;
		if(!encoderIndexHitFlag)
			zeroAbnUsingPhiM();
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		writeReg(0x41,0); //Zero encoder
		writeReg(0x47,0); //Zero encoder
		phiEoffsetReg = 0x45;
	}else if (usingExternalEncoder()){
		externalEncoderPhieOffset = 0;
	}else{
		return; // Not relevant
	}

	//setTmcPos(0);

	//setMotionMode(MotionMode::uqudext);

	//Delay(100);
	int16_t phiEpos = getPhiE();// readReg(phiEreg)>>16; // starts at current encoder position
	updateReg(phiEoffsetReg, 0, 0xffff, 16); // Set phiE offset to zero
	setPhiE_ext(phiEpos);
	setPhiEtype(PhiE::ext);
	// Ramp up flux
	rampFlux(power, 1000);
	int16_t phiE_enc = getPhiE_Enc();

	Delay(50);
	int16_t phiE_abn_old = 0;
	int16_t c = 0;
	uint16_t still = 0;
	while(still < 30 && c++ < 1000){
		// Wait for motor to stop moving
		if(abs(phiE_enc - phiE_abn_old) < 100){
			still++;
		}else{
			still = 0;
		}
		phiE_abn_old = phiE_enc;

		phiE_enc = getPhiE_Enc();

		//phiE_enc=readReg(phiEreg)>>16;
		Delay(10);
	}
	rampFlux(0, 100);

	//Write offset
	//int16_t phiE_abn = readReg(0x2A)>>16;
	int16_t phiEoffset =  phiEpos-phiE_enc;

	if(phiEoffset == 0){ // 0 invalid
		phiEoffset = 1;
	}
	if (usingExternalEncoder()){
		externalEncoderPhieOffset = phiEoffset;
	}else{
		updateReg(phiEoffsetReg, phiEoffset, 0xffff, 16);
	}

	if(conf.motconf.enctype == EncoderType_TMC::abn){
		abnconf.phiEoffset = phiEoffset;
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		aencconf.phiEoffset = phiEoffset;
	}


	setPhiE_ext(0);
	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);
	//setTmcPos(pos+getPos());
	//setTmcPos(0);

	blinkClipLed(0, 0);
}

/**
 * Moves the motor to find out analog encoder scalings and offsets
 */
void TMC4671::calibrateAenc(){

	// Rotate and measure min/max
	blinkClipLed(250, 0);
	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	//int32_t pos = getPos();
	PosSelection possel = this->conf.motconf.pos_sel;
	setPosSel(PosSelection::PhiE_openloop);
	setTmcPos(0);
	// Ramp up flux
	setFluxTorque(0, 0);
	writeReg(0x23,0); // set phie openloop 0
	setPhiEtype(PhiE::openloop);
	setMotionMode(MotionMode::torque,true);

	if(this->conf.motconf.motor_type == MotorType::STEPPER || this->conf.motconf.motor_type == MotorType::BLDC){
		rampFlux(bangInitPower, 250);
	}
	uint32_t minVal_0 = 0xffff,	minVal_1 = 0xffff,	minVal_2 = 0xffff;
	uint32_t maxVal_0 = 0,	maxVal_1 = 0,	maxVal_2 = 0;
	int32_t minpos = -0x8fff/std::max<int32_t>(1,std::min<int32_t>(this->aencconf.cpr/4,20)), maxpos = 0x8fff/std::max<int32_t>(1,std::min<int32_t>(this->aencconf.cpr/4,20));
	uint32_t speed = std::max<uint32_t>(1,20/std::max<uint32_t>(1,this->aencconf.cpr/10));

	if(this->conf.motconf.motor_type != MotorType::STEPPER && this->conf.motconf.motor_type != MotorType::BLDC){
		speed*=10; // dc motors turn at a random speed. reduce the rotation time a bit by increasing openloop speed
	}

	runOpenLoop(bangInitPower, 0, speed, 100,true);

	uint8_t stage = 0;
	int32_t poles = conf.motconf.pole_pairs;
	int32_t initialDirPos = 0;
	while(stage != 3){
		Delay(2);
		if(getPos() > maxpos*poles && stage == 0){
			runOpenLoop(bangInitPower, 0, -speed, 100,true);
			stage = 1;
		}else if(getPos() < minpos*poles && stage == 1){
			// Scale might still be wrong... maxVal-minVal is too high. In theory its 0xffff range and scaler /256. Leave some room to prevent clipping
			aencconf.AENC0_offset = ((maxVal_0 + minVal_0) / 2);
			aencconf.AENC0_scale = 0xF6FF00 / (maxVal_0 - minVal_0);
			if(conf.motconf.enctype == EncoderType_TMC::uvw){
				aencconf.AENC1_offset = ((maxVal_1 + minVal_1) / 2);
				aencconf.AENC1_scale = 0xF6FF00 / (maxVal_1 - minVal_1);
			}

			aencconf.AENC2_offset = ((maxVal_2 + minVal_2) / 2);
			aencconf.AENC2_scale = 0xF6FF00 / (maxVal_2 - minVal_2);
			aencconf.rdir = false;
			setup_AENC(aencconf);
			rampFlux(0, 100);
			runOpenLoop(0, 0, 0, 1000,true);
			Delay(250);
			// Zero aenc
			writeReg(0x41, 0);
			initialDirPos = readReg(0x41);
			runOpenLoop(bangInitPower, 0, speed, 100,true);
			stage = 2;
		}else if(getPos() > 0 && stage == 2){
			stage = 3;
			rampFlux(0, 100);
			runOpenLoop(0, 0, 0, 1000,true);
		}

		writeReg(0x03,2);
		uint32_t aencUX = readReg(0x02)>>16;
		writeReg(0x03,3);
		uint32_t aencWY_VN = readReg(0x02) ;
		uint32_t aencWY = aencWY_VN >> 16;
		uint32_t aencVN = aencWY_VN & 0xffff;

		minVal_0 = std::min(minVal_0,aencUX);
		minVal_1 = std::min(minVal_1,aencVN);
		minVal_2 = std::min(minVal_2,aencWY);

		maxVal_0 = std::max(maxVal_0,aencUX);
		maxVal_1 = std::max(maxVal_1,aencVN);
		maxVal_2 = std::max(maxVal_2,aencWY);
	}
	// Scale is not actually important. but offset must be perfect
	aencconf.AENC0_offset = ((maxVal_0 + minVal_0) / 2);
	aencconf.AENC0_scale = 0xF6FF00 / (maxVal_0 - minVal_0);
	if(conf.motconf.enctype == EncoderType_TMC::uvw){
		aencconf.AENC1_offset = ((maxVal_1 + minVal_1) / 2);
		aencconf.AENC1_scale = 0xF6FF00 / (maxVal_1 - minVal_1);
	}
	aencconf.AENC2_offset = ((maxVal_2 + minVal_2) / 2);
	aencconf.AENC2_scale = 0xF6FF00 / (maxVal_2 - minVal_2);
	int32_t newDirPos = readReg(0x41);
	aencconf.rdir =  (initialDirPos - newDirPos) > 0;
	setup_AENC(aencconf);
	// Restore settings
	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);
	setPosSel(possel);
	setTmcPos(0);

	blinkClipLed(0, 0);
}

/**
 * Reads phiE directly from the encoder selection instead of the current phiE selection
 */
int16_t TMC4671::getPhiE_Enc(){
	if(conf.motconf.enctype == EncoderType_TMC::abn){
		return (int16_t)(readReg(0x2A)>>16);
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		return (int16_t)(readReg(0x46)>>16);
	}else if(conf.motconf.enctype == EncoderType_TMC::hall){
		return (int16_t)(readReg(0x39)>>16);
	}else if(usingExternalEncoder()){
		return getPhiEfromExternalEncoder();
	}else{
		return getPhiE();
	}
}

/**
 * Steps the motor a few times to check if the encoder follows correctly
 */
bool TMC4671::checkEncoder(){
	if((this->conf.motconf.motor_type != MotorType::STEPPER && this->conf.motconf.motor_type != MotorType::BLDC) || (
			conf.motconf.enctype != EncoderType_TMC::uvw && conf.motconf.enctype != EncoderType_TMC::sincos && conf.motconf.enctype != EncoderType_TMC::abn && conf.motconf.enctype != EncoderType_TMC::ext))
	{ // If not stepper or bldc return
		return true;
	}
	blinkClipLed(150, 0);

	const uint16_t maxcount = 50; // Allowed reversals
	const uint16_t maxfail = 10; // Allowed fails
	const int16_t startAngle = getPhiE_Enc(); // Start angle offsets all angles later so there is no jump if angle is already properly aligned
	const int16_t targetAngle = 0x3FFF;

	bool result = true;
	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	setFluxTorque(0, 0);
	setPhiEtype(PhiE::ext);

	setPhiE_ext(startAngle);
	// Ramp up flux
	rampFlux(2*bangInitPower/3, 250);

	//Forward
	int16_t phiE_enc = 0;
	uint16_t failcount = 0;
	int16_t revCount = 0;
	for(int16_t angle = 0;angle<targetAngle;angle+=0x00ff){
		uint16_t c = 0;
		setPhiE_ext(angle+startAngle);
		Delay(5);
		phiE_enc = getPhiE_Enc() - startAngle;
		int16_t err = abs(phiE_enc - angle);
		int16_t nErr = abs(phiE_enc + angle);
		// Wait more until encoder settles a bit
		while(err > 2000 && nErr > 2000 && c++ < 50){
			phiE_enc = getPhiE_Enc() - startAngle;
			err = abs(phiE_enc - angle);
			nErr = abs(angle - phiE_enc);
			Delay(10);
		}
		if(err > nErr){
			revCount++;
		}
		if(c >= maxcount){
			failcount++;
			if(failcount > maxfail){
				result = false;
				break;
			}
		}
	}
	/* If we are still at the start angle the encoder did not move at all.
	 * Possible issues:
	 * Encoder connection wrong
	 * Wrong encoder selection
	 * No motor movement
	 * No encoder power
	 */
	if(startAngle == getPhiE_Enc()){
		ErrorHandler::addError(Error(ErrorCode::encoderAlignmentFailed,ErrorType::critical,"Encoder did not move during alignment"));
		this->changeState(TMC_ControlState::HardError, true);
		result = false;
	}

	// Backward

	if(result){ // Only if not already failed
		for(int16_t angle = targetAngle;angle>0;angle -= 0x00ff){
			uint16_t c = 0;
			setPhiE_ext(angle+startAngle);
			Delay(5);
			phiE_enc = getPhiE_Enc() - startAngle;
			int16_t err = abs(phiE_enc - angle);
			int16_t nErr = abs(phiE_enc + angle);
			// Wait more
			while(err > 2500 && nErr > 2500 && c++ < 50){
				phiE_enc = getPhiE_Enc() - startAngle;
				err = abs(phiE_enc - angle);
				nErr = abs(angle - phiE_enc);
				Delay(10);
			}
			if(err > nErr){
				revCount++;
			}
			if(c >= maxcount){
				failcount++;
				if(failcount > maxfail){
					result = false;
					break;
				}
			}
		}
	}

	// TODO check if we want that
	if(revCount > maxcount){ // Encoder seems reversed
		// reverse encoder
		if(this->conf.motconf.enctype == EncoderType_TMC::abn){
			this->abnconf.rdir = !this->abnconf.rdir;
			setup_ABN_Enc(abnconf);
		}else if(this->conf.motconf.enctype == EncoderType_TMC::sincos || this->conf.motconf.enctype == EncoderType_TMC::uvw){
			this->aencconf.rdir = !this->aencconf.rdir;
			setup_AENC(aencconf);
		}else if(this->conf.motconf.enctype == EncoderType_TMC::ext){
			this->conf.encoderReversed = !this->conf.encoderReversed;
		}
		ErrorHandler::addError(Error(ErrorCode::encoderReversed,ErrorType::warning,"Encoder direction reversed during check"));
	}

	rampFlux(0, 100);
	setPhiE_ext(0);
	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);

	if(result){
		encoderAligned = true;
	}
	blinkClipLed(0, 0);
	return result;
}

void TMC4671::setup_ABN_Enc(TMC4671ABNConf encconf){
	this->abnconf = encconf;
	this->conf.encoderReversed = encconf.rdir;
	uint32_t abnmode =
			(encconf.apol |
			(encconf.bpol << 1) |
			(encconf.npol << 2) |
			(encconf.ab_as_n << 3) |
			(encconf.latch_on_N << 8) |
			(encconf.rdir << 12));

	writeReg(0x25, abnmode);
	//int32_t pos = getPos();
	writeReg(0x26, encconf.cpr);
	writeReg(0x29, ((uint16_t)encconf.phiEoffset << 16) | (uint16_t)encconf.phiMoffset);
	//setTmcPos(pos);
	//writeReg(0x27,0); //Zero encoder
	//conf.motconf.phiEsource = PhiE::abn;
	if(encconf.useIndex){
		encoderIndexHitFlag = false; // Reset flag
	}


}
void TMC4671::setup_AENC(TMC4671AENCConf encconf){
	this->conf.encoderReversed = encconf.rdir;
	// offsets
	writeReg(0x0D,encconf.AENC0_offset | ((uint16_t)encconf.AENC0_scale << 16));
	writeReg(0x0E,encconf.AENC1_offset | ((uint16_t)encconf.AENC1_scale << 16));
	writeReg(0x0F,encconf.AENC2_offset | ((uint16_t)encconf.AENC2_scale << 16));

	writeReg(0x40,encconf.cpr);
	writeReg(0x3e,(uint16_t)encconf.phiAoffset);
	writeReg(0x45,(uint16_t)encconf.phiEoffset | ((uint16_t)encconf.phiMoffset << 16));
	writeReg(0x3c,(uint16_t)encconf.nThreshold | ((uint16_t)encconf.nMask << 16));

	uint32_t mode = encconf.uvwmode & 0x1;
	mode |= (encconf.rdir & 0x1) << 12;
	writeReg(0x3b, mode);

}
void TMC4671::setup_HALL(TMC4671HALLConf hallconf){
	this->hallconf = hallconf;

	uint32_t hallmode =
			hallconf.polarity |
			hallconf.filter << 4 |
			hallconf.interpolation << 8 |
			hallconf.direction << 12 |
			(hallconf.blank & 0xfff) << 16;
	writeReg(0x33, hallmode);
	// Positions
	uint32_t posA = (uint16_t)hallconf.pos0 | (uint16_t)hallconf.pos60 << 16;
	writeReg(0x34, posA);
	uint32_t posB = (uint16_t)hallconf.pos120 | (uint16_t)hallconf.pos180 << 16;
	writeReg(0x35, posB);
	uint32_t posC = (uint16_t)hallconf.pos240 | (uint16_t)hallconf.pos300 << 16;
	writeReg(0x36, posC);

	uint32_t phiOffsets = (uint16_t)hallconf.phiMoffset | (uint16_t)hallconf.phiEoffset << 16;
	writeReg(0x37, phiOffsets);
	writeReg(0x38, hallconf.dPhiMax);

	//conf.motconf.phiEsource = PhiE::hall;
}


/**
 * Calibrates the ADC by disabling the power stage and sampling a mean value. Takes time!
 */
bool TMC4671::calibrateAdcOffset(uint16_t time){

	uint16_t measuretime_idle = time;
	uint32_t measurements_idle = 0;
	uint64_t totalA=0;
	uint64_t totalB=0;
	bool allowTemp = conf.hwconf.thermistorSettings.temperatureEnabled;
	conf.hwconf.thermistorSettings.temperatureEnabled = false; // Temp check interrupts adc
	writeReg(0x03, 0); // Read raw adc
	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	setMotionMode(MotionMode::stop,true);
	Delay(100); // Wait a bit before sampling
	uint16_t lastrawA=conf.adc_I0_offset, lastrawB=conf.adc_I1_offset;

	//pulseClipLed(); // Turn on led
	// Disable drivers and measure many samples of zero current
	//enablePin.reset();
	uint32_t tick = HAL_GetTick();
	while(HAL_GetTick() - tick < measuretime_idle){ // Measure idle
		writeReg(0x03, 0); // Read raw adc
		uint32_t adcraw = readReg(0x02);
		uint16_t rawA = adcraw & 0xffff;
		uint16_t rawB = (adcraw >> 16) & 0xffff;
		// Signflip filter for SPI bug
		if(abs(lastrawA-rawA) < 10000 && abs(lastrawB-rawB) < 10000){
			totalA += rawA;
			totalB += rawB;
			measurements_idle++;
			lastrawA = rawA;
			lastrawB = rawB;
		}
//		uint32_t lastMicros = micros();
//		while(micros()-lastMicros < 100){} // Wait 100µs at least
	}
	//enablePin.set();
	int32_t offsetAidle = totalA / (measurements_idle);
	int32_t offsetBidle = totalB / (measurements_idle);

	// Check if offsets are in a valid range
	if(totalA < 100 || totalB < 100 || ((abs(offsetAidle - 0x7fff) > TMC_ADCOFFSETFAIL) || (abs(offsetBidle - 0x7fff) > TMC_ADCOFFSETFAIL)) ){
		ErrorHandler::addError(Error(ErrorCode::adcCalibrationError,ErrorType::critical,"TMC ADC offset calibration failed."));
//		blinkErrLed(100, 0); // Blink forever
//		setPwm(TMC_PwmMode::off); //Disable pwm
//		this->changeState(TMC_ControlState::HardError);
		adcCalibrated = false;
		conf.hwconf.thermistorSettings.temperatureEnabled = allowTemp;
		return false; // An adc or shunt amp is likely broken. do not proceed.
	}
	conf.adc_I0_offset = offsetAidle;
	conf.adc_I1_offset = offsetBidle;
	setAdcOffset(conf.adc_I0_offset, conf.adc_I1_offset);
	// ADC Offsets should now be close to perfect

	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);
	adcCalibrated = true;
	conf.hwconf.thermistorSettings.temperatureEnabled = allowTemp;
	return true;
}

void TMC4671::calibFailCb(){
	if(calibrationFailCount-- != 0){
		changeState(TMC_ControlState::FullCalibration); // retry
	}else{
		Error err = Error(ErrorCode::tmcCalibFail,ErrorType::critical,"TMC calibration failed");
		ErrorHandler::addError(err);
		changeState(TMC_ControlState::HardError);
	}
}
void TMC4671::encoderInit(){

	if(!powerInitialized || !hasPower()){
		changeState(TMC_ControlState::waitPower);
		requestedState = TMC_ControlState::EncoderInit;
		return;
	}

	// Initializes encoder
	if(conf.motconf.enctype == EncoderType_TMC::abn){
		setPosSel(PosSelection::PhiM_abn); // Mechanical Angle
		setVelSel(VelSelection::PhiM_abn); // Mechanical Angle (RPM)
		//setup_ABN_Enc(abnconf);
		if(!encHallRestored){
			estimateABNparams(); // If not saved try to estimate parameters
			recalibrationRequired = true;
		}
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		setPosSel(PosSelection::PhiM_aenc); // Mechanical Angle
		setVelSel(VelSelection::PhiM_aenc); // Mechanical Angle (RPM)
		//setup_AENC(aencconf);
		if(!conf.hwconf.flags.analog_enc_skip_cal){
			calibrateAenc();
		}
	}

	// find index

	if(conf.motconf.enctype == EncoderType_TMC::abn && abnconf.useIndex && !encoderIndexHitFlag){ // TODO changing direction might invalidate phiE offset because of index pulse width
		findEncoderIndex(abnconf.posOffsetFromIndex < 0 ? 10 : -10,bangInitPower/2,true,true); // Go to index and zero encoder
		setPhiEtype(PhiE::openloop); // Openloop used in last step. Use for aligning too
	}else{
		setPhiE_ext(getPhiE());
		setPhiEtype(PhiE::ext);
	}

	// Align encoder
	// TODO handle absolute external encoders
	bangInitEnc(bangInitPower);

	// Check encoder
	if(!checkEncoder()){
		if(++enc_retry > ENC_RETRY_MAX){
			encoderAligned = false;
			Error err = Error(ErrorCode::encoderAlignmentFailed,ErrorType::critical,"Encoder alignment failed");
			ErrorHandler::addError(err);
			stopMotor();
			allowStateChange = false;
			changeState(TMC_ControlState::HardError,true);
		}

		if(manualEncAlign){
			manualEncAlign = false;
			CommandHandler::broadcastCommandReply(CommandReply("Error during check",1), (uint32_t)TMC4671_commands::encalign, CMDtype::get);
		}
		return;
	}
	encoderAligned = true;



	if(conf.motconf.enctype == EncoderType_TMC::abn && abnconf.useIndex && encoderIndexHitFlag)
		setTmcPos(getPosAbs() - abnconf.posOffsetFromIndex); // Load stored position

	if(manualEncAlign){
		manualEncAlign = false;
		CommandHandler::broadcastCommandReply(CommandReply("Aligned successfully",1), (uint32_t)TMC4671_commands::encalign, CMDtype::get);
	}
	changeState(TMC_ControlState::EncoderFinished);

	if(conf.motconf.enctype == EncoderType_TMC::abn){
		setPhiEtype(PhiE::abn);
	}else if(conf.motconf.enctype == EncoderType_TMC::sincos || conf.motconf.enctype == EncoderType_TMC::uvw){
		setPhiEtype(PhiE::aenc);
	}else if(usingExternalEncoder()){
//		setPosSel(PosSelection::PhiE_ext);
//		setVelSel(VelSelection::PhiE_ext); // Mechanical Angle (RPM)
		setPhiEtype(PhiE::extEncoder);
	}

}

/**
 * Changes the encoder type and calls init methods for the encoder types.
 * Setup the specific parameters (abnconf, aencconf...) first.
 */
void TMC4671::setEncoderType(EncoderType_TMC type){
	// If no external timer is set external encoder is not valid
	if( !conf.hwconf.isEncSupported(type) || ((!externalEncoderTimer || !externalEncoderAllowed()) && type == EncoderType_TMC::ext)){
		type = EncoderType_TMC::NONE;
	}

	this->conf.motconf.enctype = type;
	this->statusMask.flags.AENC_N = 0;
	this->statusMask.flags.ENC_N = 0;
	//encoderIndexHitFlag = false;
	setStatusMask(statusMask);
	encoderAligned = false;

	abnconf.rdir = this->conf.encoderReversed;
	aencconf.rdir = this->conf.encoderReversed;

	if(type == EncoderType_TMC::abn){
		encoderAligned = false;
		// Not initialized if cpr not set
		if(this->abnconf.cpr == 0){
			return;
		}
		changeState(TMC_ControlState::EncoderInit);

		setup_ABN_Enc(abnconf);

	// SinCos encoder
	}else if(type == EncoderType_TMC::sincos){
		encoderAligned = false;
		changeState(TMC_ControlState::EncoderInit);
		this->aencconf.uvwmode = false; // sincos mode
		setup_AENC(aencconf);

	// Analog UVW encoder
	}else if(type == EncoderType_TMC::uvw){
		encoderAligned = false;
		changeState(TMC_ControlState::EncoderInit);
		this->aencconf.uvwmode = true; // uvw mode
		setup_AENC(aencconf);

	}else if(type == EncoderType_TMC::hall){ // Hall sensor. Just trust it
		changeState(TMC_ControlState::Shutdown);
		setPosSel(PosSelection::PhiM_hal);
		setVelSel(VelSelection::PhiM_hal);
		encoderAligned = true;
		setPhiEtype(PhiE::hall);
		setup_HALL(hallconf);

	}else if(type == EncoderType_TMC::ext && drvEncoder && drvEncoder->getEncoderType() != EncoderType::NONE){
		// TODO check different encoder type
		encoderAligned = false;
		setUpExtEncTimer();
		//changeState(TMC_ControlState::Shutdown);
		changeState(TMC_ControlState::ExternalEncoderInit);
	}else{
		changeState(TMC_ControlState::Shutdown);
		encoderAligned = true;
	}

}

uint32_t TMC4671::getEncCpr(){
	EncoderType_TMC type = conf.motconf.enctype;
	if(type == EncoderType_TMC::abn || type == EncoderType_TMC::NONE){
		return abnconf.cpr;
	}else if(type == EncoderType_TMC::sincos || type == EncoderType_TMC::uvw){
		return aencconf.cpr;
	}
	else{
		return getCpr();
	}
}

void TMC4671::setPhiEtype(PhiE type){
	conf.motconf.phiEsource = type;

	// External encoder is phiE ext but enables constant phiE updates too
	if(type == PhiE::extEncoder){
		type = PhiE::ext;
	}

	writeReg(0x52, (uint8_t)type & 0xff);
}
PhiE TMC4671::getPhiEtype(){
	PhiE phie = PhiE(readReg(0x52) & 0x7);
	if(phie == PhiE::ext && conf.motconf.phiEsource == PhiE::extEncoder){
		return PhiE::extEncoder;
	}
	return phie;
}

void TMC4671::setMotionMode(MotionMode mode, bool force){
	if(!force){
		nextMotionMode = mode;
		return;
	}
	if(mode != curMotionMode){
		lastMotionMode = curMotionMode;
	}
	curMotionMode = mode;
	updateReg(0x63, (uint8_t)mode, 0xff, 0);
}
MotionMode TMC4671::getMotionMode(){
	curMotionMode = MotionMode(readReg(0x63) & 0xff);
	return curMotionMode;
}

void TMC4671::setOpenLoopSpeedAccel(int32_t speed,uint32_t accel){
	writeReg(0x21, speed);
	writeReg(0x20, accel);
}


void TMC4671::runOpenLoop(uint16_t ud,uint16_t uq,int32_t speed,int32_t accel,bool torqueMode){
	if(this->conf.motconf.motor_type == MotorType::DC){
		uq = ud+uq; // dc motor has no flux. add to torque
	}
	startMotor();
	if(torqueMode){
		if(this->conf.motconf.motor_type == MotorType::DC){
			uq = ud+uq; // dc motor has no flux. add to torque
		}
		setFluxTorque(ud, uq);
	}else{
		setMotionMode(MotionMode::uqudext,true);
		setUdUq(ud,uq);
	}
	int16_t oldPhiE = getPhiE();
	setPhiEtype(PhiE::openloop);
	writeReg(0x23,oldPhiE); // Start running at last phiE value

	setOpenLoopSpeedAccel(speed, accel);
}

void TMC4671::setUdUq(int16_t ud,int16_t uq){
	writeReg(0x24, ud | (uq << 16));
}

void TMC4671::stopMotor(){
	// Stop driver if running

//	enablePin.reset();
	motorEnabledRequested = false;
	if(state == TMC_ControlState::Running || state == TMC_ControlState::EncoderFinished){
		setMotionMode(MotionMode::stop,true);
		setPwm(TMC_PwmMode::off); // disable foc
		changeState(TMC_ControlState::Shutdown);
	}
}
void TMC4671::startMotor(){
	motorEnabledRequested = true;

	if(state == TMC_ControlState::Shutdown && initialized && encoderAligned){
		changeState(TMC_ControlState::Running);
	}
	// Start driver if powered and emergency flag reset
	if(hasPower() && !emergency){
		setPwm(TMC_PwmMode::PWM_FOC); // enable foc
		enablePin.set();
		setMotionMode(nextMotionMode,true);

	}
	else{
		changeState(TMC_ControlState::waitPower);
	}

}

void TMC4671::emergencyStop(bool reset){
	if(!reset){
//		setPwm(TMC_PwmMode::HSlow_LShigh); // Short low side for instant stop
		emergency = true;
		enablePin.reset(); // Release enable pin to disable the whole driver
		motorEnabledRequested = false;
		this->stopMotor();

	}else{
//		enablePin.set();
//		writeReg(0x64, 0); // Set flux and torque 0 directly. Make sure motor does not jump
//		setPwm(TMC_PwmMode::PWM_FOC);
		emergency = false;
		motorEnabledRequested = true;
		//this->changeState(TMC_ControlState::waitPower, true); // Reinit
		this->startMotor();
		if(!motorReady()){
			if(inIsr()){
				ResumeFromISR();
			}else{
				Resume();
			}
		}
	}
}

/**
 * Calculates a flux value based on the internal and external voltage difference to dissipate energy without
 * a brake resistor
 */
int16_t TMC4671::controlFluxDissipate(){

	int32_t vDiff = getIntV() - getExtV();
	if(vDiff > FLUX_DISSIPATION_LIMIT){
		// Reaches limit at +5v if scaler is 1
		return(clip<int32_t,int32_t>(vDiff * conf.hwconf.fluxDissipationScaler * curLimits.pid_torque_flux * 0.0002,0,curLimits.pid_torque_flux));
	}
	return 0;
}

/**
 * Sets a torque in positive or negative direction
 * For ADC linearity reasons under 25000 is recommended
 */
void TMC4671::turn(int16_t power){
	if(!(this->motorReady() && motorEnabledRequested))
		return;

	// --- Position diagnostics (every turn, independent of cogging) ---
	// Lets MCU Viewer compare what Axis sees (getEncAngle) vs the raw
	// hardware reads vs the sampler cache, to locate the 360deg wrap.
	{
		Encoder* enc = this->getEncoder();
		if (enc != nullptr) {
			float pf = enc->getPos_f();
			g_tmc4671_cogging_debug.pos_rt_posf       = this->getFilteredPosition();
			g_tmc4671_cogging_debug.pos_rt_encangle_deg = 360.0f * pf;
			g_tmc4671_cogging_debug.pos_hw_posf        = pf;
			g_tmc4671_cogging_debug.pos_hw_posabsf     = enc->getPosAbs_f();
			g_tmc4671_cogging_debug.pos_hw_raw         = enc->getPos();
			g_tmc4671_cogging_debug.pos_hw_rawabs      = enc->getPosAbs();
			g_tmc4671_cogging_debug.pos_cpr            = enc->getCpr();
			g_tmc4671_cogging_debug.pos_sampler_active = enc->isSamplerActive() ? 1 : 0;
		}
	}

	int32_t flux = 0;
	int32_t totalPower = power;

	// Anticogging is enabled in firmware
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	if (cogging_enabled && !this->isCalibrationInProgress()) {
		float pos_f = this->getFilteredPosition();

		// Measure RPM from position delta (runs in turn() at firmware speed).
		// get_velocity() updates measured_rpm / measured_rpm_signed and
		// advances the shared last_vel_tick / prev_filtered_pos state.
		(void)this->get_velocity();

		// --- DO NOT SHIFT pos_f ---
		// Leave pos_f as the pure, raw spatial position.
		// Motor inductance acts as an RL low-pass filter whose phase delay
		// physically cannot exceed 90° electrical. A mechanical shift would
		// multiply by harmonic order and artificially push high harmonics
		// past 90°, sometimes flipping them completely (180°+) — that is what
		// creates texture/bumps at higher RPMs. We therefore compute the
		// electrical advance separately and apply it ONLY to harmonics at or
		// below the dominant order (see compensation loop below).
		float adv_mech_rad = 0.0f;
		if (phase_adv_curve_valid) {
			float adv_deg = interpolatePhaseAdvance(measured_rpm);
			float dir = (measured_rpm_signed >= 0.0f) ? 1.0f : -1.0f;
			adv_mech_rad = dir * (adv_deg / 360.0f) * 2.0f * PI;
		}

		// Fourier series compensation with per-RPM harmonic blending.
#ifdef COGGING_DISABLE_BLEND
		// Blending disabled: use the base cogging table (profile 0) directly.
		Harmonic* blended = this->cogging_harmonics;
#else
		// Per-RPM harmonic blending: three tables were calibrated at different
		// RPMs; blend between them based on measured_rpm.
		Harmonic blended[COGGING_HARMONICS_COUNT];
		this->blendHarmonicTables(measured_rpm, blended);
#endif

		float angle_rad = pos_f * 2.0f * PI;

		// --- FF source selection (cogging_ff_mode) ---
		//   0 = harmonic sum (default, legacy)
		//   1 = combined bin LUT (friction-free, pure cogging)
		//   2 = per-direction bin LUT (includes friction asymmetry / hysteresis)
		float compensation = 0.0f;
		bool using_bins = (this->cogging_ff_mode != 0) && this->bins_data_valid;

		if (using_bins) {
			// Pick which bin array to read from.
			const float* bin_src = nullptr;
			if (this->cogging_ff_mode == 1) {
				bin_src = this->cogging_bins_combined;
			} else { // mode 2: per-direction
				// Zero-velocity deadband: below this |omega|, hold the last-used
				// direction to avoid the FF flipping and creating a torque step.
				// The deadband is intentionally tiny (0.05 RPM) so it only bites
				// at true standstill.
				if (fabsf(measured_rpm_signed) < 0.05f) {
					bin_src = (this->cogging_last_dir >= 0) ? this->cw_bins : this->ccw_bins;
				} else {
					if (measured_rpm_signed >= 0.0f) {
						bin_src = this->cw_bins;
						this->cogging_last_dir = +1;
					} else {
						bin_src = this->ccw_bins;
						this->cogging_last_dir = -1;
					}
				}
			}
			// Linear interpolation between bin centers.
			// pos_f is in [0,1), so pos_scaled spans [0, BIN_COUNT).
			float pos_scaled = pos_f * (float)COGGING_DFT_BIN_COUNT;
			uint32_t b0 = (uint32_t)pos_scaled;
			float frac = pos_scaled - (float)b0;
			if (b0 >= COGGING_DFT_BIN_COUNT) b0 = COGGING_DFT_BIN_COUNT - 1;
			uint32_t b1 = b0 + 1;
			if (b1 >= COGGING_DFT_BIN_COUNT) b1 = 0; // wrap (position is cyclic)
			compensation = bin_src[b0] * (1.0f - frac) + bin_src[b1] * frac;
			// NOTE: bin values are iq_cmd means captured during calibration.
			// During calibration the controller drove iq_cmd = -T_cog (to hold
			// constant speed), so the stored value IS the FF we want to inject
			// to cancel cogging. No sign flip needed.
		} else {
			// Legacy harmonic-sum path (modes 0 or no bins available).
			// 1. Find the dominant harmonic (largest amplitude)
			float dom_amp = 0.0f, dom_phase = 0.0f;
			uint16_t dom_order = 1;
			for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
				if (blended[i].amplitude > dom_amp) {
					dom_amp = blended[i].amplitude;
					dom_order = (uint16_t)blended[i].order;
					dom_phase = blended[i].phase;
				}
			}

			// 2. Calculate Compensation
			for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
				if (blended[i].amplitude > 0.0f) {
					float electrical_advance = 0.0f;

				// Apply phase advance to the dominant harmonic, its 2nd
				// multiple (e.g. H12 → H24), and any lower fundamentals.
				// Higher multiples and texture harmonics remain anchored to
				// their physical spatial locations.
				if (blended[i].order <= dom_order || (dom_order > 0 && blended[i].order == dom_order * 2)) {
					electrical_advance = (float)blended[i].order * adv_mech_rad;
				}

				compensation += blended[i].amplitude * arm_sin_f32(
					angle_rad * blended[i].order + blended[i].phase + electrical_advance);
				}
			}

			// 3. Cogging waveshaping ("3rd harmonic" trim)
			if (h3_shaping != 0.0f && dom_amp > 0.0f) {
				// Apply the advance to the waveshaper too, since it tracks the dominant wave.
				float dom_elec_adv = (float)dom_order * adv_mech_rad;
				float shaped_arg = this->h3_mult * ((float)dom_order * angle_rad + dom_phase + dom_elec_adv) + this->h3_phase_trim;
				compensation -= this->h3_shaping * dom_amp * arm_sin_f32(shaped_arg);
			}
		}
		// Speed-dependent scale: interpolate from calibrated curve if available
		float dyn_scale = this->cogging_scale;

		if (scale_curve_valid) {
			// Use EMA-filtered RPM measured in acttrq handler (polled every 50ms)
			float rpm = measured_rpm;
			// Small deadband for encoder noise at standstill
			if (rpm < 0.3f) rpm = 0.0f;
			dyn_scale = interpolateScale(rpm);
		}

		this->last_anticogging_torque = (int32_t)(dyn_scale * compensation);
		this->last_cogging_scale = dyn_scale;
		this->last_power_setpoint = power;

		totalPower += this->last_anticogging_torque;
		
		// Update debug struct for ST-Link live view
		g_tmc4671_cogging_debug.phase = 0; // Idle = normal operation
		g_tmc4671_cogging_debug.iqCmd = (float)power;
		g_tmc4671_cogging_debug.iqCompensation = (float)this->last_anticogging_torque;
		g_tmc4671_cogging_debug.Appliediq = (float)totalPower;
	} else {
		this->last_anticogging_torque = 0;
	}
#endif

	// Inversion if encoder is reversed (ext) or force inversion is requested
	if((this->conf.encoderReversed && conf.motconf.enctype == EncoderType_TMC::ext) ^ conf.invertForce){
		totalPower = -totalPower;
	}

	// Flux offset for field weakening
	flux = idleFlux-clip<int32_t,int16_t>(abs(power),0,maxOffsetFlux);

	/*
	 * If flux dissipation is on prefer this over the resistor.
	 * Warning: The axis only calls this function when active and if torque changed.
	 * It may not update during sustained force and still cause overvoltage conditions.
	 * TODO periodically check and update if driver is on but no torque update is sent
	 */
	if(conf.hwconf.fluxDissipationScaler && conf.enableFluxDissipation){
		int16_t dissipationFlux = controlFluxDissipate();
		if(dissipationFlux != 0){
			flux = dissipationFlux;
		}
	}

	setFluxTorque(flux, clip<int32_t, int16_t>(totalPower, -32768, 32767));
}

/**
 * Changes the position sensor source
 */
void TMC4671::setPosSel(PosSelection psel){
	writeReg(0x51, (uint8_t)psel);
	this->conf.motconf.pos_sel = psel;
}

/**
 * Changes the velocity sensor source (RPM if PhiM source used)
 * @param mode 0 = fixed frequency sampling (~4369.067Hz), 1 = PWM sync time difference measurement
 */
void TMC4671::setVelSel(VelSelection vsel,uint8_t mode){
	uint32_t vselMode = ((uint8_t)vsel & 0xff) | ((mode & 0xff) << 8);
	writeReg(0x50, vselMode);
	this->conf.motconf.vel_sel = vsel;
}

/**
 * Changes the mode of the 8 GPIO pins
 * Banks A and B can be mapped independently to inputs or outputs, as a DS adc interface or by default as a secondary debug SPI port
 */
void TMC4671::setGpioMode(TMC_GpioMode mode){
	uint8_t modeReg = 0;
	switch(mode){
	default:
	case TMC_GpioMode::DebugSpi:
		modeReg = 0;			break;
	case TMC_GpioMode::DSAdcClkOut:
		modeReg = 0b1100111;	break;
	case TMC_GpioMode::DSAdcClkIn:
		modeReg = 0b0000111;	break;
	case TMC_GpioMode::Ain_Bin:
		modeReg = 0b0000001;	break;
	case TMC_GpioMode::Ain_Bout:
		modeReg = 0b0010001;	break;
	case TMC_GpioMode::Aout_Bin:
		modeReg = 0b0001001;	break;
	case TMC_GpioMode::Aout_Bout:
		modeReg = 0b0011001;	break;
	}

	updateReg(0x7B, modeReg,0xff,0);
}

/**
 * Reads the state of the 8 gpio pins
 */
uint8_t TMC4671::getGpioPins(){
	return readReg(0x7B) >> 24;
}

/**
 * Changes the state of gpio pins that are mapped as output
 * lower 4 bits bank A, upper 4 bits bank B
 */
void TMC4671::setGpioPins(uint8_t pins){
	uint32_t reg = pins << 16;
	updateReg(0x7B, reg,0xff,16);
}


/**
 * Returns a string with the name and version of the chip
 */
std::pair<uint32_t,std::string> TMC4671::getTmcType(){

	std::string reply = "";
	writeReg(1, 0);
	uint32_t nameInt = readReg(0);
	if(nameInt == 0 || nameInt ==  0xffffffff){
		reply = "No driver connected";
		return std::pair<uint32_t,std::string>(0,reply);
	}

	nameInt = __REV(nameInt);
	char* name = reinterpret_cast<char*>(&nameInt);
	std::string namestring = std::string(name,sizeof(nameInt));

	writeReg(1, 1);
	uint32_t versionInt = readReg(0);

	std::string versionstring = std::to_string((versionInt >> 16) && 0xffff) + "." + std::to_string((versionInt) && 0xffff);

	reply += "TMC" + namestring + " v" + versionstring;
	return std::pair<uint32_t,std::string>(versionInt,reply);
}

Encoder* TMC4671::getEncoder(){
	if((conf.motconf.enctype == EncoderType_TMC::ext && externalEncoderTimer) || conf.combineEncoder){
		return MotorDriver::drvEncoder.get();
	}else{
		return static_cast<Encoder*>(this);
	}
}

void TMC4671::setEncoder(std::shared_ptr<Encoder>& encoder){
	MotorDriver::drvEncoder = encoder;
	if(conf.motconf.enctype == EncoderType_TMC::ext && externalEncoderTimer){
		if(!extEncUpdater){ // If updater has not been set up because the encoder mode was changed before the external encoder passed force it now
			setUpExtEncTimer();
		}
		changeState(TMC_ControlState::ExternalEncoderInit);
	}
}

bool TMC4671::hasIntegratedEncoder(){
	// Use internal encoder if not external encoder is selected
	return conf.motconf.enctype != EncoderType_TMC::ext && !this->conf.combineEncoder;
}

/**
 * Changes position using offset from index
 */
void TMC4671::setPos(int32_t pos){
	if(this->conf.motconf.enctype == EncoderType_TMC::abn && abnconf.useIndex){

		int32_t tmcpos = readReg(0x6B); // Current Position
		int32_t offset = (tmcpos - pos) % 0xffff; // Difference between current position and target

//		setup_ABN_Enc(abnconf);
		abnconf.posOffsetFromIndex += offset;
		setTmcPos(getPosAbs() - abnconf.posOffsetFromIndex);
	}else{
		setTmcPos(pos);
	}

}

/**
 * Changes position in tmc register
 */
void TMC4671::setTmcPos(int32_t pos){

	writeReg(0x6B, pos);
}

int32_t TMC4671::getPosHardware() {

	int32_t pos = (int32_t)readReg(0x6B);
	return pos;
}

int32_t TMC4671::getPosAbsHardware() {
	int16_t pos;
	if(this->conf.motconf.enctype == EncoderType_TMC::abn){
		pos = (int16_t)readReg(0x2A) & 0xffff; // read phiM
	}else if(this->conf.motconf.enctype == EncoderType_TMC::hall){
		pos = (int16_t)readReg(0x3A); // read phiM
	}else if(this->conf.motconf.enctype == EncoderType_TMC::sincos || this->conf.motconf.enctype == EncoderType_TMC::uvw){
		pos = (int16_t)readReg(0x46) & 0xffff; // read phiM
	}else{
		pos = getPosHardware(); // read phiM
	}

	return pos;
}


uint32_t TMC4671::getCpr(){
//	if(this->conf.motconf.phiEsource == PhiE::abn){
//		return abnconf.cpr;
//	}else{
	if(usingExternalEncoder()){
		return drvEncoder->getCpr();
	}
	return 0xffff;
//	}

}
void TMC4671::setCpr(uint32_t cpr){
	if(cpr == 0)
		cpr = 1;


	this->abnconf.cpr = cpr;
	this->aencconf.cpr = cpr;
	writeReg(0x26, abnconf.cpr); //ABN
	writeReg(0x40, aencconf.cpr); //AENC

}

/**
 * Converts encoder counts to phiM
 */
uint32_t TMC4671::encToPos(uint32_t enc){
	return enc*(0xffff / abnconf.cpr); //*(conf.motconf.pole_pairs)
}
uint32_t TMC4671::posToEnc(uint32_t pos){
	return pos/((0xffff / abnconf.cpr)) % abnconf.cpr; //(conf.motconf.pole_pairs)
}

EncoderType TMC4671::getEncoderType(){
	if(conf.motconf.enctype == EncoderType_TMC::abn && abnconf.useIndex && encoderIndexHitFlag){
		return EncoderType::incrementalIndex;
	}
	return EncoderType::incremental;
}



void TMC4671::setAdcOffset(uint32_t adc_I0_offset,uint32_t adc_I1_offset){
	conf.adc_I0_offset = adc_I0_offset;
	conf.adc_I1_offset = adc_I1_offset;

	updateReg(0x09, adc_I0_offset, 0xffff, 0);
	updateReg(0x08, adc_I1_offset, 0xffff, 0);
}

void TMC4671::setAdcScale(uint32_t adc_I0_scale,uint32_t adc_I1_scale){
	conf.adc_I0_scale = adc_I0_scale;
	conf.adc_I1_scale = adc_I1_scale;

	updateReg(0x09, adc_I0_scale, 0xffff, 16);
	updateReg(0x08, adc_I1_scale, 0xffff, 16);
}

void TMC4671::setupFeedForwardTorque(int32_t gain, int32_t constant){
	writeReg(0x4E, 42);
	writeReg(0x4D, gain);
	writeReg(0x4E, 43);
	writeReg(0x4D, constant);
}
void TMC4671::setupFeedForwardVelocity(int32_t gain, int32_t constant){
	writeReg(0x4E, 40);
	writeReg(0x4D, gain);
	writeReg(0x4E, 41);
	writeReg(0x4D, constant);
}

void TMC4671::setFFMode(FFMode mode){
	updateReg(0x63, (uint8_t)mode, 0xff, 16);
	if(mode!=FFMode::none){

		setSequentialPI(true);
	}
}

void TMC4671::setSequentialPI(bool sequential){
	curPids.sequentialPI = sequential;
	updateReg(0x63, sequential ? 1 : 0, 0x1, 31);
}

void TMC4671::setExternalEncoderAllowed(bool allow){
#ifndef TIM_TMC
	allowExternalEncoder = false;
#else
	bool lastAllowed = allowExternalEncoder;
	allowExternalEncoder = allow;
	// External encoder was previously used but now not allowed anymore. Change to none type encoder
	if(!allow && lastAllowed && conf.motconf.enctype == EncoderType_TMC::ext){
		setEncoderType(EncoderType_TMC::NONE); // Reinit encoder
	}
#endif
}

bool TMC4671::externalEncoderAllowed(){
#ifndef TIM_TMC
	return false;
#else
	return allowExternalEncoder && conf.hwconf.flags.enc_ext;
#endif
}

void TMC4671::setMotorType(MotorType motor,uint16_t poles){

	if(!conf.hwconf.isMotSupported(motor)){
		motor = MotorType::NONE;
	}
	if(motor == MotorType::DC){
		poles = 1;
	}

	conf.motconf.motor_type = motor;
	conf.motconf.pole_pairs = poles;
	uint32_t mtype = poles | ( ((uint8_t)motor&0xff) << 16);
//	if(motor != MotorType::STEPPER){
//		maxOffsetFlux = 0; // Offsetflux only helpful for steppers. Has no effect otherwise
//	}
	writeReg(0x1B, mtype);
	if(motor == MotorType::BLDC && !ES_TMCdetected){
		setSvPwm(conf.motconf.svpwm); // Higher speed for BLDC motors. Not available in engineering samples
	}
}

void TMC4671::setTorque(int16_t torque){
	if(curMotionMode != MotionMode::torque){
		setMotionMode(MotionMode::torque,true);
	}

	// Update main torque setpoint
	updateReg(0x64, torque, 0xffff, 16);
}

int16_t TMC4671::getTorque(){
	return readReg(0x64) >> 16;
}

void TMC4671::setFlux(int16_t flux){
	if(curMotionMode != MotionMode::torque){
		setMotionMode(MotionMode::torque,true);
	}
	updateReg(0x64,flux,0xffff,0);
}
int16_t TMC4671::getFlux(){
	return readReg(0x64) && 0xffff;
}
void TMC4671::setFluxTorque(int16_t flux, int16_t torque){
	if(curMotionMode != MotionMode::torque && !emergency){
		setMotionMode(MotionMode::torque,true);
	}
#ifdef TMC4671_TORQUE_USE_ASYNC
	writeRegAsync(0x64, (flux & 0xffff) | (torque << 16));
#else
	// Update main flux and torque setpoints
	writeReg(0x64, (flux & 0xffff) | (torque << 16));
#endif
}


void TMC4671::setFluxTorqueFF(int16_t flux, int16_t torque){
	if(curMotionMode != MotionMode::torque){
		setMotionMode(MotionMode::torque,true);
	}
	writeReg(0x65, (flux & 0xffff) | (torque << 16));
}

/**
 * Ramps flux from current value to a target value over a specified duration
 */
void TMC4671::rampFlux(uint16_t target,uint16_t time_ms){
	uint16_t startFlux = readReg(0x64) & 0xffff;
	int32_t stepsize = (target - startFlux) / std::max<uint16_t>(1, time_ms/2);
	if(stepsize == 0){
		stepsize = startFlux < target ? 1 : -1;
	}
	uint16_t flux = startFlux;
	while(abs(target - flux) >= abs(stepsize)){
		flux+=stepsize;
		setFluxTorque(std::max<int32_t>(0,flux), 0);
		Delay(2);
	}
}

void TMC4671::setPids(TMC4671PIDConf pids){
	curPids = pids;
	writeReg(0x54, pids.fluxI | (pids.fluxP << 16));
	writeReg(0x56, pids.torqueI | (pids.torqueP << 16));
	writeReg(0x58, pids.velocityI | (pids.velocityP << 16));
	writeReg(0x5A, pids.positionI | (pids.positionP << 16));
	setSequentialPI(pids.sequentialPI);
}

TMC4671PIDConf TMC4671::getPids(){
	uint32_t f = readReg(0x54);
	uint32_t t = readReg(0x56);
	uint32_t v = readReg(0x58);
	uint32_t p = readReg(0x5A);
	// Update pid storage
	curPids = {(uint16_t)(f&0xffff),(uint16_t)(f>>16),(uint16_t)(t&0xffff),(uint16_t)(t>>16),(uint16_t)(v&0xffff),(uint16_t)(v>>16),(uint16_t)(p&0xffff),(uint16_t)(p>>16)};
	return curPids;
}

/**
 * Limits the PWM value
 */
void TMC4671::setUqUdLimit(uint16_t limit){
	this->curLimits.pid_uq_ud = limit;
	writeReg(0x5D, limit);
}

void TMC4671::setPowerLimit(uint16_t power) {
	maxPowerAxis = power;
	setTorqueLimit(power);
}

void TMC4671::setTorqueLimit(uint16_t limit){
	this->curLimits.pid_torque_flux = limit;
	bangInitPower = (float)limit*0.75;
	writeReg(0x5E, limit);
}

void TMC4671::setPidPrecision(TMC4671PidPrecision setting){

	this->pidPrecision = setting;
	uint16_t dat = setting.current_I;
	dat |= setting.current_P << 1;
	dat |= setting.velocity_I << 2;
	dat |= setting.velocity_P << 3;
	dat |= setting.position_I << 4;
	dat |= setting.position_P << 5;
	writeReg(0x4E, 62); // set config register address
	writeReg(0x4D, dat);
}

void TMC4671::setLimits(TMC4671Limits limits){
	this->curLimits = limits;
	writeReg(0x5C, limits.pid_torque_flux_ddt);
	writeReg(0x5D, limits.pid_uq_ud);
	writeReg(0x5E, limits.pid_torque_flux);
	writeReg(0x5F, limits.pid_acc_lim);
	writeReg(0x60, limits.pid_vel_lim);
	writeReg(0x61, limits.pid_pos_low);
	writeReg(0x62, limits.pid_pos_high);
}

TMC4671Limits TMC4671::getLimits(){
	curLimits.pid_acc_lim = readReg(0x5F);
	curLimits.pid_torque_flux = readReg(0x5E);
	curLimits.pid_torque_flux_ddt = readReg(0x5C);
	curLimits.pid_uq_ud= readReg(0x5D);
	curLimits.pid_vel_lim = readReg(0x60);
	curLimits.pid_pos_low = readReg(0x61);
	curLimits.pid_pos_high = readReg(0x62);
	return curLimits;
}

/**
 * Applies a biquad filter to the flux target
 * Set nullptr to disable
 */
void TMC4671::setBiquadFlux(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.flux = filter;
	writeReg(0x4E, 25);
	writeReg(0x4D, bq.a1);
	writeReg(0x4E, 26);
	writeReg(0x4D, bq.a2);
	writeReg(0x4E, 28);
	writeReg(0x4D, bq.b0);
	writeReg(0x4E, 29);
	writeReg(0x4D, bq.b1);
	writeReg(0x4E, 30);
	writeReg(0x4D, bq.b2);
	writeReg(0x4E, 31);
	writeReg(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the pos target
 * Set nullptr to disable
 */
void TMC4671::setBiquadPos(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.pos = filter;
	writeReg(0x4E, 1);
	writeReg(0x4D, bq.a1);
	writeReg(0x4E, 2);
	writeReg(0x4D, bq.a2);
	writeReg(0x4E, 4);
	writeReg(0x4D, bq.b0);
	writeReg(0x4E, 5);
	writeReg(0x4D, bq.b1);
	writeReg(0x4E, 6);
	writeReg(0x4D, bq.b2);
	writeReg(0x4E, 7);
	writeReg(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the actual measured velocity
 * Set nullptr to disable
 */
void TMC4671::setBiquadVel(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.vel = filter;
	writeReg(0x4E, 9);
	writeReg(0x4D, bq.a1);
	writeReg(0x4E, 10);
	writeReg(0x4D, bq.a2);
	writeReg(0x4E, 12);
	writeReg(0x4D, bq.b0);
	writeReg(0x4E, 13);
	writeReg(0x4D, bq.b1);
	writeReg(0x4E, 14);
	writeReg(0x4D, bq.b2);
	writeReg(0x4E, 15);
	writeReg(0x4D, bq.enable & 0x1);
}

/**
 * Applies a biquad filter to the torque target
 * Set nullptr to disable
 */
void TMC4671::setBiquadTorque(const TMC4671Biquad &filter){
	const TMC4671Biquad_t& bq = filter.params;
	curFilters.torque = filter;
	writeReg(0x4E, 17);
	writeReg(0x4D, bq.a1);
	writeReg(0x4E, 18);
	writeReg(0x4D, bq.a2);
	writeReg(0x4E, 20);
	writeReg(0x4D, bq.b0);
	writeReg(0x4E, 21);
	writeReg(0x4D, bq.b1);
	writeReg(0x4E, 22);
	writeReg(0x4D, bq.b2);
	writeReg(0x4E, 23);
	writeReg(0x4D, bq.enable & 0x1);
}


/**
 * Changes the torque biquad filter
 */
void TMC4671::setTorqueFilter(TMC4671Biquad_conf& conf){
//	this->torqueFilter = params;
//	setBiquadTorque(makeLpTmcFilter(params,enable));
//
	// Presets: off, Lowpass, notch, peak
	this->torqueFilterConf = conf;
	TMC4671Biquad filter;
	switch(conf.mode){
	default:
	case TMCbiquadpreset::none:
		filter = TMC4671Biquad(false);
		break;
	case TMCbiquadpreset::lowpass:
		filter = TMC4671Biquad(Biquad(BiquadType::lowpass, (float)conf.params.freq / getPwmFreq(), (float)conf.params.q/100.0,0.0), true);
		break;
	case TMCbiquadpreset::notch:
		filter = TMC4671Biquad(Biquad(BiquadType::notch, (float)conf.params.freq / getPwmFreq(), (float)conf.params.q/10.0,0.0), true);
		break;
	case TMCbiquadpreset::peak:
		filter = TMC4671Biquad(Biquad(BiquadType::peak, (float)conf.params.freq / getPwmFreq(), (float)conf.params.q/10.0,conf.gain), true);
		break;
	}
	setBiquadTorque(filter);
}


/**
 *  Sets the raw brake resistor limits.
 *  Centered at 0x7fff
 *  Set both 0xffff to deactivate
 */
void TMC4671::setBrakeLimits(uint16_t low,uint16_t high){
	uint32_t val = low | (high << 16);
	writeReg(0x75,val);
}

/**
 * Moves the rotor and estimates polarity and direction of the encoder
 * Polarity is found by measuring the n pulse.
 * If polarity was found to be reversed during the test direction will be reversed again to account for that
 */
void TMC4671::estimateABNparams(){
	blinkClipLed(100, 0);
	int32_t pos = getPos();
	setTmcPos(0);
	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	updateReg(0x25, 0,0x1000,12); // Set dir normal
	setPhiE_ext(0);
	setPhiEtype(PhiE::ext);
	setFluxTorque(0, 0);
	setMotionMode(MotionMode::torque,true);
	rampFlux(bangInitPower, 1000);

	int16_t phiE_abn = readReg(0x2A)>>16;
	int16_t phiE_abn_old = 0;
	int16_t rcount=0,c = 0; // Count how often direction was in reverse
	uint16_t highcount = 0; // Count high state of n pulse for polarity estimation

	// Rotate a bit
	for(int16_t p = 0;p<0x0fff;p+=0x2f){
		setPhiE_ext(p);
		Delay(10);
		c++;
		phiE_abn_old = phiE_abn;
		phiE_abn = readReg(0x2A)>>16;
		// Count how often the new position was lower than the previous indicating a reversed encoder or motor direction
		if(phiE_abn < phiE_abn_old){
			rcount++;
		}
		if((readReg(0x76) & 0x04) >> 2){
			highcount++;
		}
	}
	setTmcPos(pos+getPos());

	rampFlux(0, 100);
	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);

	bool npol = highcount > c/2;
	abnconf.rdir = rcount > c/2;
	if(npol != abnconf.npol) // Invert dir if polarity was reversed TODO correct? likely wrong at the moment
		abnconf.rdir = !abnconf.rdir;

	abnconf.apol = npol;
	abnconf.bpol = npol;
	abnconf.npol = npol;
	blinkClipLed(0, 0);
}

/**
 * Similar to the ABN calibration this moved the motor and measures the encoder direction
 */
void TMC4671::estimateExtEnc(){
	blinkClipLed(100, 0);

	PhiE lastphie = getPhiEtype();
	MotionMode lastmode = getMotionMode();
	int16_t oldPhiE = getPhiE();
	setPhiE_ext(oldPhiE);
	setPhiEtype(PhiE::ext);
	setFluxTorque(0, 0);
	setMotionMode(MotionMode::torque,true);
	rampFlux(bangInitPower, 1000);
	int16_t phiE_enc = getPhiEfromExternalEncoder();
	int16_t phiE_enc_old = 0;
	int16_t rcount=0,c = 0; // Count how often direction was in reverse

	// Rotate a bit
	for(int16_t p = 0;p<0x0fff;p+=0x2f){
		setPhiE_ext(p+oldPhiE);
		Delay(10);
		c++;
		phiE_enc_old = phiE_enc;
		phiE_enc = getPhiEfromExternalEncoder();
		// Count how often the new position was lower than the previous indicating a reversed encoder or motor direction
		if(phiE_enc < phiE_enc_old){
			rcount++;
		}
	}

	rampFlux(0, 100);
	setPhiEtype(lastphie);
	setMotionMode(lastmode,true);

	if(rcount > c/2)
		conf.encoderReversed = !conf.encoderReversed;

	blinkClipLed(0, 0);
}



/**
 * Sets pwm mode: \n
 * 0 = pwm off \n
 * 1 = pwm off, HS low, LS high \n
 * 2 = pwm off, HS high, LS low \n
 * 3 = pwm off \n
 * 4 = pwm off \n
 * 5 = pwm LS only \n
 * 6 = pwm HS only \n
 * 7 = pwm on centered, FOC mode
 */
void TMC4671::setPwm(TMC_PwmMode val){
	updateReg(0x1A,(uint8_t)val,0xff,0);
}

void TMC4671::setBBM(uint8_t bbmL,uint8_t bbmH){
	this->conf.bbmH = bbmH;
	this->conf.bbmL = bbmL;
	uint32_t bbmr = bbmL | (bbmH << 8);
	writeReg(0x19, bbmr);
}

void TMC4671::setPwm(uint8_t val,uint16_t maxcnt,uint8_t bbmL,uint8_t bbmH){
	setPwmMaxCnt(maxcnt);
	setPwm((TMC_PwmMode)val);
	setBBM(bbmL, bbmH);
	writeReg(0x17,0); //Polarity
}

/**
 * Enable or disable space vector pwm for 3 phase motors
 * Normally active but should be disabled if the motor has no isolated star point
 */
void TMC4671::setSvPwm(bool enable){
	conf.motconf.svpwm = enable;
	if(conf.motconf.motor_type != MotorType::BLDC){
		enable = false; // Only valid for 3 phase motors with isolated star point
	}

	updateReg(0x1A,enable,0x01,8);
}

/**
 * Returns the PWM loop frequency in Hz
 * Depends on hardware clock and pwm counter setting. Default 25kHz
 */
float TMC4671::getPwmFreq(){
	return (4.0 * this->conf.hwconf.clockfreq) / (this->conf.pwmcnt +1);
}

/**
 * Changes PWM frequency
 * Max value 4095, minimum 255
 *
 */
void TMC4671::setPwmMaxCnt(uint16_t maxcnt){
	int clipped = clip((int)maxcnt, 255, 4095);
	maxcnt = (uint16_t)clipped;
	this->conf.pwmcnt = maxcnt;
	writeReg(0x18, maxcnt);
}

/**
 * Changes the PWM frequency to a desired frequency
 * Possible values depend on the hwclock.
 * At 25MHz the lowest possible frequency is 24.1kHz
 */
void TMC4671::setPwmFreq(float freq){
	if(freq <= 0)
		return;
	uint16_t maxcnt = ((4.0 * this->conf.hwconf.clockfreq) / freq) -1;
	setPwmMaxCnt(maxcnt);
}


void TMC4671::initAdc(uint16_t mdecA, uint16_t mdecB,uint32_t mclkA,uint32_t mclkB){
	uint32_t dat = mdecA | (mdecB << 16);
	writeReg(0x07, dat);

	writeReg(0x05, mclkA);
	writeReg(0x06, mclkB);
	// Enable/Disable adcs
	updateReg(0x04, mclkA == 0 ? 0 : 1, 0x1, 4);
	updateReg(0x04, mclkB == 0 ? 0 : 1, 0x1, 20);

	writeReg(0x0A,0x18000100); // ADC Selection
}

/**
 * Returns measured flux and torque as a pair
 * Flux is first, torque second item
 */
std::pair<int32_t,int32_t> TMC4671::getActualTorqueFlux(){
	uint32_t tfluxa = readReg(0x69);
	int16_t af = (tfluxa & 0xffff);
	int16_t at = (tfluxa >> 16);
	return std::make_pair((int32_t)af, (int32_t)at);
}

/**
 * Returns measured flux
 */
int32_t TMC4671::getActualFlux(){
	uint32_t tfluxa = readReg(0x69);
	int16_t af = (tfluxa & 0xffff);
	return af;
}

/**
 * Returns measured torque
 */
int32_t TMC4671::getActualTorque(){
	uint32_t tfluxa = readReg(0x69);
	int16_t at = (tfluxa >> 16);
	return at;
}


//__attribute__((optimize("-Ofast")))
uint32_t TMC4671::readReg(uint8_t reg){
	spiPort.takeSemaphore();
	uint8_t req[5] = {(uint8_t)(0x7F & reg),0,0,0,0};
	uint8_t tbuf[5];
	// 500ns delay after sending first byte
	spiPort.transmitReceive(req, tbuf, 5,this, SPITIMEOUT);
	uint32_t ret;
	memcpy(&ret,tbuf+1,4);
	ret = __REV(ret);

	return ret;
}

//__attribute__((optimize("-Ofast")))
void TMC4671::writeReg(uint8_t reg,uint32_t dat){

	// wait until ready
	spiPort.takeSemaphore();
	spi_buf[0] = (uint8_t)(0x80 | reg);
	dat =__REV(dat);
	memcpy(spi_buf+1,&dat,4);

	// -----
	spiPort.transmit(spi_buf, 5, this, SPITIMEOUT);
}

void TMC4671::writeRegAsync(uint8_t reg,uint32_t dat){

	// wait until ready
	spiPort.takeSemaphore();
	spi_buf[0] = (uint8_t)(0x80 | reg);
	dat =__REV(dat);
	memcpy(spi_buf+1,&dat,4);

	// -----
#ifdef TMC4671_ALLOW_DMA
	spiPort.transmit_DMA(this->spi_buf, 5, this);
#else
	spiPort.transmit_IT(this->spi_buf, 5, this);
#endif
}

void TMC4671::updateReg(uint8_t reg,uint32_t dat,uint32_t mask,uint8_t shift){

	uint32_t t = readReg(reg) & ~(mask << shift);
	t |= ((dat & mask) << shift);
	writeReg(reg, t);
}

void TMC4671::beginSpiTransfer(SPIPort* port){
	assertChipSelect();
}
void TMC4671::endSpiTransfer(SPIPort* port){
	clearChipSelect();
	port->giveSemaphore();
}

/**
 * Reads status flags
 * @param maskedOnly Masks flags by previously set flag mask that would trigger an interrupt. False to read all flags
 */
StatusFlags TMC4671::readFlags(bool maskedOnly){
	uint32_t flags = readReg(0x7C);
	if(maskedOnly){
		flags = flags & this->statusMask.asInt;
	}
	this->statusFlags.asInt = flags; // Only set flags that are marked to trigger a notification
	return statusFlags;
}

void TMC4671::setStatusMask(StatusFlags mask){
	writeReg(0x7D, mask.asInt);
}

void TMC4671::setStatusMask(uint32_t mask){
	writeReg(0x7D, mask);
}

void TMC4671::setStatusFlags(uint32_t flags){
	writeReg(0x7C, flags);
}

void TMC4671::setStatusFlags(StatusFlags flags){
	writeReg(0x7C, flags.asInt);
}

/**
 * Reads and resets all status flags and executes depending on status flags
 */
void TMC4671::statusCheck(){
	flagCheckInProgress = true;
	statusFlags = readFlags(); // Update current flags

	// encoder index flag was set since last check. Check if the flag matching the current encoder is set
	if( (statusFlags.flags.ENC_N && this->conf.motconf.enctype == EncoderType_TMC::abn) || (statusFlags.flags.AENC_N && this->conf.motconf.enctype == EncoderType_TMC::sincos) ){
		encoderIndexHit();
	}

	if(statusFlags.flags.not_PLL_locked){
		// Critical error. PLL not locked
		// Creating error object not allowed. Function is called from flag isr! ignore for now.
		//ErrorHandler::addError(Error(ErrorCode::tmcPLLunlocked, ErrorType::critical, "TMC PLL not locked"));
	}


	setStatusFlags(0); // Reset flags
	if(readFlags().asInt != statusFlags.asInt){ // Condition is cleared. if not we will reset it in the main loop later to get out of the isr and cause some delay
		flagCheckInProgress = false;
	}
}

void TMC4671::exti(uint16_t GPIO_Pin){
	if(GPIO_Pin == FLAG_Pin && !flagCheckInProgress){ // Flag pin went high and flag check is currently not in progress (prevents interrupt flooding)
		statusCheck(); // In isr!
	}
}

void TMC4671::encoderIndexHit(){
	//pulseClipLed();
//	if(zeroEncoderOnIndexHit){
//		writeReg(0x27, 0);
//	}
	setEncoderIndexFlagEnabled(false,false); // Found the index. disable flag
	encoderIndexHitFlag = true;
}

TMC4671MotConf TMC4671::decodeMotFromInt(uint16_t val){
	// 0-2: MotType 3-5: Encoder source 6-15: Poles
	TMC4671MotConf mot;
	mot.motor_type = MotorType(val & 0x3);
	mot.svpwm = !(val & 0x4);
	mot.enctype = EncoderType_TMC( (val >> 3) & 0x7);
	mot.pole_pairs = val >> 6;
	return mot;
}
uint16_t TMC4671::encodeMotToInt(TMC4671MotConf mconf){
	uint16_t val = (uint8_t)mconf.motor_type & 0x3;
	val |= !mconf.svpwm ? 0x4 : 0;
	val |= ((uint8_t)mconf.enctype & 0x7) << 3;
	val |= (mconf.pole_pairs & 0x3FF) << 6;
	return val;
}

uint16_t TMC4671::encodeEncHallMisc(){
	uint16_t val = 0;
	val |= (this->abnconf.npol) & 0x01;
	val |= (this->conf.encoderReversed & 0x01)  << 1; // Direction


	val |= (this->abnconf.ab_as_n & 0x01) << 2;
	val |= (this->pidPrecision.current_I) << 3;
	val |= (this->pidPrecision.current_P) << 4;

	val |= (this->abnconf.useIndex) << 5;

	val |= (this->conf.combineEncoder) << 6;
	val |= (this->conf.invertForce) << 7;

	val |= ((this->conf.enableFluxDissipation & 0x01) << 8);
	val |= (this->hallconf.interpolation & 0x01) << 9;

	val |= (this->curPids.sequentialPI & 0x01) << 10;

	//11,12,13,14,15 hw version
	val |= ((uint8_t)this->conf.hwconf.hwVersion & 0x1F) << 11;

	return val;
}

void TMC4671::restoreEncHallMisc(uint16_t val){

	this->abnconf.apol = (val) & 0x01;

	this->abnconf.bpol = this->abnconf.apol;
	this->abnconf.npol = this->abnconf.apol;

	this->conf.encoderReversed = (val>>1) & 0x01;// Direction
	this->abnconf.rdir = this->conf.encoderReversed;
	this->aencconf.rdir = this->conf.encoderReversed;
	this->hallconf.direction = this->conf.encoderReversed;

	this->abnconf.ab_as_n = (val>>2) & 0x01;
	this->pidPrecision.current_I = (val>>3) & 0x01;
	this->pidPrecision.velocity_I = this->pidPrecision.current_I;
	this->pidPrecision.position_I = this->pidPrecision.current_I;
	this->pidPrecision.current_P = (val>>4) & 0x01;
	this->pidPrecision.velocity_P = this->pidPrecision.current_P;
	this->pidPrecision.velocity_P = this->pidPrecision.current_P;

	this->abnconf.useIndex = (val>>5) & 0x01;
	this->conf.combineEncoder = (val>>6) & 0x01;
	this->conf.invertForce = ((val>>7) & 0x01) && this->conf.combineEncoder;

	this->conf.enableFluxDissipation = ((val>>8) & 0x01);
	this->hallconf.interpolation = (val>>9) & 0x01;
	this->curPids.sequentialPI = (val>>10) & 0x01;

	setHwType((uint8_t)((val >> 11) & 0x1F));

}



/**
 * Sets some constants and features depending on the hardware version of the driver
 */
void TMC4671::setHwType(uint8_t type){
	// If only one config is valid use this regardless of requested type
	if(TMC4671::tmc4671_hw_configs.size() == 1){
		this->conf.hwconf = TMC4671::tmc4671_hw_configs[0];
	}else{ // Search for config matching requested type
		for(const TMC4671HardwareTypeConf& newConf : TMC4671::tmc4671_hw_configs){
			if(type == newConf.hwVersion){
				this->conf.hwconf = newConf;
				break;
			}
		}
	}

	setVSenseMult(this->conf.hwconf.vSenseMult); // Update vsense multiplier
	//setupBrakePin(vdiffAct, vdiffDeact, vMax); // TODO if required
	setBrakeLimits(this->conf.hwconf.brakeLimLow,this->conf.hwconf.brakeLimHigh);
	setBBM(this->conf.hwconf.bbm,this->conf.hwconf.bbm);
	// Force changing motor and encoder types to prevent invalid types being selected if new hw type does not support them
	setMotorType(this->conf.motconf.motor_type, this->conf.motconf.pole_pairs);
	setEncoderType(this->conf.motconf.enctype);
}

/**
 * Appends a formatted reply with currently available hardware version configs
 */
void TMC4671::replyHardwareVersions(const std::span<const TMC4671HardwareTypeConf>& versions,std::vector<CommandReply>& replies){
//	uint8_t idx = 0;
	for(const TMC4671HardwareTypeConf& c : versions){
		if(this->canChangeHwType || c.hwVersion == this->conf.hwconf.hwVersion){
			replies.emplace_back( std::to_string((uint8_t)c.hwVersion) + ":" + c.name,(uint8_t)c.hwVersion);
		}
	}
}

void TMC4671::registerCommands(){
	CommandHandler::registerCommands();

	registerCommand("cpr", TMC4671_commands::cpr, "CPR in TMC",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("mtype", TMC4671_commands::mtype, "Motor type",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("encsrc", TMC4671_commands::encsrc, "Encoder source",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("tmcHwType", TMC4671_commands::tmcHwType, "Version of TMC board",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("encalign", TMC4671_commands::encalign, "Align encoder",CMDFLAG_GET);
	registerCommand("poles", TMC4671_commands::poles, "Motor pole pairs",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("acttrq", TMC4671_commands::acttrq, "Measure torque, flux, cogging",CMDFLAG_GET);
	registerCommand("pwmlim", TMC4671_commands::pwmlim, "PWM limit",CMDFLAG_DEBUG | CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("torqueP", TMC4671_commands::torqueP, "Torque P",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("torqueI", TMC4671_commands::torqueI, "Torque I",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("fluxP", TMC4671_commands::fluxP, "Flux P",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("fluxI", TMC4671_commands::fluxI, "Flux I",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("velocityP", TMC4671_commands::velocityP, "Velocity P",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("velocityI", TMC4671_commands::velocityI, "Velocity I",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("posP", TMC4671_commands::posP, "Pos P",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("posI", TMC4671_commands::posI, "Pos I",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("tmctype", TMC4671_commands::tmctype, "Version of TMC chip",CMDFLAG_GET);
	registerCommand("pidPrec", TMC4671_commands::pidPrec, "PID precision bit0=I bit1=P. 0=Q8.8 1= Q4.12",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("phiesrc", TMC4671_commands::phiesrc, "PhiE source",CMDFLAG_DEBUG | CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("fluxoffset", TMC4671_commands::fluxoffset, "Offset flux scale for field weakening",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("seqpi", TMC4671_commands::seqpi, "Sequential PI",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("iScale", TMC4671_commands::tmcIscale, "Counts per A",CMDFLAG_STR_ONLY);
	registerCommand("encdir", TMC4671_commands::encdir, "Encoder dir",CMDFLAG_DEBUG | CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("abnpol", TMC4671_commands::encpol, "Encoder polarity",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("temp", TMC4671_commands::temp, "Temperature in C * 100",CMDFLAG_GET);
	registerCommand("reg", TMC4671_commands::reg, "Read or write a TMC register at adr",CMDFLAG_DEBUG | CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("svpwm", TMC4671_commands::svpwm, "Space-vector PWM",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("autohome", TMC4671_commands::findIndex, "Find abn index",CMDFLAG_GET);
	registerCommand("abnindex", TMC4671_commands::abnindexenabled, "Enable ABN index",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("calibrate", TMC4671_commands::fullCalibration, "Full calibration",CMDFLAG_GET);
	registerCommand("calibrated", TMC4671_commands::calibrated, "Calibration valid",CMDFLAG_GET);
	registerCommand("state", TMC4671_commands::getState, "Get state",CMDFLAG_GET);
	registerCommand("combineEncoder", TMC4671_commands::combineEncoder, "Use TMC for movement. External encoder for position",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("invertForce", TMC4671_commands::invertForce, "Invert incoming forces",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("vm", TMC4671_commands::vmTmc, "VM in mV",CMDFLAG_GET);
	registerCommand("extphie", TMC4671_commands::extphie, "external phie",CMDFLAG_GET);
	registerCommand("trqbq_mode", TMC4671_commands::torqueFilter_mode, "Torque filter mode: none;lowpass;notch;peak",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("trqbq_f", TMC4671_commands::torqueFilter_f, "Torque filter freq 1000 max. 0 to disable. (Stored f/2)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("trqbq_q", TMC4671_commands::torqueFilter_q, "Torque filter q*100",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pidautotune", TMC4671_commands::pidautotune, "Start PID autoruning",CMDFLAG_GET);
	registerCommand("fluxbrake", TMC4671_commands::fluxbrake, "Prefer energy dissipation in motor",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("pwmfreq", TMC4671_commands::pwmfreq, "Get/set pwm frequency",CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_DEBUG);
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	registerCommand("cogging", TMC4671_commands::cogging, "Get/Set the cogging compensation",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("calibrateCogging", TMC4671_commands::calibrateCogging, "Cogging calibration",CMDFLAG_GET);
	registerCommand("coggingTable", TMC4671_commands::coggingTable, "Get the cogging table, or clear table (set 0)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingScale", TMC4671_commands::coggingScale, "Cogging compensation scale (0-100)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingSpeedP", TMC4671_commands::coggingSpeedP, "Manual speed loop P gain for cogging",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingSpeedI", TMC4671_commands::coggingSpeedI, "Manual speed loop I gain for cogging",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingHarmonics", TMC4671_commands::coggingHarmonics, "Get full harmonic table (adr 0=base, 1=RPM2, 2=RPM3)",CMDFLAG_GET | CMDFLAG_GETADR);
	registerCommand("coggingCwCcw", TMC4671_commands::coggingCwCcw, "Get CW and CCW raw harmonics", CMDFLAG_GET);
	registerCommand("coggingSave", TMC4671_commands::coggingSave, "Save cogging table to flash",CMDFLAG_GET);
	registerCommand("coggingShape", TMC4671_commands::coggingShape, "Waveshaping factor (*100)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingSpeedD", TMC4671_commands::coggingSpeedD, "Speed D gain for cogging",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("scaleCurve", TMC4671_commands::scaleCurve, "Get/Set scale curve values",CMDFLAG_GET | CMDFLAG_SETADR);
	registerCommand("phaseAdvCurve", TMC4671_commands::phaseAdvCurve, "Get/Set phase advance curve (degrees per RPM)",CMDFLAG_GET | CMDFLAG_SETADR);
	registerCommand("coggingH3", TMC4671_commands::coggingH3, "Cogging waveshaping: get 'shaping:phaseTrim:mult' or set adr 0=shaping(*1000) 1=phaseTrim(mdeg) 2=mult",CMDFLAG_GET | CMDFLAG_SETADR);
	registerCommand("coggingCalibCount", TMC4671_commands::coggingCalibCount, "Get number of calibration profiles",CMDFLAG_GET);
	registerCommand("coggingCalibRPM", TMC4671_commands::coggingCalibRPM, "Get/Set RPM target per calibration profile (adr=idx, val=RPM*10)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("coggingCalibIters", TMC4671_commands::coggingCalibIters, "Get/Set iterations per calibration profile (adr=idx)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("coggingCalibPidP", TMC4671_commands::coggingCalibPidP, "Get/Set manual P gain per calibration profile (adr=idx, val=PID*1000)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("coggingCalibPidI", TMC4671_commands::coggingCalibPidI, "Get/Set manual I gain per calibration profile (adr=idx, val=PID*1000)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("coggingCalibPidD", TMC4671_commands::coggingCalibPidD, "Get/Set manual D gain per calibration profile (adr=idx, val=PID*1000)",CMDFLAG_GETADR | CMDFLAG_SETADR);
	registerCommand("coggingCalibAutoPid", TMC4671_commands::coggingCalibAutoPid, "Get/Set auto PID tune flag (1=auto, 0=manual)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingCalibInertiaCorr", TMC4671_commands::coggingCalibInertiaCorr, "Get/Set inertia acceleration correction during DFT (1=on,0=off)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingCalibFrictionFF", TMC4671_commands::coggingCalibFrictionFF, "Get/Set friction feedforward during DFT (1=on,0=off)",CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("coggingBins", TMC4671_commands::coggingBins, "Get spatial bin snapshots (adr 0=CW 1=CCW 2=verCW 3=verCCW 4=verCWharm 5=verCCWharm)",CMDFLAG_GET | CMDFLAG_GETADR);
	registerCommand("coggingFFMode", TMC4671_commands::coggingFFMode, "FF source: 0=harmonics 1=combined bins 2=per-direction bins",CMDFLAG_GET | CMDFLAG_SET);
#endif
}


CommandStatus TMC4671::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	// During cogging calibration, block all poll commands to avoid SPI contention
	// and serial log flooding. Only calibration progress broadcasts are allowed through.
	if (state == TMC_ControlState::CoggingCalibration && static_cast<TMC4671_commands>(cmd.cmdId) != TMC4671_commands::calibrateCogging) {
		return CommandStatus::NO_REPLY;
	}
#endif
	CommandStatus status = CommandStatus::OK;
	switch(static_cast<TMC4671_commands>(cmd.cmdId)){
	case TMC4671_commands::combineEncoder:
		status = handleGetSet(cmd, replies, this->conf.combineEncoder);
		if(!this->conf.combineEncoder){
			this->conf.invertForce = false; // Force off
		}

		break;

	case TMC4671_commands::cpr:
		handleGetFuncSetFunc(cmd, replies, &TMC4671::getEncCpr, &TMC4671::setCpr, this);
	break;

	case TMC4671_commands::invertForce:
		handleGetSet(cmd, replies, conf.invertForce);
		break;

	case TMC4671_commands::getState:
		replies.emplace_back((uint32_t)getState());
		break;

	case TMC4671_commands::fullCalibration:
		calibrationFailCount = 1; // allow 1 fail
		changeState(TMC_ControlState::FullCalibration);
		// TODO start full calibration and save in flash
		break;

	case TMC4671_commands::calibrated:
		replies.emplace_back(!recalibrationRequired && adcCalibrated);
		break;

	case TMC4671_commands::mtype:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->conf.motconf.motor_type);
		}else if(cmd.type == CMDtype::set && (uint8_t)cmd.val <= (uint8_t)MotorType::BLDC){
			this->setMotorType((MotorType)cmd.val, this->conf.motconf.pole_pairs);
		}else{
			std::string rplstr = "";
			TMC4671HardwareTypeConf::SupportedModes_s* confflags = &conf.hwconf.flags;
			if(confflags->mot_none) rplstr += "NONE=0,";
			if(confflags->mot_dc) rplstr += "DC=1,";
			if(confflags->mot_stepper) rplstr += "Stepper 2Ph=2,";
			if(confflags->mot_bldc) rplstr += "BLDC 3Ph=3";
			replies.emplace_back(rplstr);
		}
		break;

	case TMC4671_commands::encsrc:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->conf.motconf.enctype);
		}else if(cmd.type == CMDtype::set){
			this->setEncoderType((EncoderType_TMC)cmd.val);
		}else{
			std::string rplstr = "";
			TMC4671HardwareTypeConf::SupportedModes_s* confflags = &conf.hwconf.flags;
			if(confflags->enc_none) rplstr += "NONE=0,";
			if(confflags->enc_abn) rplstr += "ABN=1,";
			if(confflags->enc_sincos) rplstr += "SinCos=2,";
			if(confflags->enc_uvw) rplstr += "UVW=3,";
			if(confflags->enc_hall) rplstr += "HALL=4,";
			if(confflags->enc_ext && externalEncoderAllowed()) rplstr += "External=5";
			replies.emplace_back(rplstr);
		}
		break;

	case TMC4671_commands::tmcHwType:
		if(cmd.type == CMDtype::get){
			replies.push_back((uint8_t)conf.hwconf.hwVersion);
		}else if(cmd.type == CMDtype::set){
			if(canChangeHwType)
				setHwType((uint8_t)(cmd.val & 0x1F));
		}else{
			// List known hardware versions
			replyHardwareVersions(tmc4671_hw_configs, replies);
		}
		break;

	case TMC4671_commands::encalign:
		if(cmd.type == CMDtype::get){
			encoderAligned = false;
			this->setEncoderType(this->conf.motconf.enctype);
			manualEncAlign = true;
			return CommandStatus::NO_REPLY;
		}else{
			return CommandStatus::ERR;
		}
		break;

	case TMC4671_commands::poles:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->conf.motconf.pole_pairs);
		}else if(cmd.type == CMDtype::set){
			this->setMotorType(this->conf.motconf.motor_type,cmd.val);
		}
		break;

	case TMC4671_commands::acttrq:
		if(cmd.type == CMDtype::get){
			std::pair<int32_t,int32_t> current = getActualTorqueFlux();
			std::string replyStr = std::to_string(current.second) + ":" + std::to_string(current.first);
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
			replyStr += ":" + std::to_string(this->last_anticogging_torque);
			// Append scale then position then pre-cogging iqCmd setpoint then RPM
			replyStr += ":" + std::to_string((int16_t)(this->last_cogging_scale * 100.0f));
			replyStr += ":" + std::to_string((int32_t)(this->getFilteredPosition() * 10000.0f));
			replyStr += ":" + std::to_string((int16_t)(this->last_power_setpoint));
			replyStr += ":" + std::to_string((int16_t)(this->measured_rpm));
#else
			replyStr += ":" + std::to_string((int16_t)(this->last_power_setpoint));
#endif
			replies.emplace_back(CommandReply(replyStr, current.second, current.first));
		}
		break;

	case TMC4671_commands::pwmlim:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->curLimits.pid_uq_ud);
		}else if(cmd.type == CMDtype::set){
			this->setUqUdLimit(cmd.val);
		}
		break;

	case TMC4671_commands::torqueP:
		handleGetSet(cmd, replies, this->curPids.torqueP);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::torqueI:
		handleGetSet(cmd, replies, this->curPids.torqueI);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::fluxP:
		handleGetSet(cmd, replies, this->curPids.fluxP);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::fluxI:
		handleGetSet(cmd, replies, this->curPids.fluxI);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::velocityP:
		handleGetSet(cmd, replies, this->curPids.velocityP);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::velocityI:
		handleGetSet(cmd, replies, this->curPids.velocityI);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::posP:
		handleGetSet(cmd, replies, this->curPids.positionP);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::posI:
		handleGetSet(cmd, replies, this->curPids.positionI);
		if(cmd.type == CMDtype::set)
			setPids(curPids);
		break;

	case TMC4671_commands::abnindexenabled:
		handleGetSet(cmd, replies, this->abnconf.useIndex);
		if(cmd.type == CMDtype::set)
			setup_ABN_Enc(abnconf);
		break;

	case TMC4671_commands::tmctype:
	{
		std::pair<uint32_t,std::string> ver = getTmcType();
		replies.emplace_back(ver.second,ver.first);
		break;
	}

	case TMC4671_commands::vmTmc:
		replies.emplace_back(getTmcVM());
		break;

	case TMC4671_commands::pidPrec:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->pidPrecision.current_I | (this->pidPrecision.current_P << 1));
		}else if(cmd.type == CMDtype::set){
			this->pidPrecision.current_I = cmd.val & 0x1;
			this->pidPrecision.current_P = (cmd.val >> 1) & 0x1;
			this->setPidPrecision(pidPrecision);
		}
		break;
	case TMC4671_commands::phiesrc:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->getPhiEtype());
		}else if(cmd.type == CMDtype::set){
			this->setPhiEtype((PhiE)cmd.val);
		}else{
			replies.emplace_back("ext=1,openloop=2,abn=3,hall=5,aenc=6,aencE=7");
		}
		break;
	case TMC4671_commands::fluxoffset:
		handleGetSet(cmd, replies, maxOffsetFlux);
		break;
	case TMC4671_commands::seqpi:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->curPids.sequentialPI);
		}else if(cmd.type == CMDtype::set){
			this->setSequentialPI(cmd.val != 0);
		}
		break;
	case TMC4671_commands::tmcIscale:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(std::to_string(this->conf.hwconf.currentScaler)); // TODO float as value?
		}
		break;
	case TMC4671_commands::encdir:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->abnconf.rdir);
		}else if(cmd.type == CMDtype::set){
			this->abnconf.rdir = cmd.val != 0;
			this->setup_ABN_Enc(this->abnconf);
		}
		break;

	case TMC4671_commands::encpol:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->abnconf.npol);
		}else if(cmd.type == CMDtype::set){
			this->abnconf.npol = cmd.val != 0;
			this->abnconf.apol = cmd.val != 0;
			this->abnconf.bpol = cmd.val != 0;
			this->setup_ABN_Enc(this->abnconf);
		}
		break;
	case TMC4671_commands::temp:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((int32_t)(this->getTemp()*100.0));
		}
		break;
	case TMC4671_commands::reg:
		if(cmd.type == CMDtype::getat){
			replies.emplace_back(readReg(cmd.val));
		}else if(cmd.type == CMDtype::setat){
			writeReg(cmd.adr,cmd.val);
		}else{
			return CommandStatus::ERR;
		}
		break;

	case TMC4671_commands::findIndex:
		changeState(TMC_ControlState::IndexSearch);
		break;

	case TMC4671_commands::svpwm:
	{
		if(cmd.type == CMDtype::set){
			setSvPwm(cmd.val != 0);
		}else if(cmd.type == CMDtype::get){
			replies.emplace_back(conf.motconf.svpwm);
		}
		break;
	}
	case TMC4671_commands::extphie:
	{

		replies.emplace_back(getPhiEfromExternalEncoder());

		break;
	}
	case TMC4671_commands::torqueFilter_mode:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint8_t)this->torqueFilterConf.mode);
		}else if(cmd.type == CMDtype::set && (uint8_t)cmd.val < 4){
			torqueFilterConf.mode = (TMCbiquadpreset)(cmd.val);
			this->setTorqueFilter(torqueFilterConf);
		}else{
			replies.emplace_back("OFF=0,Lowpass=1,Notch=2,Peak=3");
		}
		break;
	case TMC4671_commands::torqueFilter_f:
	{
		if(cmd.type == CMDtype::set){
				torqueFilterConf.params.freq = clip(cmd.val,1,0x1fff);
				this->setTorqueFilter(torqueFilterConf);
			}else if(cmd.type == CMDtype::get){
				replies.emplace_back(torqueFilterConf.params.freq);
			}
		break;
	}

	case TMC4671_commands::torqueFilter_q:
		if(cmd.type == CMDtype::set){
			torqueFilterConf.params.q = clip(cmd.val,0,127);
				this->setTorqueFilter(torqueFilterConf);
			}else if(cmd.type == CMDtype::get){
				replies.emplace_back(torqueFilterConf.params.q);
			}
		break;
	case TMC4671_commands::pidautotune:
		changeState(TMC_ControlState::Pidautotune);
		return CommandStatus::NO_REPLY;

	case TMC4671_commands::fluxbrake:
		handleGetSet(cmd, replies, conf.enableFluxDissipation);
		break;

	case TMC4671_commands::pwmfreq:
		if(cmd.type == CMDtype::set){
				setPwmFreq(cmd.val);
			}else if(cmd.type == CMDtype::get){
				replies.emplace_back(getPwmFreq());
			}
		break;

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	case TMC4671_commands::cogging:
		handleGetSet(cmd, replies, cogging_enabled);
		break;
	
	case TMC4671_commands::calibrateCogging:

		if (emergency && hasPower()) {
			replies.emplace_back("Calibration aborted: Emergency stop engaged", 0);
		} else if (conf.motconf.motor_type == MotorType::NONE) {
			replies.emplace_back("Calibration aborted: No motor type set", 0);
		} else {
			changeState(TMC_ControlState::CoggingCalibration);
			return CommandStatus::NO_REPLY;
		}

		break;


	case TMC4671_commands::coggingTable:
		if(cmd.type == CMDtype::get){
			std::string s;
			s.reserve(512);
			s = "item:0,data:(";

			for (uint16_t order = 0; order < COGGING_CALIB_DFT_HARMONICS; order++) {
				
				int16_t val = 0;
				for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
					if (cogging_harmonics[n].order == order && cogging_harmonics[n].amplitude > 0.0f) {
						val = (int16_t)cogging_harmonics[n].amplitude;
						break;
					}
				}

				s.append(std::to_string(val));

				if (s.length() >= 500) { // send data by string cut at 500 items
					s.append(")");
					CommandHandler::broadcastCommandReply(CommandReply(s,0), (uint32_t)TMC4671_commands::coggingTable, CMDtype::get);
					s.clear();
					s = "item:" + std::to_string(order+1) + ",data:(";
				} else {
					if (order < COGGING_CALIB_DFT_HARMONICS - 1) {
						s.append(",");
					}
				}
			}

			s.append(")");
			CommandHandler::broadcastCommandReply(CommandReply(s,0), (uint32_t)TMC4671_commands::coggingTable, CMDtype::get);
			return CommandStatus::NO_REPLY;
		} else if(cmd.type == CMDtype::set && cmd.val == 0) {
			clearCoggingTable();
		} else {
			status = CommandStatus::ERR;
		}
		break;

	case TMC4671_commands::coggingScale:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int16_t)(this->cogging_scale * 10000.0f));
		} else if (cmd.type == CMDtype::set) {
			this->cogging_scale = (float)(int16_t)cmd.val / 10000.0f;
		}
		break;

	case TMC4671_commands::coggingSpeedP:
		handleGetSet(cmd, replies, this->coggingSpeedP);
		break;

	case TMC4671_commands::coggingSpeedI:
		handleGetSet(cmd, replies, this->coggingSpeedI);
		break;

	case TMC4671_commands::coggingSave:
		if (cmd.type == CMDtype::get) {
			saveCoggingTable();
			replies.emplace_back(1);
		}
		break;

	case TMC4671_commands::coggingShape:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int16_t)(this->coggingShape * 100.0f));
		} else if (cmd.type == CMDtype::set) {
			this->coggingShape = clip<float>((float)(int16_t)cmd.val / 100.0f, 0.0f, 10.0f);
		}
		break;

	case TMC4671_commands::coggingSpeedD:
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((int16_t)(this->coggingSpeedD));
		} else if (cmd.type == CMDtype::set) {
			this->coggingSpeedD = clip<float>((float)(int16_t)cmd.val, 0.0f, 100000.0f);
		}
		break;

	case TMC4671_commands::scaleCurve:
		if (cmd.type == CMDtype::get) {
			std::string s;
			for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
				if (i > 0) s += ",";
				s += std::to_string((int16_t)(scale_curve_rpm_points[i])) + ":";
				// Return effective default (1.0) when curve not calibrated yet
				if (scale_curve_valid) {
					s += std::to_string((int16_t)(scale_curve_values[i] * 1000.0f));
				} else {
					s += "1000";
				}
			}
			replies.emplace_back(s);
		} else if (cmd.type == CMDtype::setat) {
			uint8_t idx = (uint8_t)cmd.adr;
			if (idx < SCALE_CURVE_POINTS) {
				scale_curve_values[idx] = (float)(int16_t)cmd.val / 1000.0f;
				scale_curve_count = SCALE_CURVE_POINTS;
				scale_curve_valid = true;
			}
		}
		break;

	case TMC4671_commands::phaseAdvCurve:
		if (cmd.type == CMDtype::get) {
			// Format identical to scaleCurve: "rpm:deg100,rpm:deg100,..."
			// Phase advance stored as degrees * 100 for 0.01 deg resolution.
			std::string s;
			for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
				if (i > 0) s += ",";
				s += std::to_string((int16_t)(scale_curve_rpm_points[i])) + ":";
				// Return 0 when curve not calibrated yet
				if (phase_adv_curve_valid) {
					s += std::to_string((int16_t)(phase_advance_curve_values[i] * 100.0f));
				} else {
					s += "0";
				}
			}
			replies.emplace_back(s);
		} else if (cmd.type == CMDtype::setat) {
			uint8_t idx = (uint8_t)cmd.adr;
			if (idx < SCALE_CURVE_POINTS) {
				phase_advance_curve_values[idx] = (float)(int16_t)cmd.val / 100.0f;
				phase_adv_curve_valid = true;
			}
		}
		break;

	case TMC4671_commands::coggingH3:
		// Cogging waveshaping. adr/index 0 = shaping (*1000, signed),
		// 1 = phase trim (millirad, signed), 2 = mult (1..31).
		// adr=3: clear harmonic table (val: 0=base, 1=rpm2, 2=rpm3).
		// adr=4: set harmonic (val = slot<<24 | order<<16 | amplitude).
		// adr=5: set harmonic phase (val = slot<<24 | phase_mrad).
		if (cmd.type == CMDtype::get) {
			std::string s;
			s += std::to_string((int16_t)(this->h3_shaping * 1000.0f)) + ":";
			s += std::to_string((int16_t)(this->h3_phase_trim * 1000.0f)) + ":";
			s += std::to_string((uint16_t)this->h3_mult);
			replies.emplace_back(s);
		} else if (cmd.type == CMDtype::setat) {
			uint8_t idx = (uint8_t)cmd.adr;
			if (idx == 0) {
				this->h3_shaping = (float)(int16_t)cmd.val / 1000.0f;
			} else if (idx == 1) {
				this->h3_phase_trim = (float)(int16_t)cmd.val / 1000.0f;
			} else if (idx == 2) {
				uint16_t m = (uint16_t)cmd.val;
				if (m >= 1 && m <= 31) this->h3_mult = m;
			} else if (idx == 3) {
				// Clear harmonic table
				uint8_t table_idx = (uint8_t)cmd.val;
				Harmonic* tbl = cogging_harmonics;
				if (table_idx == 1) { tbl = cogging_harmonics_rpm2; rpm2_table_valid = false; }
				if (table_idx == 2) { tbl = cogging_harmonics_rpm3; rpm3_table_valid = false; }
				memset(tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
			} else if (idx == 4) {
				// Set harmonic amplitude+order: slot<<24 | order<<16 | amplitude
				uint8_t slot = (uint8_t)((cmd.val >> 24) & 0xFF);
				uint16_t order = (uint16_t)((cmd.val >> 16) & 0xFF);
				int16_t amp = (int16_t)(cmd.val & 0xFFFF);
				// Write to base table only (extend later if needed)
				if (slot < COGGING_HARMONICS_COUNT && order > 0) {
					cogging_harmonics[slot].order = order;
					cogging_harmonics[slot].amplitude = (float)amp;
					// keep existing phase if any
				}
			} else if (idx == 5) {
				// Set harmonic phase: slot<<24 | phase_mrad
				uint8_t slot = (uint8_t)((cmd.val >> 24) & 0xFF);
				int16_t phase_mrad = (int16_t)(cmd.val & 0xFFFF);
				if (slot < COGGING_HARMONICS_COUNT) {
					cogging_harmonics[slot].phase = (float)phase_mrad / 1000.0f;
				}
			}
		}
		break;

	case TMC4671_commands::coggingHarmonics:
		if(cmd.type == CMDtype::get){
			// Select table by adr: 0=cogging_harmonics, 1=rpm2, 2=rpm3
			Harmonic* tbl = cogging_harmonics;
			if (cmd.adr == 1 && rpm2_table_valid) tbl = cogging_harmonics_rpm2;
			if (cmd.adr == 2 && rpm3_table_valid) tbl = cogging_harmonics_rpm3;

			// Send full harmonic table: "order:amplitude:phase,..." (up to 20 harmonics)
			std::string s;
			s.reserve(512);
			bool first = true;
			for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
				if (tbl[i].amplitude > 0.0f || tbl[i].order > 0) {
					if (!first) s += ",";
					first = false;
					s += std::to_string(tbl[i].order) + ":";
					s += std::to_string((int16_t)tbl[i].amplitude) + ":";
					s += std::to_string((int16_t)(tbl[i].phase * 1000.0f));
				}
			}
			if (first) {
				s = "0:0:0"; // Empty table
			}
			CommandHandler::broadcastCommandReply(CommandReply(s,cmd.adr), (uint32_t)TMC4671_commands::coggingHarmonics, CMDtype::get);
			return CommandStatus::NO_REPLY;
		}
		break;

	case TMC4671_commands::coggingCwCcw:
		if(cmd.type == CMDtype::get){
			// Return CW and CCW separate harmonic tables
			// Format: "CW:order:amp:phase,...|CCW:order:amp:phase,..."
			std::string s;
			s.reserve(512);

			if (cwccw_data_valid) {
				s += "CW:";
				bool first = true;
				for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
					if (cw_store[i].amplitude > 0.0f) {
						if (!first) s += ",";
						first = false;
						s += std::to_string(cw_store[i].order) + ":";
						s += std::to_string((int16_t)cw_store[i].amplitude) + ":";
						s += std::to_string((int16_t)(cw_store[i].phase * 1000.0f));
					}
				}
				if (first) s += "0:0:0";

				s += "|CCW:";
				first = true;
				for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
					if (ccw_store[i].amplitude > 0.0f) {
						if (!first) s += ",";
						first = false;
						s += std::to_string(ccw_store[i].order) + ":";
						s += std::to_string((int16_t)ccw_store[i].amplitude) + ":";
						s += std::to_string((int16_t)(ccw_store[i].phase * 1000.0f));
					}
				}
				if (first) s += "0:0:0";
			} else {
				s = "CW:0:0:0|CCW:0:0:0";
			}
			CommandHandler::broadcastCommandReply(CommandReply(s,0), (uint32_t)TMC4671_commands::coggingCwCcw, CMDtype::get);
			return CommandStatus::NO_REPLY;
		}
		break;
		break;

	case TMC4671_commands::coggingCalibCount:
		if(cmd.type == CMDtype::get){
			replies.emplace_back((uint32_t)this->cogging_calib_count);
		}
		break;

	case TMC4671_commands::coggingCalibRPM:
		if(cmd.type == CMDtype::getat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				replies.emplace_back((int16_t)(this->cogging_calib_rpm[idx] * 10.0f));
		} else if(cmd.type == CMDtype::setat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				this->cogging_calib_rpm[idx] = (float)(int16_t)cmd.val / 10.0f;
		}
		break;

	case TMC4671_commands::coggingCalibIters:
		if(cmd.type == CMDtype::getat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				replies.emplace_back((uint32_t)this->cogging_calib_iters[idx]);
		} else if(cmd.type == CMDtype::setat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				this->cogging_calib_iters[idx] = (uint16_t)cmd.val;
		}
		break;

	case TMC4671_commands::coggingCalibPidP:
		if(cmd.type == CMDtype::getat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				replies.emplace_back((uint32_t)this->cogging_calib_pidP[idx]);
		} else if(cmd.type == CMDtype::setat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES){
				this->cogging_calib_pidP[idx] = cmd.val;
				if(idx == 0) this->coggingSpeedP = (float)cmd.val;
			}
		}
		break;

	case TMC4671_commands::coggingCalibPidI:
		if(cmd.type == CMDtype::getat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				replies.emplace_back((uint32_t)this->cogging_calib_pidI[idx]);
		} else if(cmd.type == CMDtype::setat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES){
				this->cogging_calib_pidI[idx] = cmd.val;
				if(idx == 0) this->coggingSpeedI = (float)cmd.val;
			}
		}
		break;

	case TMC4671_commands::coggingCalibPidD:
		if(cmd.type == CMDtype::getat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES)
				replies.emplace_back((uint32_t)this->cogging_calib_pidD[idx]);
		} else if(cmd.type == CMDtype::setat){
			uint8_t idx = (uint8_t)cmd.adr;
			if(idx < COGGING_MAX_CALIB_PROFILES){
				this->cogging_calib_pidD[idx] = cmd.val;
				if(idx == 0) this->coggingSpeedD = (float)cmd.val;
			}
		}
		break;

	case TMC4671_commands::coggingCalibAutoPid:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->cogging_calib_autoPid ? 1 : 0);
		} else if(cmd.type == CMDtype::set){
			this->cogging_calib_autoPid = (cmd.val != 0);
		}
		break;

	case TMC4671_commands::coggingCalibInertiaCorr:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->cogging_calib_inertiaCorr ? 1 : 0);
		} else if(cmd.type == CMDtype::set){
			this->cogging_calib_inertiaCorr = (cmd.val != 0);
		}
		break;

	case TMC4671_commands::coggingCalibFrictionFF:
		if(cmd.type == CMDtype::get){
			replies.emplace_back(this->cogging_calib_frictionFF ? 1 : 0);
		} else if(cmd.type == CMDtype::set){
			this->cogging_calib_frictionFF = (cmd.val != 0);
		}
		break;

	case TMC4671_commands::coggingBins:
		// Chunked spatial-bin readout for the configurator.
		// adr: 0=CW bins, 1=CCW bins, 2=verCW bins, 3=verCCW bins,
		//      4=verCW top-20 DFT, 5=verCCW top-20 DFT.
		// Bins are sent as "B<adr>:item:<offset>,data:(v0,v1,...)" chunks of ~80
		// values so each reply stays well under the serial line limit. The
		// configurator routes chunks by the B<adr>: prefix.
		// Top-20 DFT uses "B<adr>:order:amp:phase,...".
		if(cmd.type == CMDtype::getat || cmd.type == CMDtype::get){
			if(!this->bins_data_valid){
				CommandHandler::broadcastCommandReply(CommandReply("NOBINS", cmd.adr),
					(uint32_t)TMC4671_commands::coggingBins, CMDtype::get);
				return CommandStatus::NO_REPLY;
			}
			uint8_t adr = (uint8_t)cmd.adr;
			if(adr <= 3){
				// Bin array readout — chunked.
				const float* src = nullptr;
				switch(adr){
					case 0: src = this->cw_bins; break;
					case 1: src = this->ccw_bins; break;
					case 2: src = this->ver_cw_bins; break;
					case 3: src = this->ver_ccw_bins; break;
				}
				const uint32_t CHUNK = 80;
				for(uint32_t off = 0; off < COGGING_DFT_BIN_COUNT; off += CHUNK){
					std::string s = "B";
					s += std::to_string(adr);
					s += ":item:";
					s += std::to_string(off);
					s += ",data:(";
					uint32_t end = off + CHUNK;
					if(end > COGGING_DFT_BIN_COUNT) end = COGGING_DFT_BIN_COUNT;
					for(uint32_t i = off; i < end; i++){
						if(i > off) s += ",";
						// Round to int16 to keep payload compact; bin means are
						// torque units in the thousands, so 1-unit resolution is fine.
						s += std::to_string((int16_t)src[i]);
					}
					s += ")";
					CommandHandler::broadcastCommandReply(CommandReply(s, adr),
						(uint32_t)TMC4671_commands::coggingBins, CMDtype::get);
				}
			} else if(adr == 4 || adr == 5){
				// Verification top-20 DFT readout
				const Harmonic* src = (adr == 4) ? this->ver_cw_top : this->ver_ccw_top;
				std::string s = "B";
				s += std::to_string(adr);
				s += ":";
				bool any = false;
				for(int n = 0; n < 20; n++){
					if(src[n].order == 0 && src[n].amplitude <= 0.0f) continue;
					if(any) s += ",";
					any = true;
					s += std::to_string(src[n].order) + ":";
					s += std::to_string((int16_t)src[n].amplitude) + ":";
					s += std::to_string((int16_t)(src[n].phase * 1000.0f));
				}
				if(!any) s += "0:0:0";
				CommandHandler::broadcastCommandReply(CommandReply(s, adr),
					(uint32_t)TMC4671_commands::coggingBins, CMDtype::get);
			}
			return CommandStatus::NO_REPLY;
		}
		break;

	case TMC4671_commands::coggingFFMode:
		// 0 = harmonic sum (default), 1 = combined bins, 2 = per-direction bins.
		if (cmd.type == CMDtype::get) {
			replies.emplace_back((uint32_t)this->cogging_ff_mode);
		} else if (cmd.type == CMDtype::set) {
			uint8_t m = (uint8_t)cmd.val;
			if (m > 2) m = 0;
			// Modes 1 and 2 require bins to be valid (calibration must have run).
			// Do NOT push to replies from a SET branch (board-reboot gotcha) —
			// use a broadcast tagged as calibrateCogging instead so it shows in
			// the calibration log without faulting the command handler.
			if (m != 0 && !this->bins_data_valid) {
				CommandHandler::broadcastCommandReply(
					CommandReply("(\"coggingFFMode: bins not available - run calibration first\",0)"),
					(uint32_t)TMC4671_commands::calibrateCogging, CMDtype::get);
				m = 0;
			}
			this->cogging_ff_mode = m;
		}
		break;
#endif

		default:
			return CommandStatus::NOT_FOUND;
	}
	return status;


}

#if defined(TIM_TMC) || defined(TIM_CALIBRATION)
	void TMC4671::timerElapsed(TIM_HandleTypeDef* htim){
#ifdef TIM_CALIBRATION
		// If calibration timer triggered the interrupt, notify the waiting calibration loop thread.
		if(this->calibTimer != nullptr && htim == this->calibTimer){
			this->NotifyFromISR();
			return;
		}
#endif
#ifdef TIM_TMC
		if(htim == this->externalEncoderTimer){
			// === Notify sampler thread to read position at TIM_TMC rate ===
			// The ISR only signals the thread via NotifyFromISR (FreeRTOS-safe).
			// The thread does the SPI read in thread context where semaphores are legal.
			if (this->initialized && this->encoderAligned && samplerThread != nullptr) {
				samplerThread->triggerFromIsr();
			}

			// Read encoder and send to tmc
			if(usingExternalEncoder() && externalEncoderAllowed() && this->conf.motconf.phiEsource == PhiE::extEncoder && extEncUpdater != nullptr){
				//setPhiE_ext(getPhiEfromExternalEncoder());
				// Signal phiE update
				extEncUpdater->updateFromIsr(); // Use task so that the update is not being done inside an ISR
			}

			// If we are using the external encoder timer to pace calibration
			if (usingExternalEncoder() && this->calibTicksTarget > 0) {
				this->calibTicksCount = this->calibTicksCount + 1;
				if (this->calibTicksCount >= this->calibTicksTarget) {
					this->calibTicksCount = 0;
					this->NotifyFromISR();
				}
			}
		}
#endif
	}
#endif

void TMC4671::setUpExtEncTimer(){
#ifdef TIM_TMC
	if(extEncUpdater == nullptr) // Create updater thread
		extEncUpdater = std::make_unique<TMC_ExternalEncoderUpdateThread>(this);
	// Setup timer
	this->externalEncoderTimer = &TIM_TMC;
	this->externalEncoderTimer->Instance->ARR = TIM_TMC_ARR; // 200 = 5khz = 5 tmc cycles, 250 = 4khz, 240 = 6 tmc cycles
	this->externalEncoderTimer->Instance->PSC = ((TIM_TMC_BCLK)/1000000) +1; // 1µs ticks
	this->externalEncoderTimer->Instance->CR1 = 1;
	HAL_TIM_Base_Start_IT(this->externalEncoderTimer);
#endif
}

/**
 * Configures and starts the calibration hardware timer, or sets up tick counting
 * using the external encoder timer if in use, to avoid desynchronization.
 */
void TMC4671::startCalibTimers(uint32_t period_us) {
#ifdef TIM_TMC
	// If an external encoder is used, we pace the calibration using the existing encoder timer
	// to avoid stopping, reconfiguring, or desynchronizing it.
	if (usingExternalEncoder() && this->externalEncoderTimer != nullptr) {
		uint32_t tmc_period = this->externalEncoderTimer->Instance->ARR;
		if (tmc_period == 0) {
			tmc_period = TIM_TMC_ARR;
		}
		this->calibTicksTarget = period_us / tmc_period;
		if (this->calibTicksTarget == 0) {
			this->calibTicksTarget = 1;
		}
		this->calibTicksCount = 0;
	} else
#endif
	{
		this->calibTicksTarget = 0;
		this->calibTicksCount = 0;
#ifdef TIM_CALIBRATION
		if (this->calibTimer != nullptr) {
			// Stop the timer before changing configurations
			HAL_TIM_Base_Stop_IT(this->calibTimer);
			// Configure the prescaler to count in microseconds (1 MHz frequency)
			this->calibTimer->Instance->PSC = (SystemCoreClock / 1000000) - 1;
			// Auto-reload register defines the period in microseconds
			this->calibTimer->Instance->ARR = period_us;
			// Pre-charge counter to trigger the first interrupt quickly
			this->calibTimer->Instance->CNT = period_us - 20;
			this->calibTimer->Instance->CR1 = 1;
			HAL_TIM_Base_Start_IT(this->calibTimer);
		}
#endif
	}
	// Clear any pending thread notifications before starting wait loops
	this->WaitForNotification(0);
}

/**
 * Stops the calibration hardware timer and resets the calibration tick target.
 */
void TMC4671::stopCalibTimers() {
	this->calibTicksTarget = 0;
	this->calibTicksCount = 0;
#ifdef TIM_CALIBRATION
	if (this->calibTimer != nullptr) {
		HAL_TIM_Base_Stop_IT(this->calibTimer);
	}
#endif
}

/**
 * Returns the actual calibration period in microseconds based on the active timer pacing source.
 */
uint32_t TMC4671::getActualCalibPeriod(uint32_t target_period_us) {
#ifdef TIM_TMC
	// If pacing calibration using TIM_TMC, the actual period is a multiple of TIM_TMC's ARR period.
	if (usingExternalEncoder() && this->externalEncoderTimer != nullptr) {
		uint32_t tmc_period = this->externalEncoderTimer->Instance->ARR;
		if (tmc_period == 0) {
			tmc_period = TIM_TMC_ARR;
		}
		uint32_t ticks = target_period_us / tmc_period;
		if (ticks == 0) {
			ticks = 1;
		}
		return tmc_period * ticks;
	}
#endif
	return target_period_us;
}

/**
 * Medium priority task to update external encoders
 */
TMC4671::TMC_ExternalEncoderUpdateThread::TMC_ExternalEncoderUpdateThread(TMC4671* tmc) : cpp_freertos::Thread("TMCENC",80,33),tmc(tmc){
	this->Start();
}

void TMC4671::TMC_ExternalEncoderUpdateThread::Run(){
	while(true){
		this->WaitForNotification();
		if(tmc->usingExternalEncoder() && !tmc->spiPort.isTaken()){
			tmc->writeRegAsync(0x1C, (tmc->getPhiEfromExternalEncoder())); // Write phiE_ext
		}
	}
}

void TMC4671::TMC_ExternalEncoderUpdateThread::updateFromIsr(){
	if(tmc->initialized)
		this->NotifyFromISR();
}

void TMC4671::errorCallback(const Error &error, bool cleared){
	if(!cleared && error.code == ErrorCode::brakeResistorFailure){
		// shut down and block.
		emergencyStop(false);
		this->changeState(TMC_ControlState::HardError, true);
	}
}

bool TMC4671::isCalibrationInProgress() {
	return (state == TMC_ControlState::FullCalibration || 
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
			state == TMC_ControlState::CoggingCalibration || 
#endif
			state == TMC_ControlState::Pidautotune || 
			state == TMC_ControlState::IndexSearch ||
			state == TMC_ControlState::EncoderInit ||
			state == TMC_ControlState::ExternalEncoderInit);
}

void TMC4671::handleStateWaitPower() {
	allowStateChange = false;
	pulseClipLed(); // blink led

	if (!hasPower() || emergency) {
		this->powerCheckCounter = 0;
		// Special case: Allow CoggingCalibration simulation even without power
		if (requestedState == TMC_ControlState::CoggingCalibration) {
			allowStateChange = true;
		}
		Delay(250);
		return;
	}

	if (++this->powerCheckCounter > 5) {
		if (!powerInitialized) {
			initializeWithPower();
		}
		allowStateChange = true;

		// If a calibration was pending power, go there now
		if (this->postPowerState != TMC_ControlState::NONE) {
			changeState(this->postPowerState);
			this->postPowerState = TMC_ControlState::NONE;
		} else if (encoderAligned) {
			// Normal flow if encoder is already aligned
			changeState(requestedState);
		}
	}
	Delay(100);
}

void TMC4671::handleStateRunning() {

	// Check status, Temps, Everything alright?
	uint32_t tick = HAL_GetTick();
	if (tick - lastStatTime > 2000) { // Every 2s
		lastStatTime = tick;
		statusCheck();
		// Get enable input. If tmc does not reply the result will read 0 or 0xffffffff (not possible normally)
		uint32_t pins = readReg(0x76);
		bool tmc_en = ((pins >> 15) & 0x01) && pins != 0xffffffff;
		if (!tmc_en && motorEnabledRequested) { // Hardware emergency.
			this->estopTriggered = true;
			this->emergencyStop(false);
			ErrorHandler::addError(ESTOP_ERROR);
		}

		// Temperature sense
		if (conf.hwconf.thermistorSettings.temperatureEnabled) {
			float temp = getTemp();
			if (temp > conf.hwconf.thermistorSettings.temp_limit) {
				changeState(TMC_ControlState::OverTemp);
				pulseErrLed();
			}
		}
	}
}

void TMC4671::handleStateFullCalibration() {
	if (!hasPower()) {
		this->postPowerState = TMC_ControlState::FullCalibration;
		changeState(TMC_ControlState::waitPower);
		return;
	}

	fullCalibrationInProgress = true;
	curFilters.flux.params.enable = false;
	setBiquadFlux(curFilters.flux);
	
	// Calibrate ADC
	enablePin.set();
	setPwm(TMC_PwmMode::PWM_FOC); // enable foc to calibrate adc
	Delay(50);
	if (calibrateAdcOffset(500)) {
		saveAdcParams();
	} else {
		calibFailCb();
		return;
	}

	// Encoder
	calibrateEncoder();
	setEncoderType(conf.motconf.enctype);
	recalibrationRequired = false;
	curFilters.flux.params.enable = true;
	setBiquadFlux(curFilters.flux);
}

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
// Helper: Apply torque with zero flux for pure cogging measurement
void TMC4671::applySafeTorque(float torque_cmd) {
	float totalPower = torque_cmd;

	// Respect inversion settings
	if ((this->conf.encoderReversed && conf.motconf.enctype == EncoderType_TMC::ext) ^ conf.invertForce) {
		totalPower = -totalPower;
	}

	int16_t pwr = (int16_t)clip<float, float>(totalPower, -32768, 32767);
	// Use zero flux (Id = 0) to ensure measured current is 100% dedicated to torque (Iq)
	setFluxTorque(0, pwr);
}

// INTERNAL — only called by TMC_SamplerThread. Caller never uses this.
float TMC4671::computeCoggingFF(float pos_f) {
	if (active_tbl == nullptr) return 0.0f;
	float cog = 0.0f;
	float ar = pos_f * 2.0f * PI;
	for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++) {
		if (active_tbl[h].amplitude > 0.0f) {
			cog += active_tbl[h].amplitude *
			       arm_sin_f32(ar * active_tbl[h].order + active_tbl[h].phase);
		}
	}
	return this->cogging_scale * cog;
}

// Helper: Calculate wrapped error between 0 and 1 turn
float TMC4671::getWrappedError(float target, float actual) {
	target = target - (float)((int32_t)target); 
    actual = actual - (float)((int32_t)actual); 
	float err = target - actual;
	if (err > 0.5f) err -= 1.0f;
	if (err < -0.5f) err += 1.0f;
	return err;
}

float TMC4671::getAbsolutePosition() {
	return getPosAbs_f();
}

float TMC4671::interpolateScale(float rpm) {
	if (!scale_curve_valid || scale_curve_count < 2) return this->cogging_scale;
	const float* rpm_pts = scale_curve_rpm_points;
	if (rpm <= rpm_pts[0]) return scale_curve_values[0];
	for (uint8_t i = 0; i < scale_curve_count - 1; i++) {
		if (rpm >= rpm_pts[i] && rpm <= rpm_pts[i+1]) {
			float t = (rpm - rpm_pts[i]) / (rpm_pts[i+1] - rpm_pts[i]);
			return scale_curve_values[i] + t * (scale_curve_values[i+1] - scale_curve_values[i]);
		}
	}
	// Extrapolate past last calibrated point using slope of last segment
	uint8_t last = scale_curve_count - 1;
	float slope = (scale_curve_values[last] - scale_curve_values[last-1]) /
	              (rpm_pts[last] - rpm_pts[last-1]);
	float extrap = scale_curve_values[last] + slope * (rpm - rpm_pts[last]);
	return clip<float>(extrap, 0.1f, 3.0f);
}

float TMC4671::interpolatePhaseAdvance(float rpm) {
	if (!phase_adv_curve_valid || scale_curve_count < 2) return 0.0f;
	const float* rpm_pts = scale_curve_rpm_points;
	if (rpm <= rpm_pts[0]) return phase_advance_curve_values[0];
	for (uint8_t i = 0; i < scale_curve_count - 1; i++) {
		if (rpm >= rpm_pts[i] && rpm <= rpm_pts[i+1]) {
			float t = (rpm - rpm_pts[i]) / (rpm_pts[i+1] - rpm_pts[i]);
			return phase_advance_curve_values[i] + t * (phase_advance_curve_values[i+1] - phase_advance_curve_values[i]);
		}
	}
	// Extrapolate past last calibrated point using slope of last segment
	uint8_t last = scale_curve_count - 1;
	float slope = (phase_advance_curve_values[last] - phase_advance_curve_values[last-1]) /
	              (rpm_pts[last] - rpm_pts[last-1]);
	return phase_advance_curve_values[last] + slope * (rpm - rpm_pts[last]);
}

float TMC4671::getFilteredPosition() {
	Encoder* activeEnc = usingExternalEncoder() ? drvEncoder.get() : static_cast<Encoder*>(this);
	int32_t cpr = activeEnc->getCpr();
	if (cpr == 0) return 0.0f;
	// Integer modulo BEFORE float conversion guarantees zero precision loss.
	// When sampler is active, getPos() returns the cache (raw counts).
	// When inactive, it falls through to getPosHardware().
	// Either way, modulo wrapping ensures 0..1 turns output.
	int32_t remainder = activeEnc->getPos() % cpr;
	if (remainder < 0) remainder += cpr; // mathematical modulo (always positive)
	return (float)remainder / (float)cpr;
}

float TMC4671::get_velocity() {
	// Returns velocity (turns/s) computed from position deltas, which
	// works for ALL encoder types (internal ABN/AENC/hall, external).
	// The old code relied on Encoder::getVelocity() which only updated
	// when the TIM_TMC sampler was active (enctype == ext), leaving
	// measured_rpm / measured_rpm_signed permanently at 0 for internal
	// encoders.  That broke phase advance, per-direction FF bin selection,
	// and scale-curve interpolation at runtime.
	//
	// We now compute velocity directly from two getFilteredPosition()
	// samples and a micros() timestamp, using the same pattern as every
	// calibration velocity loop in this file.
	float pos = getFilteredPosition();
	uint32_t now_us = micros();

	float v_turns_s = 0.0f;
	if (last_vel_tick != 0) {
		float dt = (float)(now_us - last_vel_tick) / 1e6f;
		if (dt > 0.0001f && dt < 1.0f) {
			float delta = getWrappedError(prev_filtered_pos, pos);
			v_turns_s = delta / dt;
		}
	}
	prev_filtered_pos = pos;
	last_vel_tick = now_us;

	measured_rpm = fabsf(v_turns_s) * 60.0f;
	measured_rpm_signed = v_turns_s * 60.0f;
	return v_turns_s;
}

#ifdef COGGING_TABLE_FLASH_START_ADDRESS
/**
 * Rebuilds cogging_harmonics from stored CW/CCW data with position offsets.
 * Called when configurator changes CW/CCW position offsets.
 * Position offset shifts the waveform horizontally: phase -= order * offset_turns * 2π.
 * Magnitude scaling is NOT applied (vertical offset is configurator-only visualization).
 * CW and CCW store arrays are independently sorted; we match by harmonic order.
 */
void TMC4671::recomputeCoggingFromCwCcw() {
	if (!cwccw_data_valid) return;

	float cw_pos_rad = 0.0f;
	float ccw_pos_rad = 0.0f;

	struct Sel { float mag; uint16_t order; float phase; };
	Sel best[COGGING_HARMONICS_COUNT];
	memset(best, 0, sizeof(best));

	bool seen[COGGING_CALIB_DFT_HARMONICS] = {false};

	for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
		uint16_t order;
		float cw_mag = 0.0f, ccw_mag = 0.0f;
		float cw_ph = 0.0f, ccw_ph = 0.0f;

		if (cw_store[i].amplitude > 0.0f) {
			order = cw_store[i].order;
			if (seen[order]) continue;
			seen[order] = true;
			cw_mag = cw_store[i].amplitude;
			// Position offset: phase adjustment is k * Δθ (shift right by Δθ = subtract k*Δθ from phase)
			cw_ph = cw_store[i].phase - (float)order * cw_pos_rad;

			for (uint8_t j = 0; j < COGGING_HARMONICS_COUNT; j++) {
				if (ccw_store[j].order == order && ccw_store[j].amplitude > 0.0f) {
					ccw_mag = ccw_store[j].amplitude;
					ccw_ph = ccw_store[j].phase - (float)order * ccw_pos_rad;
					break;
				}
			}
		} else {
			continue;
		}

		float avg_mag = (cw_mag + ccw_mag) / 2.0f;

		float phase_diff = cw_ph - ccw_ph;
		if (phase_diff > PI) ccw_ph += 2.0f * PI;
		if (phase_diff < -PI) ccw_ph -= 2.0f * PI;
		float avg_phase = (cw_ph + ccw_ph) / 2.0f;

		for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
			if (avg_mag > best[n].mag) {
				for (int s = COGGING_HARMONICS_COUNT - 1; s > n; s--) best[s] = best[s-1];
				best[n].mag = avg_mag;
				best[n].order = order;
				best[n].phase = avg_phase;
				break;
			}
		}
	}

	// Process CCW-only orders
	for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
		if (ccw_store[i].amplitude <= 0.0f) continue;
		uint16_t order = ccw_store[i].order;
		if (seen[order]) continue;
		seen[order] = true;

		float avg_mag = ccw_store[i].amplitude;
		float avg_phase = ccw_store[i].phase - (float)order * ccw_pos_rad;

		for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
			if (avg_mag > best[n].mag) {
				for (int s = COGGING_HARMONICS_COUNT - 1; s > n; s--) best[s] = best[s-1];
				best[n].mag = avg_mag;
				best[n].order = order;
				best[n].phase = avg_phase;
				break;
			}
		}
	}

	memset(cogging_harmonics, 0, sizeof(cogging_harmonics));
	for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
		if (best[n].mag <= 0.0f) continue;
		cogging_harmonics[n].order = best[n].order;
		cogging_harmonics[n].amplitude = best[n].mag;
		cogging_harmonics[n].phase = best[n].phase;
	}

	// NOTE: saveCoggingTable() is NOT called here.
	// Flash writes during motor operation can cause SPI faults / crashes.
	// The configurator sends coggingSave separately after all offset commands.
}
#endif

/**
 * Calculates a detent torque compensation map (anti-cogging) for the motor.
 * This function is now ATOMIC: it blocks the TMC thread to perform high-frequency sampling.
 */

/**
 * Blends per-RPM harmonic tables based on current measured RPM.
 * Below blend_rpm1: 100% cogging_harmonics
 * blend_rpm1..blend_rpm2: linear blend between cogging_harmonics and cogging_harmonics_rpm2
 * Above blend_rpm2: linear blend between cogging_harmonics_rpm2 and cogging_harmonics_rpm3
 * Unavailable tables are treated as all-zero.
 */
void TMC4671::blendHarmonicTables(float rpm, Harmonic* out_table) {
	memset(out_table, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));

	float w_lo = 1.0f, w_hi = 0.0f;
	const Harmonic* tab_a = this->cogging_harmonics;
	const Harmonic* tab_b = this->cogging_harmonics_rpm2;

	if (rpm < this->blend_rpm1 || !this->rpm2_table_valid) {
		// Low RPM only (or no mid table available)
		w_lo = 1.0f; w_hi = 0.0f;
		tab_b = this->cogging_harmonics; // unused, set to avoid nullptr issues
	} else if (rpm < this->blend_rpm2 && this->rpm2_table_valid) {
		// Blend between low and mid
		float t = (rpm - this->blend_rpm1) / (this->blend_rpm2 - this->blend_rpm1);
		w_lo = 1.0f - t;
		w_hi = t;
	} else if (rpm < this->blend_rpm3 && this->rpm3_table_valid) {
		// Blend between mid and high
		tab_a = this->cogging_harmonics_rpm2;
		tab_b = this->cogging_harmonics_rpm3;
		float t = (rpm - this->blend_rpm2) / (this->blend_rpm3 - this->blend_rpm2);
		w_lo = 1.0f - t;
		w_hi = t;
	} else {
		// Above all thresholds: use highest available
		if (this->rpm3_table_valid) {
			memcpy(out_table, this->cogging_harmonics_rpm3, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
		} else if (this->rpm2_table_valid) {
			memcpy(out_table, this->cogging_harmonics_rpm2, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
		} else {
			memcpy(out_table, this->cogging_harmonics, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
		}
		return;
	}

	// Linear blend: for each harmonic order, interpolate amplitude and phase.
	// Phase needs unwrapping.
	// Build lookup by order for tab_a and tab_b.
	for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
		uint16_t order = tab_a[i].order;
		if (order == 0 && tab_a[i].amplitude <= 0.0f) continue;

		float amp_a = tab_a[i].amplitude;
		float ph_a = tab_a[i].phase;

		// Find matching order in tab_b
		float amp_b = 0.0f, ph_b = 0.0f;
		for (uint8_t j = 0; j < COGGING_HARMONICS_COUNT; j++) {
			if (tab_b[j].order == order && tab_b[j].amplitude > 0.0f) {
				amp_b = tab_b[j].amplitude;
				ph_b = tab_b[j].phase;
				break;
			}
		}

		// Phase unwrap
		float diff = ph_b - ph_a;
		if (diff > PI) ph_b -= 2.0f * PI;
		if (diff < -PI) ph_b += 2.0f * PI;

		out_table[i].order = order;
		out_table[i].amplitude = amp_a * w_lo + amp_b * w_hi;
		out_table[i].phase = ph_a * w_lo + ph_b * w_hi;
	}
}

void TMC4671::handleStateCoggingCalibration() {

	auto broadcastCalibLog = [this](int val, const char* format, ...) {
		char buf[128];
		buf[0] = '(';
		buf[1] = '\"';
		va_list args;
		va_start(args, format);
		const int max_fmt = (int)sizeof(buf) - 17; // reserve space for '("', '",val)', null
		int len = vsnprintf(buf + 2, max_fmt + 1, format, args);
		va_end(args);
		if (len < 0) return;
		if (len > max_fmt) len = max_fmt; // clamp to prevent buffer overflow
		snprintf(buf + 2 + len, sizeof(buf) - len - 2, "\",%d)", val);
		CommandHandler::broadcastCommandReply(CommandReply(std::string(buf)), (uint32_t)TMC4671_commands::calibrateCogging, CMDtype::get);
	};
	const char* errorMessage = nullptr;
	bool accelPosWasReset = false;
	float* iq_acc_cos = nullptr;
	float* iq_acc_sin = nullptr;
	// Spatial-bin arrays for noise-robust DFT (see allocation block for rationale).
	float* iq_bin_sum = nullptr;
	uint16_t* iq_bin_count = nullptr;
	// COGGING_DFT_BIN_COUNT now defined in TMC4671.h (720) — shared with member arrays.
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
	float* id_acc_cos = nullptr;
	float* id_acc_sin = nullptr;
#endif

	prevCalibMode = getMotionMode();
	TMC4671PIDConf prevPids = curPids;
	TMC4671PIDConf calibPids = curPids;
	
	// REVOLUTION_TIME_MS is now computed dynamically in the RPM profile loop
	// below, based on the active calib_rpm, so each sweep is exactly 1 revolution.
	// const uint32_t REVOLUTION_TIME_MS removed — see dynamic calc below.
	const uint32_t BREAKOUT_STEP_MS = 50;
	const uint32_t SYSID_J_PULSE_MS = 150;
	const uint32_t SYSID_B_DURATION_MS = 2000;
	const uint32_t VAL_TOTAL_DURATION_MS = 2500; 
	const uint32_t COGGING_WARMUP_MS = 1500;
	uint32_t cogging_warmup_ms = COGGING_WARMUP_MS; // may be scaled by inertia later
	const uint32_t GAIN_SWEEP_WARMUP_MS = 750;

	arm_pid_instance_f32 pid_soft;
	float max_test_torque = bangInitPower > 0 ? (float)bangInitPower * 0.8f : 2000.0f; 
	volatile TMC4671CoggingDebugVars& dbg = g_tmc4671_cogging_debug;


	auto resetDebugWatch = [&]() {
		dbg.pidExecCount = 0;
		dbg.pidExecRate = 0;
		dbg.positionErrorDeg = 0.0f;
		dbg.iqCompensation = 0.0f;
		dbg.iqCmd = 0.0f;
		dbg.Appliediq = 0.0f;
		dbg.actualIq = 0;
		dbg.lastCaptureDebugUs = 0;
	};

	auto captureDebug = [&](TMC4671CoggingDebugPhase phase,
			float target_rpm,
			float target_pos_turns,
			float actual_pos_turns,
			float error_turns,
			float iq_pid,
			//float iq_friction,
			float iq_inertia,
			float iq_compensation,
			float iq_cmd,
			float iq_applied,
			float current_vel_turns,
			float current_accel_rad,
			float identified_j,
			float identified_b,
			float dynamic_friction) {
		// Real Hz from micros() delta — unaffected by decimation or loop jitter
		uint32_t now_us = micros();
		if (now_us != 0) {
			uint32_t delta_us = now_us - dbg.lastCaptureDebugUs;
			if (delta_us > 0) {
				dbg.pidExecRate = delta_us;
			}
		}
		dbg.lastCaptureDebugUs = now_us;
		
		dbg.positionErrorDeg = error_turns * 36000.0f; // ×10 scaled so sub-degree error is visible alongside mA-scale iqCmd
		dbg.angle = actual_pos_turns * 360.0f; // actual angle in degrees (0-360)
		dbg.iqCompensation = iq_compensation;
		dbg.iqCmd = iq_cmd;
		dbg.Appliediq = iq_applied;
		//dbg.iqPid = iq_pid;
		//dbg.iqFriction = iq_friction;
		//dbg.iqInertia = iq_inertia;
		//dbg.currentAccelRad = current_accel_rad;
		//dbg.dynamicFriction = dynamic_friction;
	};

	resetDebugWatch();

	// === WIPE ALL PREVIOUS CALIBRATION DATA ===
	// This must be the first action — before any log, CPR check, or calibration
	// logic — so the user sees a clean slate as soon as Calibrate is clicked.
	memset(this->cogging_harmonics, 0, sizeof(this->cogging_harmonics));
	memset(this->cogging_harmonics_rpm2, 0, sizeof(this->cogging_harmonics_rpm2));
	memset(this->cogging_harmonics_rpm3, 0, sizeof(this->cogging_harmonics_rpm3));
	memset(this->cw_bins, 0, sizeof(this->cw_bins));
	memset(this->ccw_bins, 0, sizeof(this->ccw_bins));
	memset(this->ver_cw_bins, 0, sizeof(this->ver_cw_bins));
	memset(this->ver_ccw_bins, 0, sizeof(this->ver_ccw_bins));
	memset(this->ver_cw_top, 0, sizeof(this->ver_cw_top));
	memset(this->ver_ccw_top, 0, sizeof(this->ver_ccw_top));
	memset(this->cw_store, 0, sizeof(this->cw_store));
	memset(this->ccw_store, 0, sizeof(this->ccw_store));
	memset(this->cogging_bins_combined, 0, sizeof(this->cogging_bins_combined));
	memset(this->scale_curve_values, 0, sizeof(this->scale_curve_values));
	memset(this->phase_advance_curve_values, 0, sizeof(this->phase_advance_curve_values));
	this->cogging_enabled = false;
	this->cogging_scale = 0.0f;
	this->cwccw_data_valid = false;
	this->bins_data_valid = false;
	this->rpm2_table_valid = false;
	this->rpm3_table_valid = false;
	this->scale_curve_valid = false;
	this->phase_adv_curve_valid = false;
	

	broadcastCalibLog(0, "Starting Cogging Calibration: Continuous DFT...");
	
	if (this->getCpr() == 0) { 
		errorMessage = "Abort: CPR is 0"; 
		goto cleanup; 
	}
	
	allowStateChange = false;
	// Baseline plateau: full compensation at low RPM, zero phase advance.
	scale_curve_values[0] = 1.0f;
	phase_advance_curve_values[0] = 0.0f;
	
	// Set temporary robust PIDs for velocity control during calibration
	calibPids.velocityP = 1000;
	calibPids.velocityI = 100;
	setPids(calibPids);

	// Disable flux filter during calibration (match FullCalibration logic)
	curFilters.flux.params.enable = false;
	setBiquadFlux(curFilters.flux);
	
	// 1. ALLOCATE ACCUMULATORS
	// The live time-domain DFT (cos/sin accumulation in the sweep loop) was
	// biasing the stored FF high: with Kp in the millions, iq_cmd = Kp*pos_err
	// oscillates ±hundreds of units around the true cogging mean, and that
	// noise correlates with angle because cogging velocity ripple clusters
	// samples at troughs. The DFT then traced the upper envelope of iq_cmd
	// instead of its mean.
	// Fix: bin samples by angle (0.5° resolution), average within each bin,
	// then run a clean rectangular DFT on the per-bin mean. The per-bin
	// average drives the position-correlated PID noise toward its local
	// mean before the DFT sees it.
	iq_acc_cos = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	iq_acc_sin = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	iq_bin_sum = (float*)pvPortMalloc(COGGING_DFT_BIN_COUNT * sizeof(float));
	iq_bin_count = (uint16_t*)pvPortMalloc(COGGING_DFT_BIN_COUNT * sizeof(uint16_t));
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
	id_acc_cos = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	id_acc_sin = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#endif
	
	if(!iq_acc_cos || !iq_acc_sin || !iq_bin_sum || !iq_bin_count
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
		|| !id_acc_cos || !id_acc_sin
#endif
	) {
		errorMessage = "Abort: Memory fail (Heap)";
		goto cleanup;
	}

	memset(iq_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	memset(iq_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	memset(iq_bin_sum, 0, COGGING_DFT_BIN_COUNT * sizeof(float));
	memset(iq_bin_count, 0, COGGING_DFT_BIN_COUNT * sizeof(uint16_t));
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
	memset(id_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	memset(id_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#endif

	// 2. CALIBRATION PROCESS AND ANALYSIS
	{
		uint32_t total_samples = 0;
		bool any_profile_succeeded = false;
		float starting_torque = bangInitPower > 0 ? (float)bangInitPower * 0.15f : 300.0f; 
		float tuning_torque = starting_torque;
		float dynamic_friction = 0.0f;

	// Unified friction feedforward: sign(velocity) × dynamic_friction × 0.5
	// Callers pass whatever velocity they have (RPM, turns/s, etc.) — only sign matters.
	auto calcFrictionFF = [&](float vel) -> float {
		if (vel > 0.0f) return dynamic_friction * 0.5f;
		if (vel < 0.0f) return -dynamic_friction * 0.5f;
		return 0.0f;
	};

	uint32_t enc_cpr = this->getCpr();
		uint32_t enc_decimation_ratio = 1;
		float resolution_penalty = 1.0f;
		float calib_rpm = 60.0f / (float)COGGING_CALIB_TIME_PER_REV_S;
		const char* enc_perf = "High-Res";

		float J = 0.0f;
		float B = 0.0f;
		uint32_t next_tick = 0;
		uint32_t act_period = 0;
		float dt_sec = 0.0f;

		struct QuarterErrorStats {
			float quarterMaxErrDeg[4] = {0.0f, 0.0f, 0.0f, 0.0f};
			float worstQuarterDeg = 0.0f;
			uint8_t quarterMask = 0;
		};

		auto resetQuarterErrorStats = [&](QuarterErrorStats& stats) {
			stats.worstQuarterDeg = 0.0f;
			stats.quarterMask = 0;
			for (uint8_t quarter = 0; quarter < 4; quarter++) {
				stats.quarterMaxErrDeg[quarter] = 0.0f;
			}
		};

		auto logQuarterErrorStats = [&](const char* label, float kp, float ki, const QuarterErrorStats& stats) {
			broadcastCalibLog(0,
				"%s -> Kp:%.0f Ki:%.0f WorstQ:%.2f Q(%.2f %.2f %.2f %.2f)",
				label,
				kp,
				ki,
				stats.worstQuarterDeg,
				stats.quarterMaxErrDeg[0],
				stats.quarterMaxErrDeg[1],
				stats.quarterMaxErrDeg[2],
				stats.quarterMaxErrDeg[3]);
		};

		auto runQuarterTurnValidation = [&](TMC4671CoggingDebugPhase phase,
				float kp,
				float ki,
				uint32_t warmup_ms,
				uint32_t timeout_ms,
				QuarterErrorStats& stats) {
			resetQuarterErrorStats(stats);

			uint32_t act_period = getActualCalibPeriod(1000);
			float dt_sec = (float)act_period / 1000000.0f;
			pid_soft.Kp = clip<float,float>(kp, 50.0f, 250000.0f);
			pid_soft.Ki = clip<float,float>(ki, 0.0f, 100000.0f);
			pid_soft.Kd = coggingSpeedD / dt_sec; // rate-compensated (1/Ts)
			arm_pid_init_f32(&pid_soft, 1);

			float target_pos_turns = getAbsolutePosition();
			uint32_t eval_start = HAL_GetTick();
			uint32_t next_tick = micros();
			uint32_t enc_decimation_counter = 0;
			bool all_quarters_seen = false;

			startCalibTimers(1000);
			while (HAL_GetTick() - eval_start < timeout_ms && !emergency && hasPower()) {
				next_tick += act_period;
				target_pos_turns += (calib_rpm / 60.0f) * dt_sec;

				if (enc_decimation_counter % enc_decimation_ratio == 0) {
					float actual_pos_turns = getAbsolutePosition();
					float err = target_pos_turns - actual_pos_turns;
					float iq_pid = arm_pid_f32(&pid_soft, err);
			float iq_ff = this->cogging_calib_frictionFF ? calcFrictionFF(calib_rpm) : 0.0f;
			float iq_cmd = clip<float,float>(iq_pid + iq_ff, -max_test_torque, max_test_torque);

				captureDebug(phase, calib_rpm, target_pos_turns, actual_pos_turns, err, iq_pid, 0.0f, 0.0f, iq_cmd, iq_cmd, 0.0f, 0.0f, J, B, dynamic_friction);
				applySafeTorque(iq_cmd);

					if (HAL_GetTick() - eval_start >= warmup_ms) {
						float wrapped_pos = actual_pos_turns - (float)((int32_t)actual_pos_turns);
						if (wrapped_pos < 0.0f) {
							wrapped_pos += 1.0f;
						}
						uint32_t quarter = (uint32_t)(wrapped_pos * 4.0f);
						if (quarter > 3) {
							quarter = 3;
						}

						float abs_err_deg = fabsf(err) * 360.0f;
						if (abs_err_deg > stats.quarterMaxErrDeg[quarter]) {
							stats.quarterMaxErrDeg[quarter] = abs_err_deg;
						}
						if (abs_err_deg > stats.worstQuarterDeg) {
							stats.worstQuarterDeg = abs_err_deg;
						}
						stats.quarterMask |= (uint8_t)(1u << quarter);
						all_quarters_seen = (stats.quarterMask == 0x0Fu);
					}
				}

				enc_decimation_counter++;
				refreshWatchdog();
#ifdef TIM_CALIBRATION
				if (this->calibTimer != nullptr) {
					this->WaitForNotification();
				} else
#endif
				{
					while ((micros() - next_tick) & 0x80000000) {}
				}

				if (all_quarters_seen) {
					break;
				}
			}
			stopCalibTimers();
			applySafeTorque(0);
			return all_quarters_seen && !emergency && hasPower();
		};

		if (!hasPower()) {
			// --- DEBUG SIMULATION MODE ---
			broadcastCalibLog(0, "Simulating acquisition (No power detected)...");
			total_samples = 2000;
			for (uint32_t i = 0; i < total_samples; i++) {
				float pos_f = (float)i / (float)total_samples;
				float angle_rad = pos_f * 2.0f * PI;
				float iq = 1500.0f * arm_sin_f32(angle_rad * 13.0f) + 800.0f * arm_sin_f32(angle_rad * 36.0f + (PI/2.0f));
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
				float id = 100.0f;
#endif
				float s1, c1;
				arm_sin_cos_f32(pos_f * 360.0f, &s1, &c1);

				float cur_s = s1, cur_c = c1;
				for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
					iq_acc_cos[k] += (iq * cur_c);
					iq_acc_sin[k] += (iq * cur_s);
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
					id_acc_cos[k] += (id * cur_c);
					id_acc_sin[k] += (id * cur_s);
#endif
					float next_c = cur_c * c1 - cur_s * s1;
					float next_s = cur_c * s1 + cur_s * c1;
					cur_c = next_c; cur_s = next_s;
				}
				if(i % 100 == 0) refreshWatchdog();
			}
		} else {
			// --- REAL ACQUISITION SETUP ---
			startMotor();
			setMotionMode(MotionMode::torque, true);

			// --- STEP 1/4: ENCODER IDENTIFICATION ---
			broadcastCalibLog(0, "STEP 1/4: Encoder Identification...");
			
			if (enc_cpr > 0 && enc_cpr < 20000) {
				enc_decimation_ratio = 8;      // PID runs at 500 Hz
				resolution_penalty = 0.2f;     // Heavily damp the proportional gain
				calib_rpm = 12.0f;            // Spin faster to accumulate enough ticks between loops
				enc_perf = "Low-Res";
			} else if (enc_cpr >= 20000 && enc_cpr < 50000) {
				enc_decimation_ratio = 4;      // PID runs at 1 kHz
				resolution_penalty = 0.5f;     // Mildly damp the proportional gain
				calib_rpm = 6.0f;             // Slightly faster calibration
				enc_perf = "Medium-Res";
			}
			
			broadcastCalibLog(0, "Encoder: %s (CPR: %lu). Decimation: %lu, Pen: %.2f", enc_perf, enc_cpr, enc_decimation_ratio, resolution_penalty);

			// --- STEP 2/4: SYSTEM IDENTIFICATION (J & B) ---
			broadcastCalibLog(0, "STEP 2/4: Physical System Identification...");
			dbg.phase = static_cast<uint32_t>(TMC4671CoggingDebugPhase::SysId);

			float start_pos = getAbsolutePosition();
			bool friction_broken = false;

			// Hoisted: IMC Kp baseline for per-profile P-gain auto-tuning.
			// Declared here so it's visible in the RPM profile loop below.
			float imc_kp = 50.0f;

	// Default J to 10 so it's available even when using manual PID
	J = 10.0f;

if (coggingSpeedP == 0.0f && coggingSpeedI == 0.0f) {
		// SysId: break friction, measure J, measure B, then compute IMC gains
		// Step 1.1: Break static friction
		broadcastCalibLog(0, "Step 1.1: Breaking static friction...");
		while (!friction_broken && tuning_torque < max_test_torque && !emergency) {
			applySafeTorque(tuning_torque);
			Delay(BREAKOUT_STEP_MS); 
			if (fabs(getWrappedError(start_pos, getAbsolutePosition())) > 0.005f) {
				friction_broken = true;
				tuning_torque *= 1.1f; 
			} else {
				tuning_torque += 100.0f; 
			}
		}
		applySafeTorque(0);
		Delay(500); 

		// Step 1.2: Measure Inertia (J) using a precise torque pulse
		broadcastCalibLog(0, "Step 1.2: Measuring Mechanical Inertia (J)...");
		float j_torque = tuning_torque;
		start_pos = getAbsolutePosition();
		uint32_t j_start_us = micros();
		uint32_t j_target_us = j_start_us + (SYSID_J_PULSE_MS * 1000);
		
		applySafeTorque(j_torque);
		// Strict timing loop: Busy-wait to guarantee precise pulse duration
		while (((micros() - j_target_us) & 0x80000000) && !emergency) {
			refreshWatchdog();
		}
		uint32_t j_end_us = micros();
		float end_pos = getAbsolutePosition();
		applySafeTorque(0);
		
		float dt_j = (float)(j_end_us - j_start_us) / 1000000.0f;
		float d_pos_turns = fabs(getWrappedError(start_pos, end_pos));
		float d_pos_rad = d_pos_turns * 2.0f * PI; 
		
		if (d_pos_rad < 0.001f) {
			errorMessage = "Abort: Motor did not move during inertia measurement";
			goto cleanup;
		}

		// Physics Formula: J = (Torque * dt^2) / (2 * delta_pos_rad)
		// Scale by 100 to map physical J to the abstract TMC integer units
		J = ((j_torque * dt_j * dt_j) / (2.0f * d_pos_rad)) * 100.0f;
		Delay(500);

		// Scale warmup time with inertia: heavier motors need longer to reach speed.
		// J is in abstract TMC units (physical J * 100).  Typical J: K52G~100, Mige~630.
		// Warmup = max(1500ms, J * calib_rpm / 100 + 1000ms), capped at 8s.
		{
			float warmup_from_J = J / 100.0f * calib_rpm + 1000.0f;
			cogging_warmup_ms = (uint32_t)warmup_from_J;
			if (cogging_warmup_ms < 1500) cogging_warmup_ms = 1500;
			if (cogging_warmup_ms > 8000) cogging_warmup_ms = 8000;
		}

		// Step 1.3: Measure Viscous Friction (B)
		broadcastCalibLog(0, "Step 1.3: Measuring Viscous Friction (B)...");
		float test_rpm = 30.0f;
		float b_target_vel_turns = test_rpm / 60.0f; 
		float b_target_vel_rad = b_target_vel_turns * 2.0f * PI;
		float b_pos = getAbsolutePosition();
		float b_sum_torque = 0.0f;
		uint32_t b_samples = 0;
		uint32_t b_start = HAL_GetTick();
		next_tick = micros();
		
		float kp_vel = j_torque * 5.0f; 
		act_period = getActualCalibPeriod(1000);
		dt_sec = (float)act_period / 1000000.0f;
		startCalibTimers(1000);
		while (HAL_GetTick() - b_start < SYSID_B_DURATION_MS && !emergency) {
			next_tick += act_period;
			b_pos += b_target_vel_turns * dt_sec;
			float err = b_pos - getAbsolutePosition();
			float cmd = err * kp_vel;
			cmd = clip<float,float>(cmd, -max_test_torque, max_test_torque);
			applySafeTorque(cmd);
			
			// WARMUP: Only integrate torque after half the duration to ensure steady-state velocity
			if (HAL_GetTick() - b_start > (SYSID_B_DURATION_MS / 2)) { 
				b_sum_torque += cmd;
				b_samples++;
			}
			refreshWatchdog();
#ifdef TIM_CALIBRATION
			if (this->calibTimer != nullptr) {
				this->WaitForNotification();
			} else
#endif
			{
				while ((micros() - next_tick) & 0x80000000) {}
			}
		}
		stopCalibTimers();
		applySafeTorque(0);
		dynamic_friction = b_sum_torque / (float)b_samples;
		B = (dynamic_friction / b_target_vel_rad) * 100.0f;
		B = clip<float, float>(B, 0.0f, 100000.0f);
		Delay(500);

		// Step 1.4: IMC (Internal Model Control) Pole Placement Calculation
		/* * 1. Continuous Bandwidth (f_bw) Calculation
		 * Instead of hardcoded steps, we linearly degrade the bandwidth based on inertia (J).
		 * - Low inertia (e.g., K52G, J ~ 100): High bandwidth (clamped to 15.0 Hz) for maximum responsiveness.
		 * - High inertia (e.g., Mige, J ~ 630): Lower bandwidth (~13.5 Hz) to prevent Kp saturation and resonance.
		 * - Extreme/Error inertia (J > 2000): Safe mode bandwidth (clamped to 6.0 Hz) for stability.
		 */
			float f_bw = clip<float>(16.5f - (0.0047f * J), 6.0f, 15.0f);
			
			/* * 2. Continuous Integral Scale (ki_scale) Calculation
			 * NOTE: ki_scale is divided by freq_khz because the main loops run at freq_khz instead of 1 kHz.
			 */
			float freq_khz = 1000.0f / (float)TIM_TMC_ARR;
			float ki_scale = clip<float>(0.3f / J, 0.0002f, 0.001f) / freq_khz;
			
			float wn = 2.0f * PI * f_bw;
			
			pid_soft.Kp = 2.0f * 1.0f * wn * J - B;
			pid_soft.Ki = wn * wn * J * ki_scale; 
			pid_soft.Kd = 0.0f;
			
			// Higher Kp limit allowed at 5kHz due to lower latency, scale by resolution
			pid_soft.Kp = pid_soft.Kp * resolution_penalty;
			pid_soft.Kp = clip<float,float>(pid_soft.Kp, 50.0f, 250000.0f);
			pid_soft.Ki = clip<float,float>(pid_soft.Ki, 1.0f, 100000.0f);
			arm_pid_init_f32(&pid_soft, 1);

			imc_kp = pid_soft.Kp;
			float imc_ki = pid_soft.Ki;
			// Use the initial calib_rpm (3 RPM = 20000 ms) for the IMC validation sweep.
			uint32_t gain_sweep_timeout_ms = GAIN_SWEEP_WARMUP_MS + ((20000U * 2U) / 3U) + 500U;
			if (gain_sweep_timeout_ms < VAL_TOTAL_DURATION_MS) {
				gain_sweep_timeout_ms = VAL_TOTAL_DURATION_MS;
			}
			const uint8_t P_SWEEP_MAX_STEPS = 6;     // (sweep disabled — preserved for future use)
			const uint8_t I_SWEEP_MAX_STEPS = 6;     // (sweep disabled)
			const uint8_t MAX_NON_IMPROVING_STEPS = 2; // (sweep disabled)
			const float IMPROVEMENT_EPSILON_DEG = 0.05f; // (sweep disabled)
			(void)P_SWEEP_MAX_STEPS; (void)I_SWEEP_MAX_STEPS;
			(void)MAX_NON_IMPROVING_STEPS; (void)IMPROVEMENT_EPSILON_DEG;

			broadcastCalibLog(0, "IMC baseline -> J:%.0f B:%.0f | Kp:%.0f Ki:%.0f", J, B, imc_kp, imc_ki);

			QuarterErrorStats imc_stats;
			broadcastCalibLog(0, "IMC validation rotation...");
			if (!runQuarterTurnValidation(TMC4671CoggingDebugPhase::Validation, imc_kp, imc_ki, GAIN_SWEEP_WARMUP_MS, gain_sweep_timeout_ms, imc_stats)) {
				if (emergency || !hasPower()) {
					errorMessage = emergency ? "Abort: IMC validation interrupted" : "Abort: Power lost during IMC validation";
					goto cleanup;
				}
				// Didn't cover all quarters — motor may be hitting travel limits.
				// Not fatal; IMC baseline is already computed. Continue with warning.
				broadcastCalibLog(0, "IMC validation incomplete (quarters missed) — continuing.");
			}
			logQuarterErrorStats("IMC", imc_kp, imc_ki, imc_stats);

			// PI sweep disabled — using IMC gains directly to avoid over-driving the motor
			broadcastCalibLog(0, "Using IMC gains directly (sweep disabled)");
			float best_sweep_kp = imc_kp;
			float best_sweep_ki = 0.0f;  // I=0 enforced — auto-tuning is P-only

			// Rate-compensated Kd: CMSIS PID D-term scales with Ts², so we divide
			// by sample time to get a continuous-time-equivalent derivative gain.
			// Without this, D needs absurdly large values (e.g. 9M at 7kHz) to have
			// any effect on cogging-speed position oscillations.
			float dt_soft = (float)TIM_TMC_ARR / 1000000.0f;
			pid_soft.Kp = best_sweep_kp;
			pid_soft.Ki = best_sweep_ki;
			pid_soft.Kd = coggingSpeedD / dt_soft;
			// Store IMC Kp into profile 0 so the auto-tuning can use it as baseline.
			// I is forced to zero — the auto-tuning tune is P-only.
			this->cogging_calib_pidP[0] = (uint32_t)best_sweep_kp;
			this->cogging_calib_pidI[0] = 0;
			this->cogging_calib_pidD[0] = 0;
			} else {
				float dt_soft = (float)TIM_TMC_ARR / 1000000.0f;
				pid_soft.Kp = coggingSpeedP;
				pid_soft.Ki = coggingSpeedI;
				pid_soft.Kd = coggingSpeedD / dt_soft;
				imc_kp = coggingSpeedP;  // manual P as baseline for auto-tuning
#ifndef COGGING_PHASE_SHIFT_MULTIRPM
				// Store manual values into profile 0 so the RPM profile loop
				// below reloads them instead of zeroing them out.
				this->cogging_calib_pidP[0] = (uint32_t)coggingSpeedP;
				this->cogging_calib_pidI[0] = (uint32_t)coggingSpeedI;
				this->cogging_calib_pidD[0] = (uint32_t)coggingSpeedD;
#endif
				broadcastCalibLog(0, "Manual PID Override -> Kp:%.0f Ki:%.0f Kd:%.0f", pid_soft.Kp, pid_soft.Ki, pid_soft.Kd);
			}
			arm_pid_init_f32(&pid_soft, 1);
			Delay(250);

// --- STEP 3/4: DFT ACQUISITION AT CONSTANT SPEED ---
			// Multi-RPM profile loop: calibrate at each configured RPM speed.
			// Each profile uses its own iteration count from cogging_calib_iters[].
			// Each profile writes DIRECTLY into its own target table (active_tbl),
			// so the routine is identical across profiles — only the target location
			// increments (location 1 = cogging_harmonics, 2 = _rpm2, 3 = _rpm3).

			// Clear stale higher-table validity flags + tables from a previous
			// calibration. If this run uses fewer profiles (or a higher profile
			// aborts) we must not leave a stale-true flag pointing at empty/garbage
			// data, or blendHarmonicTables() would return an empty table at speed.
			this->rpm2_table_valid = false;
			this->rpm3_table_valid = false;
			memset(this->cogging_harmonics_rpm2, 0, sizeof(this->cogging_harmonics_rpm2));
			memset(this->cogging_harmonics_rpm3, 0, sizeof(this->cogging_harmonics_rpm3));

			// Track whether ANY profile produced a usable table. The finalize block
			// uses this instead of total_samples (which only reflects the LAST
			// iteration of the LAST profile, and can be 0 if that profile aborted
			// even though earlier profiles — e.g. rpm1 — succeeded).
	#ifdef COGGING_PHASE_SHIFT_MULTIRPM
			// MULTIRPM: test all scale_curve_rpm_points except RPM 0 (plateau)
			uint8_t multirpm_count = SCALE_CURVE_POINTS - 1;
			for (uint8_t rpm_profile = 0;
				 rpm_profile < multirpm_count && !emergency && hasPower();
				 rpm_profile++) {
				calib_rpm = scale_curve_rpm_points[rpm_profile + 1]; // skip RPM 0
				if (calib_rpm <= 0.0f) calib_rpm = 5.0f;
				bool is_return_tune = false;
				const uint8_t MAX_DFT_ITERATIONS = 1;
#else
			for (uint8_t rpm_profile = 0;
				 rpm_profile < this->cogging_calib_count && !emergency && hasPower();
				 rpm_profile++) {

				calib_rpm = this->cogging_calib_rpm[rpm_profile];
				if (calib_rpm <= 0.0f) calib_rpm = 60.0f / (float)COGGING_CALIB_TIME_PER_REV_S;
				const uint8_t MAX_DFT_ITERATIONS = this->cogging_calib_iters[rpm_profile];
#endif
				if (MAX_DFT_ITERATIONS < 1) continue;

				// Recompute acquisition duration so each CW and CCW sweep
				// is exactly 1 revolution, regardless of the RPM target.
				const uint32_t rev_ms = (uint32_t)((60.0f / calib_rpm) * 1500.0f);
				const uint32_t REVOLUTION_TIME_MS = rev_ms + cogging_warmup_ms;

				broadcastCalibLog(0, "RPM profile %u/%u: target %.1f RPM, %u iterations (%.1f s/rev)",
					rpm_profile + 1, this->cogging_calib_count, calib_rpm, MAX_DFT_ITERATIONS,
					(float)REVOLUTION_TIME_MS / 1000.0f);

			// Load per-profile PID.  Under MULTIRPM, wrap to the last
			// configured profile slot for any profile beyond the array.
			{
				uint8_t pid_src = rpm_profile;
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
				if (pid_src >= COGGING_MAX_CALIB_PROFILES)
					pid_src = COGGING_MAX_CALIB_PROFILES - 1;
#endif
				uint32_t pidP = this->cogging_calib_pidP[pid_src];
				uint32_t pidI = this->cogging_calib_pidI[pid_src];
				uint32_t pidD = this->cogging_calib_pidD[pid_src];
				if (pid_src > 0 && pidP == 0 && pidI == 0 && pidD == 0) {
					pidP = this->cogging_calib_pidP[0];
					pidI = this->cogging_calib_pidI[0];
					pidD = this->cogging_calib_pidD[0];
				}
				this->coggingSpeedP = (float)pidP;
				this->coggingSpeedI = (float)pidI;
				this->coggingSpeedD = (float)pidD;
			}

			// Re-initialize pid_soft with this profile's gains.
			// The STEP 2 IMC calc only ran once; each RPM profile
			// must reset the CMSIS PID state for DFT velocity control.
			pid_soft.Kp = this->coggingSpeedP;
			pid_soft.Ki = this->coggingSpeedI;
			{
				float dt_dft = (float)getActualCalibPeriod(TIM_TMC_ARR) / 1000000.0f;
				pid_soft.Kd = this->coggingSpeedD / dt_dft;
			}
			arm_pid_init_f32(&pid_soft, 1);

// --- P-GAIN AUTO-TUNING SEQUENCE ---
			// Run per RPM profile when cogging_calib_autoPid is enabled.
			// Uses trapezoidal velocity sweeps (accel → cruise → decel) to find
			// the optimal proportional gain that minimizes position tracking error
			// without inducing oscillation. I and D are held at zero during tuning.
			if (this->cogging_calib_autoPid) {
				broadcastCalibLog(0, "Auto-tuning Kp for %.1f RPM...", calib_rpm);
				dbg.phase = static_cast<uint32_t>(TMC4671CoggingDebugPhase::TuneSweepP);

				// Start at IMC-derived baseline for profile 0, or below profile 0's stored P.
				// IMC runs once in STEP 2 and its Kp is the seed for the slowest RPM.
				float test_kp = (rpm_profile == 0) ? (imc_kp * 0.75f) :
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
					// MULTIRPM: start from previous profile's tuned Kp, scaled by RPM ratio
					(coggingSpeedP > 0 ? coggingSpeedP : this->cogging_calib_pidP[0]) * 0.75f;
#else
					this->cogging_calib_pidP[0] * 0.75f;
#endif
				if (test_kp < 50.0f) test_kp = 50.0f;

				float best_kp = 0.0f;  // invalid until a clamp-free test completes
				float lowest_p2p = 999.0f;
				int8_t test_dir = 1; // Alternates 1 and -1 to go back and forth
				bool tuning_done = false;
				bool sweep_up = true;       // current sweep direction: true=UP (×1.25), false=DOWN (÷1.25)
				bool did_down_sweep = false; // true after first error-growth reversal
				bool clamp_just_cleared = false; // true after clamp backoff succeeded — use small step
				uint8_t step_count = 0;
				static constexpr uint8_t MAX_TUNE_STEPS = 20;
			const char* stop_reason = "limit";

				// Fixed I=0, D=0 during P-only tuning
				pid_soft.Ki = 0.0f;
				pid_soft.Kd = 0.0f;

				uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR);
				float dt_sec = (float)period_us / 1000000.0f;

			// --- KINEMATIC TRAJECTORY SETUP ---
			// Time-based ramp (like DFT's warmup ramp): accelerate smoothly over
			// cogging_warmup_ms to avoid torque spikes at direction reversals.
			// cogging_warmup_ms already scales with J (heavier motors get longer ramps).
			float target_vel_turns = calib_rpm / 60.0f;
			float ramp_rate = target_vel_turns / (float)cogging_warmup_ms * 1000.0f; // turns/s²
			// Cruise for at least 600ms (enough for P2P measurement at any RPM).
			// Decel uses half the ramp time (motor coasts naturally, PID helps brake).
			const uint32_t CRUISE_TIME_MS = 600;
			uint32_t sweep_total_ms = cogging_warmup_ms + CRUISE_TIME_MS + (cogging_warmup_ms / 2);

			while (step_count < MAX_TUNE_STEPS && !tuning_done && !emergency && hasPower()) {
				step_count++;
				pid_soft.Kp = test_kp;
				arm_pid_init_f32(&pid_soft, 1);

			// Coast settle before each P-tuner sweep (scale with RPM like DFT's direction-reversal settle)
			applySafeTorque(0);
			{
				float rev_s = 60.0f / calib_rpm;
				uint32_t settle_ms = (uint32_t)(rev_s * 2000.0f); // ~2 rev worth (matches DFT)
				if (settle_ms < 500)  settle_ms = 500;
				if (settle_ms > 3000) settle_ms = 3000;
				uint32_t settleStart = HAL_GetTick();
				while (HAL_GetTick() - settleStart < settle_ms && !emergency && hasPower()) {
					refreshWatchdog();
					Delay(10);
				}
				arm_pid_init_f32(&pid_soft, 1);
			}
			// Re-capture starting position after coast settle so the sweep
			// always begins from the rotor's actual resting position (may have
			// rolled into a cog detent during settling).
			float target_pos_f = getFilteredPosition();

				float max_err_deg = -999.0f;
				float min_err_deg = 999.0f;
				bool clamp_hit = false;

				float current_vel_turns = 0.0f;

				uint32_t next_tick = micros();
				uint32_t sweepStart = HAL_GetTick();
				startCalibTimers(TIM_TMC_ARR);

				while (HAL_GetTick() - sweepStart < sweep_total_ms && !emergency && hasPower()) {
					next_tick += period_us;

					// Time-based trapezoidal velocity profile (like DFT ramp at line 5207-5218)
					uint32_t elapsed = HAL_GetTick() - sweepStart;
					if (elapsed < cogging_warmup_ms) {
						// Ramp up: linear from 0 to target velocity
						current_vel_turns = ramp_rate * (float)elapsed * 0.001f * test_dir;
						if (fabsf(current_vel_turns) < target_vel_turns * 0.005f)
							current_vel_turns = target_vel_turns * 0.005f * test_dir;
					} else if (elapsed < cogging_warmup_ms + CRUISE_TIME_MS) {
						// Cruise at target velocity
						current_vel_turns = target_vel_turns * test_dir;
					} else {
						// Ramp down to zero over remaining time
						uint32_t decel_elapsed = elapsed - (cogging_warmup_ms + CRUISE_TIME_MS);
						uint32_t decel_total = sweep_total_ms - (cogging_warmup_ms + CRUISE_TIME_MS);
						float frac = (float)decel_elapsed / (float)decel_total;
						if (frac > 1.0f) frac = 1.0f;
						current_vel_turns = target_vel_turns * (1.0f - frac) * test_dir;
					}

					float step = current_vel_turns * dt_sec;
					target_pos_f += step;
					if (target_pos_f >= 1.0f) target_pos_f -= 1.0f;
					if (target_pos_f < 0.0f) target_pos_f += 1.0f;

					float actual_pos_f = getFilteredPosition();
					float err = getWrappedError(target_pos_f, actual_pos_f);
					float err_deg = err * 360.0f;

					float iq_pid = arm_pid_f32(&pid_soft, err);
					// Add slight friction feedforward to help tracking without altering tuning dynamics
				float iq_ff = this->cogging_calib_frictionFF ? calcFrictionFF(calib_rpm * (float)test_dir) : 0.0f;
			float iq_cmd = clip<float,float>(iq_pid + iq_ff, -max_test_torque, max_test_torque);
			applySafeTorque(iq_cmd);
				captureDebug(TMC4671CoggingDebugPhase::TuneSweepP, calib_rpm * (float)test_dir, target_pos_f, actual_pos_f, err, iq_pid, 0.0f, 0.0f, iq_cmd, iq_cmd, current_vel_turns, 0.0f, J, B, dynamic_friction);
					// ONLY measure error during the Cruise phase (ignores accel/decel transients)
					if (elapsed >= cogging_warmup_ms && elapsed < cogging_warmup_ms + CRUISE_TIME_MS) {
						if (err_deg > max_err_deg) max_err_deg = err_deg;
						if (err_deg < min_err_deg) min_err_deg = err_deg;

						// A SINGLE clamp hit signifies fatal oscillation
						if (fabsf(iq_cmd) >= max_test_torque * 0.99f) {
							clamp_hit = true;
							break;
						}
					}

					refreshWatchdog();
#ifdef TIM_CALIBRATION
					if (this->calibTimer != nullptr) this->WaitForNotification();
					else
#endif
					while ((micros() - next_tick) & 0x80000000) {}
				}

				stopCalibTimers();
				applySafeTorque(0);
				// Note: coast settle is done at top of next loop iteration above.

					float p2p_deg = max_err_deg - min_err_deg;
					bool valid_p2p = (p2p_deg > 0.0f && p2p_deg < 720.0f);

					// --- CLAMP HANDLING ---
					if (clamp_hit) {
						// Clamp means oscillation. The motor is ringing — stop it,
						// re-init the PID, and wait for the oscillation to decay
						// before testing the next (lower) Kp.
						applySafeTorque(0);
						arm_pid_init_f32(&pid_soft, 1);
						Delay(500);  // let oscillation decay
						// Reset trajectory origin to current position so the
						// next sweep does a clean accel→cruise→decel from here.
						target_pos_f = getFilteredPosition();

						if (best_kp > 0.0f) {
							// We have a previously-tested safe Kp.
							float backoff_kp = test_kp / 1.25f;
							if (backoff_kp <= best_kp * 1.01f) {
								stop_reason = "clamp";
								broadcastCalibLog(0, "Clamp at Kp:%.0f. Using best safe Kp:%.0f.", test_kp, best_kp);
								tuning_done = true;
								break;
							}
							broadcastCalibLog(0, "Clamp at Kp:%.0f — backing off to %.0f.", test_kp, backoff_kp);
							test_kp = backoff_kp;
							sweep_up = false;
							test_dir = -test_dir;
							continue;
						}
						// No safe Kp yet — keep dividing until we escape or hit floor.
						float backoff_kp = test_kp / 1.25f;
						if (backoff_kp < 50.0f) {
							stop_reason = "clamp";
							broadcastCalibLog(0, "Clamp at Kp:%.0f — hit floor. Cannot find safe Kp.", test_kp);
							tuning_done = true;
							break;
						}
						broadcastCalibLog(0, "Clamp at Kp:%.0f (no safe Kp). Backing off to %.0f.", test_kp, backoff_kp);
						test_kp = backoff_kp;
						sweep_up = false;
						test_dir = -test_dir;
						continue;
					}

					// --- CLAMP BACKOFF RECOVERY ---
					// We were backing off from a clamp. If we're now clamp-free,
					// resume the UP sweep but use a smaller step (×1.10) to avoid
					// stepping right back to the Kp that just clamped.
					if (!sweep_up) {
						sweep_up = true;
						clamp_just_cleared = true;
						broadcastCalibLog(0, "Clamp cleared at Kp:%.0f. Resuming UP sweep.", test_kp);
					}

					// --- P2P EVALUATION ---
					if (valid_p2p) {
						// Track the best (lowest P2P) Kp seen so far
						if (p2p_deg < lowest_p2p) {
							lowest_p2p = p2p_deg;
							best_kp = test_kp;
						}
						// Error growing by >50% from the global minimum: we've passed
						// the optimum in the current sweep direction.
						else if (p2p_deg > lowest_p2p * 1.5f && lowest_p2p < 1.0f) {
							if (!did_down_sweep) {
								// First overshoot — reverse direction and sweep DOWN
								// from the best Kp to bracket the minimum from below.
								broadcastCalibLog(0, "P2P growing (%.2f\xC2\xB0 > %.2f\xC2\xB0). Sweeping DOWN.", p2p_deg, lowest_p2p);
								test_kp = best_kp / 1.25f;
								did_down_sweep = true;
								sweep_up = false;
								test_dir = -test_dir;
								continue;
							} else {
								// Second overshoot (after DOWN sweep) — minimum is bracketed.
								stop_reason = "growth";
								broadcastCalibLog(0, "P2P growing (%.2f\xC2\xB0 > %.2f\xC2\xB0). Minimum bracketed.", p2p_deg, lowest_p2p);
								tuning_done = true;
								break;
							}
						}
						// If we're in the DOWN sweep and error starts increasing by >50%
						// above the best from this DOWN phase, we've passed the minimum.
						// (handled by the else-if above with did_down_sweep==true)
					}

					broadcastCalibLog(0, "Tested Kp:%.0f -> P2P:%.2f\xC2\xB0", test_kp, p2p_deg);

					// Step Kp in the current sweep direction.
					// After clearing a clamp, use a smaller step (×1.10 instead
					// of ×1.25) to avoid stepping right back to the clamping Kp.
					if (sweep_up) {
						test_kp *= clamp_just_cleared ? 1.10f : 1.25f;
						clamp_just_cleared = false;
					} else {
						test_kp /= 1.25f;
						if (test_kp < 50.0f) test_kp = 50.0f;  // floor
					}
					test_dir = -test_dir; // Alternate motion direction for next pass
				}

				// Report result with the reason tuning stopped.
				if (stop_reason[0] == 'c') {  // "clamp"
					if (lowest_p2p >= 999.0f) {
						// No valid measurement — every Kp tested clamped.
						// Fall back to half the initial test Kp as a last resort.
						float fallback_kp = (rpm_profile == 0) ? (imc_kp * 0.75f) : this->cogging_calib_pidP[0] * 0.75f;
						if (fallback_kp < 50.0f) fallback_kp = 50.0f;
						fallback_kp *= 0.5f;  // halve it for safety
						best_kp = fallback_kp;
						broadcastCalibLog(0, "Selected Kp:%.0f (FALLBACK — all Kp clamped, Lowest P2P:N/A)", best_kp);
					} else {
						broadcastCalibLog(0, "Selected Kp:%.0f (Clamp at %.0f, Lowest P2P:%.2f\xC2\xB0)", best_kp, test_kp, lowest_p2p);
					}
				} else if (stop_reason[0] == 'g') {  // "growth"
					broadcastCalibLog(0, "Selected Kp:%.0f (Optimal, Lowest P2P:%.2f\xC2\xB0)", best_kp, lowest_p2p);
				} else if (step_count >= MAX_TUNE_STEPS) {
					broadcastCalibLog(0, "Selected Kp:%.0f (Max steps %u, Lowest P2P:%.2f\xC2\xB0)", best_kp, MAX_TUNE_STEPS, lowest_p2p);
				} else {
					broadcastCalibLog(0, "Selected Kp:%.0f (Max steps, Lowest P2P:%.2f\xC2\xB0)", best_kp, lowest_p2p);
				}
				// Write results back to per-profile storage
				this->coggingSpeedP = best_kp;
				this->coggingSpeedI = 0.0f;
				this->coggingSpeedD = 0.0f;
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
				{
					uint8_t store_src = (rpm_profile < COGGING_MAX_CALIB_PROFILES) ? rpm_profile : (COGGING_MAX_CALIB_PROFILES - 1);
					this->cogging_calib_pidP[store_src] = (uint32_t)best_kp;
					this->cogging_calib_pidI[store_src] = 0;
					this->cogging_calib_pidD[store_src] = 0;
				}
#else
				this->cogging_calib_pidP[rpm_profile] = (uint32_t)best_kp;
				this->cogging_calib_pidI[rpm_profile] = 0;
				this->cogging_calib_pidD[rpm_profile] = 0;
#endif
			}

			// Re-initialize pid_soft with the (possibly auto-tuned) gains for the DFT sweep
			pid_soft.Kp = this->coggingSpeedP;
			pid_soft.Ki = this->coggingSpeedI;
			{
				float dt_dft = (float)getActualCalibPeriod(TIM_TMC_ARR) / 1000000.0f;
				pid_soft.Kd = this->coggingSpeedD / dt_dft;
			}
			arm_pid_init_f32(&pid_soft, 1);

			// Per-profile target table: the calibration routine is IDENTICAL for
			// every RPM profile — only the target location increments. Profile 0
			// writes cogging_harmonics (location 1), profile 1 cogging_harmonics_rpm2
			// (location 2), profile 2 cogging_harmonics_rpm3 (location 3). Each
			// table is cleared, measured fresh, and phasor-refined independently, so
			// every RPM speed gets its own self-contained cogging map.
			Harmonic* active_tbl = this->cogging_harmonics;
			if (rpm_profile == 1) active_tbl = this->cogging_harmonics_rpm2;
			else if (rpm_profile >= 2) active_tbl = this->cogging_harmonics_rpm3;
			this->active_tbl = active_tbl;
			memset(active_tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
			this->cogging_scale = 1.0f;
			// Background FF stays OFF during PID-DFT — the DFT loop applies
			// torque directly via applySafeTorque(). If FF were ON, the
			// sampler thread would fight the DFT loop during WaitForNotification()
			// by applying (0 + FF) between DFT iterations.
			this->calib_ff_active = false;
			this->calib_ff_base_torque = 0.0f;

			Harmonic prev_harmonics[COGGING_HARMONICS_COUNT]; // backup of best table
			float global_max_iq = 0.0f; // track peak PID torque for accel DFT starting point
			float cw_dc_avg = 0.0f;   // DC avg from CW sweep (friction), saved for Step B start

			// DFT retry: if the acquisition clamps (Kp too high for this RPM),
			// lower Kp and restart. Keeps retrying until Kp hits floor (50)
			// or the DFT succeeds clamp-free.
			for (;;) {
				if (pid_soft.Kp < 50.0f || emergency || !hasPower()) {
					broadcastCalibLog(0, "DFT clamp retries hit Kp floor (%.0f).", pid_soft.Kp);
					break;
				}
			bool dft_clamped = false;

			// Save base PID gains for per-iteration halving.
			// As the feedforward table improves with each iteration,
			// the residual cogging shrinks — gentler gains reduce
			// PID-induced noise in the DFT measurement.
			float base_Kp = pid_soft.Kp;
			float base_Ki = pid_soft.Ki;
			float base_Kd = pid_soft.Kd;

			for (uint8_t dft_iter = 0; dft_iter < MAX_DFT_ITERATIONS && !emergency && hasPower(); dft_iter++) {
				// Halve PID gains for each extra iteration (÷2, ÷4, ÷8, ...).
				// Iteration 0 uses full gains; higher iterations use
				// progressively gentler gains tuned to the shrinking residual.
				if (dft_iter > 0) {
					float div = (float)(1 << dft_iter);
					pid_soft.Kp = base_Kp / div;
					pid_soft.Ki = base_Ki / div;
					pid_soft.Kd = base_Kd / div;
					arm_pid_init_f32(&pid_soft, 1);
				}

				total_samples = 0;
				float iter_max_err_deg = 0.0f;

			broadcastCalibLog(0, "DFT iteration %u/%u (Kp:%.0f Ki:%.0f Kd:%.0f)...",
				dft_iter + 1, MAX_DFT_ITERATIONS,
				pid_soft.Kp, pid_soft.Ki, pid_soft.Kd);

				// Separate storage for CW and CCW harmonics — extracted per direction
				// so they can be averaged after both sweeps complete
				struct TempHarmonic { float mag; float phase; };
				TempHarmonic cw_harms[COGGING_CALIB_DFT_HARMONICS];
				TempHarmonic ccw_harms[COGGING_CALIB_DFT_HARMONICS];
				memset(cw_harms, 0, sizeof(cw_harms));
				memset(ccw_harms, 0, sizeof(ccw_harms));

				int8_t dirs[2] = {1, -1};
				
				for (int8_t p : dirs) {
					
					float target_rpm = (p == 1) ? calib_rpm : -calib_rpm;
					float integrated_distance = 0.0f;
					float max_iq_cmd_used = 0.0f;
					float max_err_seen = 0.0f;
					float ema_err = 0.0f;  // lowpass-filtered error for glitch-robust peak tracking

					arm_pid_init_f32(&pid_soft, 1);

					// Clear accumulators per direction so CW and CCW DFTs don't mix
					memset(iq_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(iq_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(iq_bin_sum, 0, COGGING_DFT_BIN_COUNT * sizeof(float));
					memset(iq_bin_count, 0, COGGING_DFT_BIN_COUNT * sizeof(uint16_t));
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
					memset(id_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(id_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#endif
					uint32_t dir_samples = 0;

				// Coast settle: let rotor spin down naturally with zero torque.
				// Time scales with RPM — high speeds need more revolutions to
				// dissipate kinetic energy, low speeds stop almost instantly.
				applySafeTorque(0);
				{
					float rev_s = 60.0f / fabsf(calib_rpm);
					uint32_t settle_ms = (uint32_t)(rev_s * 2000.0f); // ~2 rev worth
					if (settle_ms < 500)  settle_ms = 500;
					if (settle_ms > 3000) settle_ms = 3000;
					uint32_t settleStart = HAL_GetTick();
					while (HAL_GetTick() - settleStart < settle_ms && !emergency && hasPower()) {
						refreshWatchdog();
						Delay(10);
					}
				}
				arm_pid_init_f32(&pid_soft, 1);
					if (!emergency && hasPower()) {
						float target_pos_f = getFilteredPosition();

						calibStartTime = HAL_GetTick();
						uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR); 
						uint32_t next_tick = micros();
						float dt_sec = (float)period_us / 1000000.0f;
						
						float prev_actual_pos_f = getFilteredPosition();
						float prev_vel_turns = target_rpm / 60.0f; // start at steady-state to avoid accel spike
						float full_vel_turns = target_rpm / 60.0f;
						float ramp_vel_turns = 0.0f; // ramped velocity
						// Ramp-up time equals warmup: accelerate smoothly from 0 to target
						float ramp_rate = full_vel_turns / (float)cogging_warmup_ms * 1000.0f; // turns/s²
						
						uint32_t enc_decimation_counter = 0;
						// DFT decimation: keep DFT accumulation ≤ ~4 kHz to prevent
						// 32-bit float overflow
						uint32_t dft_decimation_ratio = 2;
						uint32_t dft_decimation_counter = 0;
						float iq_cmd = 0.0f;
						float iq_inertia = 0.0f;
						float iq_pid = 0.0f;
#ifndef COGGING_DFT_USE_IQ_CMD
						int32_t actual_iq_raw = 0;
#endif

						startCalibTimers(TIM_TMC_ARR);
						while (HAL_GetTick() - calibStartTime < REVOLUTION_TIME_MS && !dft_clamped && !emergency && hasPower()) {
							next_tick += period_us;
							
							// Ramp velocity from 0 to full during warmup to avoid torque spikes
							// at direction reversals and startup. Tiny floor ensures the
							// target moves at least ~1 encoder count/iteration immediately,
							// preventing stick-slip with high-res encoders at low RPM.
							float elapsed = (float)(HAL_GetTick() - calibStartTime);
							if (elapsed < (float)cogging_warmup_ms) {
								ramp_vel_turns = ramp_rate * elapsed * 0.001f; // turns/s
								if (ramp_vel_turns < fabsf(full_vel_turns) * 0.005f)
									ramp_vel_turns = fabsf(full_vel_turns) * 0.005f;
							} else {
								ramp_vel_turns = full_vel_turns;
							}
							float step = ramp_vel_turns * dt_sec;
							target_pos_f += step;
							if (target_pos_f >= 1.0f) target_pos_f -= 1.0f;
							if (target_pos_f < 0.0f) target_pos_f += 1.0f;

							float actual_pos_f = getFilteredPosition();
							
							if (enc_decimation_counter % enc_decimation_ratio == 0) {
								float error = getWrappedError(target_pos_f, actual_pos_f);
								iq_pid = arm_pid_f32(&pid_soft, error);
								
								float delta_pos = getWrappedError(prev_actual_pos_f, actual_pos_f);
								float dt_decimated = dt_sec * enc_decimation_ratio;
								float current_vel_turns = delta_pos / dt_decimated;
								float current_accel_rad = ((current_vel_turns - prev_vel_turns) / dt_decimated) * 2.0f * PI; 
								
								prev_actual_pos_f = actual_pos_f;
								prev_vel_turns = current_vel_turns;

								if (this->cogging_calib_inertiaCorr) {
								iq_inertia = (J / 100.0f) * current_accel_rad;
								}
							float iq_ff = this->cogging_calib_frictionFF ? calcFrictionFF(target_rpm) : 0.0f;
								iq_cmd = iq_pid + iq_ff;

							if (HAL_GetTick() - calibStartTime > cogging_warmup_ms) {
								// EMA-filtered error rejects single-sample glitches
								// (e.g. encoder SPI glitch that would otherwise spike max_err_seen 100x)
								ema_err = ema_err * 0.8f + fabsf(error) * 0.2f;
								if (ema_err > max_err_seen) max_err_seen = ema_err;
								if (fabs(iq_cmd) > max_iq_cmd_used) max_iq_cmd_used = fabs(iq_cmd);
							}
								
								// Cogging feed-forward from THIS profile's table (active_tbl). It is
								// empty on iteration 0 (so the full cogging is measured), then on
								// iterations 1..N it feeds back the running estimate so the DFT only
								// sees the residual, which the phasor-add refines (see commit block).
								float cog_comp = 0.0f;
								{
									float angle_rad = actual_pos_f * 2.0f * PI;
									for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++) {
										if (active_tbl[h].amplitude > 0.0f) {
											cog_comp += active_tbl[h].amplitude * arm_sin_f32(angle_rad * active_tbl[h].order + active_tbl[h].phase);
										}
									}
								}
								float iq_applied = iq_cmd + (this->cogging_scale * cog_comp);
								iq_applied = clip<float,float>(iq_applied, -max_test_torque, max_test_torque);
								// Clamp during DFT means Kp is too high for this RPM —
								// flag for retry with a lower Kp.
								if (fabsf(iq_applied) >= max_test_torque * 0.99f) {
									dft_clamped = true;
								}
								
#ifndef COGGING_DFT_USE_IQ_CMD
								actual_iq_raw = getActualTorque();
								dbg.actualIq = actual_iq_raw;
#endif
								captureDebug(TMC4671CoggingDebugPhase::Acquisition, target_rpm, target_pos_f, actual_pos_f, error, iq_pid, iq_inertia, this->cogging_scale * cog_comp, iq_cmd, iq_applied, current_vel_turns, current_accel_rad, J, B, dynamic_friction);
								applySafeTorque(iq_applied);
							}
							
							enc_decimation_counter++;
							dft_decimation_counter++;
							
							if (integrated_distance < 1.0f && (HAL_GetTick() - calibStartTime > cogging_warmup_ms)) {
								integrated_distance += fabs(step);
								// Only run heavy DFT math every Nth loop to prevent
								// CPU starvation and float accumulator overflow at high rates
								if (dft_decimation_counter % dft_decimation_ratio == 0) {
								// DFT signal source — toggle with #define COGGING_DFT_USE_IQ_CMD
								// actual_iq_raw = total applied torque (includes cogging FF on later passes)
								// iq_cmd        = PID + friction only (residual cogging, for phasor-add)
#ifdef COGGING_DFT_USE_IQ_CMD
								float iq = iq_cmd;
								if (this->cogging_calib_inertiaCorr) {
									// Subtract inertia from DFT signal to isolate
									// pure cogging. Motor still gets inertia via iq_pid.
									iq -= iq_inertia;
								}
#else
								float iq = (float)actual_iq_raw;
								if (this->cogging_calib_inertiaCorr) {
									iq -= iq_inertia;
								}
#endif
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
								float id = (float)getActualFlux();
								// Legacy time-domain DFT kept for ID diagnostic only.
								float s1, c1;
								arm_sin_cos_f32(actual_pos_f * 360.0f, &s1, &c1);
								float cur_s = s1, cur_c = c1;
								for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
									id_acc_cos[k] += (id * cur_c);
									id_acc_sin[k] += (id * cur_s);
									float next_c = cur_c * c1 - cur_s * s1;
									float next_s = cur_c * s1 + cur_s * c1;
									cur_c = next_c; cur_s = next_s;
								}
#endif
								// Spatial binning of iq: accumulate per 0.5° angular bin.
								// The per-bin average (computed in the extraction phase)
								// rejects the Kp*pos_error noise that was biasing the
								// old time-domain iq DFT high (it was tracing the
								// upper envelope of iq_cmd instead of its mean).
								uint32_t bin = (uint32_t)(actual_pos_f * (float)COGGING_DFT_BIN_COUNT);
								if (bin >= COGGING_DFT_BIN_COUNT) bin = COGGING_DFT_BIN_COUNT - 1;
								iq_bin_sum[bin] += iq;
								if (iq_bin_count[bin] != 0xFFFF) iq_bin_count[bin]++;
								dir_samples++;
							}
						}

						refreshWatchdog();
#ifdef TIM_CALIBRATION
							if (this->calibTimer != nullptr) {
								this->WaitForNotification();
							} else
#endif
							{
								while ((micros() - next_tick) & 0x80000000) { }
							}
						}
						stopCalibTimers();
					}
					applySafeTorque(0);

					if (max_err_seen * 360.0f > iter_max_err_deg) {
						iter_max_err_deg = max_err_seen * 360.0f;
					}

					// Extract per-direction harmonics via binned DFT.
					// Compute the mean iq per 0.5° bin, then run a rectangular
					// DFT on that clean iq(theta) function. Phase reference is
					// the bin center (b+0.5)/N, matching the sample-by-sample
					// DFT's angle reference (each sample's actual_pos_f).
					if (dir_samples > 0) {
						float norm = 2.0f / (float)COGGING_DFT_BIN_COUNT;
						for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
							float re = 0.0f, im = 0.0f;
							// Starting angle = bin-0 center angle for harmonic k
							// = (0.5 / N) * 2*pi * k = 180 * k / N degrees
							float s_start, c_start;
							arm_sin_cos_f32((float)k * 180.0f / (float)COGGING_DFT_BIN_COUNT, &s_start, &c_start);
							// Per-bin angle increment for harmonic k = 360 * k / N degrees
							float s1, c1;
							arm_sin_cos_f32((float)k * 360.0f / (float)COGGING_DFT_BIN_COUNT, &s1, &c1);
							float cur_s = s_start, cur_c = c_start;
							for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++) {
								if (iq_bin_count[b] > 0) {
									float mean = iq_bin_sum[b] / (float)iq_bin_count[b];
									re += mean * cur_c;
									im += mean * cur_s;
								}
								float next_c = cur_c * c1 - cur_s * s1;
								float next_s = cur_c * s1 + cur_s * c1;
								cur_c = next_c; cur_s = next_s;
							}
							re *= norm;
							im *= norm;
							if (p == 1) { // CW
								cw_harms[k].mag = sqrtf(re*re + im*im);
								cw_harms[k].phase = atan2f(re, im);
							} else { // CCW
								ccw_harms[k].mag = sqrtf(re*re + im*im);
								ccw_harms[k].phase = atan2f(re, im);
							}
						}
						// Snapshot per-bin mean for configurator readout.
						{
							float* dst = (p == 1) ? this->cw_bins : this->ccw_bins;
							for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++)
								dst[b] = (iq_bin_count[b] > 0) ? (iq_bin_sum[b] / (float)iq_bin_count[b]) : 0.0f;
						}
						// DC component = average of all bin means (discarded
						// by the k=1..N DFT above). Log it so the user can
						// see how much constant torque (friction + DC offset)
						// the PID needed during this direction's sweep.
						{
							float dc_sum = 0.0f; uint32_t dc_n = 0;
							for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++) {
								if (iq_bin_count[b] > 0) {
									dc_sum += iq_bin_sum[b] / (float)iq_bin_count[b];
									dc_n++;
								}
							}
							if (dc_n > 0) {
								const char* dc_dir = (p == 1) ? "CW" : "CCW";
								float dc_val = dc_sum / (float)dc_n;
								if (p == 1) cw_dc_avg = dc_val;
								broadcastCalibLog(0, "DFT DC avg (%s): %.1f", dc_dir, dc_val);
							}
						}
						total_samples += dir_samples;
						if (max_iq_cmd_used > global_max_iq) global_max_iq = max_iq_cmd_used;
					}
				}

				// If DFT clamped during this iteration, abort the whole
				// iteration loop so the retry can lower Kp and restart.
				if (dft_clamped) break;

				// --- CW+CCW HARMONICS COMBINATION (Piccoli phase alignment) ---
				// Average magnitudes and phases from CW and CCW sweeps.
				// Magnitude averaging cancels AC-neutral friction bias.
				// Phase averaging with unwrapping cancels PID tracking lag delta.
				if (total_samples > 0) {
					// On first pass, clear THIS profile's table (active_tbl already
					// cleared at the top of the profile loop, but this guards against
					// any stray state from a previous calibration run).
					if (dft_iter == 0) {
						memset(active_tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
					}

					// Diagnostics on first iteration
					if (dft_iter == 0 && cw_harms[1].mag > 5000.0f) {
						broadcastCalibLog(0, "Warning: High motor eccentricity detected");
					}

					// Build combined candidate list from averaged CW+CCW magnitudes
					struct Sel { float mag; uint16_t order; float phase; };
					Sel best[COGGING_HARMONICS_COUNT];
					memset(best, 0, sizeof(best));

					// Start at 1 to include Order 1, 2, 3, etc. (Skip 0, which is DC offset)
					for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
    
    				// 1. Average amplitude
    				float avg_mag = (cw_harms[k].mag + ccw_harms[k].mag) / 2.0f;

					// 2. Convert phases to complex vectors (Re/Im) to avoid wrapping bugs
					float cw_re = cosf(cw_harms[k].phase);
					float cw_im = sinf(cw_harms[k].phase);
					
					float ccw_re = cosf(ccw_harms[k].phase);
					float ccw_im = sinf(ccw_harms[k].phase);

					// 3. Average the vectors
					float avg_re = (cw_re + ccw_re) / 2.0f;
					float avg_im = (cw_im + ccw_im) / 2.0f;

					// 4. Extract the combined phase
					float avg_phase = atan2f(avg_im, avg_re);

						// Insertion sort: keep top COGGING_HARMONICS_COUNT by magnitude
						for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (avg_mag > best[n].mag) {
								for (int s = COGGING_HARMONICS_COUNT - 1; s > n; s--) {
									best[s] = best[s-1];
								}
								best[n].mag = avg_mag;
								best[n].order = k;
								best[n].phase = avg_phase;
								break;
							}
						}
					}

					// Phasor-add the measured residual into THIS profile's table
					// (active_tbl). On iter 0 the table is empty so this assigns the full
					// measured cogging; on later iters the running estimate is refined.
					for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						if (best[n].mag <= 0.0f) continue;
						uint16_t order = best[n].order;
						int slot = -1;
						for (int j = 0; j < COGGING_HARMONICS_COUNT; j++) {
							if (active_tbl[j].order == order && active_tbl[j].amplitude > 0.0f) {
								slot = j; break;
							}
						}
						if (slot < 0) {
							for (int j = 0; j < COGGING_HARMONICS_COUNT; j++) {
								if (active_tbl[j].amplitude <= 0.0f) { slot = j; break; }
							}
						}
						if (slot >= 0) {
#ifdef COGGING_DFT_USE_IQ_CMD
							float ex_re = active_tbl[slot].amplitude * cosf(active_tbl[slot].phase);
							float ex_im = active_tbl[slot].amplitude * sinf(active_tbl[slot].phase);
							float new_re = best[n].mag * cosf(best[n].phase);
							float new_im = best[n].mag * sinf(best[n].phase);
							float sum_re = ex_re + new_re;
							float sum_im = ex_im + new_im;
							active_tbl[slot].order = order;
							active_tbl[slot].amplitude = sqrtf(sum_re*sum_re + sum_im*sum_im);
							active_tbl[slot].phase = atan2f(sum_im, sum_re);
#else
							active_tbl[slot].order = order;
							active_tbl[slot].amplitude = best[n].mag;
							active_tbl[slot].phase = best[n].phase;
#endif
						}
					}
				}

				broadcastCalibLog(0, "DFT iter %u max err: %.2f deg", dft_iter + 1, iter_max_err_deg);

				// --- Store and broadcast CW/CCW raw harmonics for configurator offset tuning ---
				{
					struct { float mag; uint16_t order; float phase; } top[COGGING_HARMONICS_COUNT];

					// Extract top CW harmonics
					memset(top, 0, sizeof(top));
					for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
						if (cw_harms[k].mag <= 0.0f) continue;
						for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (cw_harms[k].mag > top[n].mag) {
								for (int s = COGGING_HARMONICS_COUNT - 1; s > n; s--) top[s] = top[s-1];
								top[n].mag = cw_harms[k].mag;
								top[n].order = (uint16_t)k;
								top[n].phase = cw_harms[k].phase;
								break;
							}
						}
					}
					for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						cw_store[n].order = top[n].order;
						cw_store[n].amplitude = top[n].mag;
						cw_store[n].phase = top[n].phase;
					}

					// Broadcast CW data in chunks
					{
						std::string cwStr = "CWD:";
						for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (cw_store[n].amplitude <= 0.0f) continue;
							char chunk[48];
							snprintf(chunk, sizeof(chunk), "%u:%.0f:%.0f,", cw_store[n].order, cw_store[n].amplitude, cw_store[n].phase * 1000.0f);
							if (cwStr.length() + strlen(chunk) > 100) {
								broadcastCalibLog(0, "%s", cwStr.c_str());
								cwStr = "CWD:";
							}
							cwStr += chunk;
						}
						if (cwStr.length() > 4) broadcastCalibLog(0, "%s", cwStr.c_str());
					}

					// Extract top CCW harmonics
					memset(top, 0, sizeof(top));
					for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
						if (ccw_harms[k].mag <= 0.0f) continue;
						for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (ccw_harms[k].mag > top[n].mag) {
								for (int s = COGGING_HARMONICS_COUNT - 1; s > n; s--) top[s] = top[s-1];
								top[n].mag = ccw_harms[k].mag;
								top[n].order = (uint16_t)k;
								top[n].phase = ccw_harms[k].phase;
								break;
							}
						}
					}
					for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						ccw_store[n].order = top[n].order;
						ccw_store[n].amplitude = top[n].mag;
						ccw_store[n].phase = top[n].phase;
					}

					// Broadcast CCW data in chunks
					{
						std::string ccwStr = "CCWD:";
						for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (ccw_store[n].amplitude <= 0.0f) continue;
							char chunk[48];
							snprintf(chunk, sizeof(chunk), "%u:%.0f:%.0f,", ccw_store[n].order, ccw_store[n].amplitude, ccw_store[n].phase * 1000.0f);
							if (ccwStr.length() + strlen(chunk) > 100) {
								broadcastCalibLog(0, "%s", ccwStr.c_str());
								ccwStr = "CCWD:";
							}
							ccwStr += chunk;
						}
						if (ccwStr.length() > 5) broadcastCalibLog(0, "%s", ccwStr.c_str());
					}

					cwccw_data_valid = true;
				}
								// Degradation check DISABLED: all iterations always run.
				// To re-enable, uncomment the block below.
				// if (dft_iter > 0 && iter_max_err_deg >= prev_iter_err) {
				// 	memcpy(cogging_harmonics, prev_harmonics, sizeof(cogging_harmonics));
				// 	broadcastCalibLog(0, "DFT degraded, keeping previous table (%.2f deg)", prev_iter_err);
				// 	break;
				// }
				// Save backup of current best table (this profile's table)
				memcpy(prev_harmonics, active_tbl, sizeof(prev_harmonics));
			}

			// --- DFT CLAMP RETRY ---
			// If the acquisition saturated torque, Kp is too high for this RPM.
			// Lower it and restart. Retries until Kp hits floor (50) or DFT succeeds.
			if (dft_clamped && !emergency && hasPower()) {
				float lowered_kp = pid_soft.Kp / 1.25f;
				if (lowered_kp < 50.0f) lowered_kp = 50.0f;
				broadcastCalibLog(0, "DFT clamped at Kp:%.0f! Retrying with Kp:%.0f.", pid_soft.Kp, lowered_kp);
				pid_soft.Kp = lowered_kp;
				this->coggingSpeedP = lowered_kp;
#ifdef COGGING_PHASE_SHIFT_MULTIRPM
				{
					uint8_t store_src = (rpm_profile < COGGING_MAX_CALIB_PROFILES) ? rpm_profile : (COGGING_MAX_CALIB_PROFILES - 1);
					this->cogging_calib_pidP[store_src] = (uint32_t)lowered_kp;
				}
#else
				this->cogging_calib_pidP[rpm_profile] = (uint32_t)lowered_kp;
#endif
				arm_pid_init_f32(&pid_soft, 1);
				memset(active_tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
				applySafeTorque(0);
				Delay(250);
				continue;  // restart DFT for this profile with lower Kp
			}
			// DFT succeeded without clamp — exit retry loop
			break;

			} // end DFT retry loop

			// Coast settle after DFT before next profile.
			applySafeTorque(0);
			{
				uint32_t settle_ms = cogging_warmup_ms;
				if (settle_ms > 4000) settle_ms = 4000;
				uint32_t settleStart = HAL_GetTick();
				while (HAL_GetTick() - settleStart < settle_ms && !emergency && hasPower()) {
					refreshWatchdog();
					Delay(10);
				}
				arm_pid_init_f32(&pid_soft, 1);
			}

			// Each profile wrote directly into its own table (active_tbl), so nothing
			// to copy here. Mark validity ONLY if this profile actually collected
			// samples (an aborted profile must not leave a valid flag on an empty
			// table, or blendHarmonicTables() would output 0 at that speed band).
		if (total_samples > 0) {
			any_profile_succeeded = true;

#ifdef COGGING_PHASE_SHIFT_MULTIRPM
			// --- MULTI-RPM PHASE SHIFT: test all 24 RPM breakpoints ---
			// Profile 0 (first non-zero RPM) is the baseline — full harmonic
			// table saved.  Higher profiles only extract phase lag & attenuation
			// for the MASTER dominant harmonic (locked in during profile 0).
			// Uses cw_store/ccw_store (top-20 harmonics) like PHASE_SHIFT_CAL.
			{
				static uint16_t master_dom_order = 1;
				static float master_ref_mag = 0.0f;

				// Lock dominant harmonic on profile 0 only
				if (rpm_profile == 0) {
					float max_cw_mag = 0.0f;
					for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						if (cw_store[n].amplitude > max_cw_mag) {
							max_cw_mag = cw_store[n].amplitude;
							master_dom_order = cw_store[n].order;
						}
					}
				}

				// Get dominant harmonic phase & magnitude from both directions
				float cw_ph = 0, ccw_ph = 0, cw_mg = 0, ccw_mg = 0;
				for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
					if (cw_store[n].order == master_dom_order) { cw_ph = cw_store[n].phase; cw_mg = cw_store[n].amplitude; }
					if (ccw_store[n].order == master_dom_order) { ccw_ph = ccw_store[n].phase; ccw_mg = ccw_store[n].amplitude; }
				}
				float avg_mag = (cw_mg + ccw_mg) / 2.0f;

				// Phase unwrapping
				float phase_diff = cw_ph - ccw_ph;
				if (phase_diff > PI) phase_diff -= 2.0f * PI;
				if (phase_diff < -PI) phase_diff += 2.0f * PI;

				if (rpm_profile == 0) {
					// Profile 0: save the full spatial cogging table.
#ifndef COGGING_DISABLE_SCALE_CURVE
					scale_curve_values[0] = 1.0f;
#endif
					phase_advance_curve_values[0] = 0.0f;
					master_ref_mag = avg_mag;
					broadcastCalibLog(0, "Base map %.1f RPM locked H%u ref %.0f",
						calib_rpm, (unsigned int)master_dom_order, master_ref_mag);
				} else {
					// Higher profiles: extract phase lag & attenuation
					float scale = (master_ref_mag > 0.0f) ? avg_mag / master_ref_mag : 1.0f;
					scale = clip<float>(scale, 0.1f, 3.0f);

					float lag_mech_deg = 0.0f;
					if (master_dom_order > 0) {
						float lag_mech_rad = fabsf(phase_diff / 2.0f) / (float)master_dom_order;
						lag_mech_deg = lag_mech_rad * (180.0f / PI);
					}

					// Write directly at rpm_profile+1 index (matches scale_curve_rpm_points)
					uint8_t idx = rpm_profile + 1;
					if (idx < SCALE_CURVE_POINTS) {
#ifndef COGGING_DISABLE_SCALE_CURVE
						scale_curve_values[idx] = scale;
#endif
						phase_advance_curve_values[idx] = lag_mech_deg;
						if (idx + 1 > scale_curve_count) scale_curve_count = idx + 1;
					}

					broadcastCalibLog(0, "RPM %.1f idx %u scale %.2f shift %.2f H%u",
						calib_rpm, idx, scale, lag_mech_deg, (unsigned int)master_dom_order);
				}
			}
#elif defined(COGGING_PHASE_SHIFT_CAL)
				// --- PHASE-SHIFT METHOD: measure phase lag & attenuation ---
				// The RPM breakpoints (scale_curve_rpm_points) are FIXED and must
				// match the configurator RPM_POINTS or values won't display.
				// Each calibration profile is mapped to the NEAREST breakpoint index.
				//
				// Find the dominant harmonic in stored CW/CCW data for phase tracking.
				uint16_t dom_order = 1;
				float max_cw_mag = 0.0f;
				for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
					if (cw_store[n].amplitude > max_cw_mag) {
						max_cw_mag = cw_store[n].amplitude;
						dom_order = cw_store[n].order;
					}
				}

				float cw_phase_dom = 0.0f, ccw_phase_dom = 0.0f;
				float cw_mag_dom = 0.0f, ccw_mag_dom = 0.0f;
				for (uint8_t n = 0; n < COGGING_HARMONICS_COUNT; n++) {
					if (cw_store[n].order == dom_order) {
						cw_phase_dom = cw_store[n].phase;
						cw_mag_dom = cw_store[n].amplitude;
					}
					if (ccw_store[n].order == dom_order) {
						ccw_phase_dom = ccw_store[n].phase;
						ccw_mag_dom = ccw_store[n].amplitude;
					}
				}

				// Average magnitude cancels AC-neutral friction bias
				float avg_mag = (cw_mag_dom + ccw_mag_dom) / 2.0f;

				// Find nearest RPM breakpoint index for this profile's calib_rpm
				uint8_t bp_idx = 0;
				float best_dist = 999999.0f;
				for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
					float dist = fabsf(scale_curve_rpm_points[i] - calib_rpm);
					if (dist < best_dist) {
						best_dist = dist;
						bp_idx = i;
					}
				}

				if (rpm_profile == 0) {
					// Plateau (index 0) and first calibration point at nearest breakpoint.
#ifndef COGGING_DISABLE_SCALE_CURVE
					scale_curve_values[0] = 1.0f;
					scale_curve_values[bp_idx] = 1.0f;
#endif
					phase_advance_curve_values[0] = 0.0f;
					phase_advance_curve_values[bp_idx] = 0.0f;
					// Save reference magnitude for computing attenuation in higher profiles.
					this->last_cogging_scale = avg_mag;
					broadcastCalibLog(0, "Base map at %.1f RPM -> bp[%u]=%.0f RPM (dom %u, ref mag %.0f)",
						calib_rpm, bp_idx, scale_curve_rpm_points[bp_idx], dom_order, avg_mag);
				} else {
					// Phase unwrapping for difference calculation
					float phase_diff = cw_phase_dom - ccw_phase_dom;
					if (phase_diff > PI) phase_diff -= 2.0f * PI;
					if (phase_diff < -PI) phase_diff += 2.0f * PI;

#ifndef COGGING_DISABLE_SCALE_CURVE
					// Scale: attenuation ratio vs. the base profile's dominant magnitude.
					float scale = (this->last_cogging_scale > 0.0f) ? avg_mag / this->last_cogging_scale : 1.0f;
					scale = clip<float>(scale, 0.1f, 3.0f);
					scale_curve_values[bp_idx] = scale;
#endif

					// Phase advance: phase_diff is CW-CCW in electrical radians.
					float lag_mech_rad = fabsf(phase_diff / 2.0f) / (float)dom_order;
					float lag_mech_deg = lag_mech_rad * (180.0f / PI);
					phase_advance_curve_values[bp_idx] = lag_mech_deg;

					broadcastCalibLog(0, "RPM %.1f -> bp[%u]=%.0f RPM | Atten: %.2f | Shift: %.2f deg (dom %u)",
						calib_rpm, bp_idx, scale_curve_rpm_points[bp_idx],
#ifndef COGGING_DISABLE_SCALE_CURVE
						scale,
#else
						1.0f,
#endif
						lag_mech_deg, dom_order);
				}

				// scale_curve_count = number of distinct breakpoints used
				if (bp_idx + 1 > scale_curve_count)
					scale_curve_count = bp_idx + 1;
#endif // COGGING_PHASE_SHIFT_CAL / COGGING_PHASE_SHIFT_MULTIRPM

#ifndef COGGING_DISABLE_BLEND
			// Mark per-RPM blend tables as valid for runtime blending.
			// Independent of phase-shift: both can coexist.
			if (rpm_profile == 1) this->rpm2_table_valid = true;
			else if (rpm_profile >= 2) this->rpm3_table_valid = true;
#endif
			// Broadcast combined harmonic table for this profile immediately
			// so the configurator can display it right away.
			// Prefix with "profile:N:" so the configurator routes to the correct
			// per-profile storage without needing the adr from the serial frame.
			{
				uint8_t profile_adr = (rpm_profile >= 2) ? 2 : rpm_profile;
				std::string hs = "profile:";
				hs += std::to_string(rpm_profile + 1); // 1-based profile number
				hs += ":";
				bool any = false;
				for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
					if (active_tbl[i].amplitude > 0.0f || active_tbl[i].order > 0) {
						if (any) hs += ",";
						any = true;
						hs += std::to_string(active_tbl[i].order) + ":";
						hs += std::to_string((int16_t)active_tbl[i].amplitude) + ":";
						hs += std::to_string((int16_t)(active_tbl[i].phase * 1000.0f));
					}
				}
				if (!any) hs += "0:0:0";
				CommandHandler::broadcastCommandReply(CommandReply(hs, profile_adr),
					(uint32_t)TMC4671_commands::coggingHarmonics, CMDtype::get);
			}

			// --- VERIFICATION PASS: measure residual cogging with feedforward active ---
			// Runs CW and CCW sweeps using the finalized table as feedforward,
			// then prints the detected harmonics per direction so the user can see
			// how much cogging remains after compensation in each direction.
			if (!emergency && hasPower() && total_samples > 0) {
				broadcastCalibLog(0, "Verification pass: measuring residual with feedforward...");
				applySafeTorque(0);
				Delay(500);

				float ver_rpm = calib_rpm;
				// Add warmup time so the motor accelerates smoothly before
				// binning starts — otherwise the startup transient (high iq_pid
				// while accelerating from rest) contaminates the DFT with
				// spurious low-order content.
				// Use 1.5x rev time (same as main DFT's rev_ms formula) so
				// integrated_distance has margin to reach 1.0 even with
				// velocity ripple from residual cogging.
				uint32_t ver_rev_ms = (uint32_t)((60.0f / ver_rpm) * 1500.0f) + cogging_warmup_ms;
				uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR);
				float dt_sec = (float)period_us / 1000000.0f;
				uint32_t dft_decimation_ratio = 2;

				int8_t dirs[2] = {1, -1};
				const char* dir_names[2] = {"CW", "CCW"};

				for (uint8_t di = 0; di < 2; di++) {
					int8_t p = dirs[di];
					float target_rpm = (p == 1) ? ver_rpm : -ver_rpm;
					float full_vel_turns = target_rpm / 60.0f;

					arm_pid_init_f32(&pid_soft, 1);
					memset(iq_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(iq_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(iq_bin_sum, 0, COGGING_DFT_BIN_COUNT * sizeof(float));
					memset(iq_bin_count, 0, COGGING_DFT_BIN_COUNT * sizeof(uint16_t));
					uint32_t ver_samples = 0;

					applySafeTorque(0);
					Delay(300);

					float target_pos_f = getFilteredPosition();
					float integrated_distance = 0.0f;
					float ramp_vel_turns = 0.0f;
					// Ramp-up rate matches the main DFT: accelerate smoothly from
					// 0 to target over cogging_warmup_ms.  Tiny floor ensures the
					// target moves at least ~1 encoder count/iteration immediately.
					float ramp_rate = full_vel_turns / (float)cogging_warmup_ms * 1000.0f; // turns/s²
					calibStartTime = HAL_GetTick();
					uint32_t next_tick = micros();
					uint32_t dft_decimation_counter = 0;

					startCalibTimers(TIM_TMC_ARR);
					while (HAL_GetTick() - calibStartTime < ver_rev_ms && !emergency && hasPower()) {
						next_tick += period_us;

						// Velocity ramp: 0 → full_vel_turns during warmup,
						// then hold at full speed.
						float elapsed = (float)(HAL_GetTick() - calibStartTime);
						if (elapsed < (float)cogging_warmup_ms) {
							ramp_vel_turns = ramp_rate * elapsed * 0.001f; // turns/s
							if (ramp_vel_turns < fabsf(full_vel_turns) * 0.005f)
								ramp_vel_turns = fabsf(full_vel_turns) * 0.005f;
						} else {
							ramp_vel_turns = full_vel_turns;
						}
						float step = ramp_vel_turns * dt_sec;
						target_pos_f += step;
						if (target_pos_f >= 1.0f) target_pos_f -= 1.0f;
						if (target_pos_f < 0.0f) target_pos_f += 1.0f;

						float actual_pos_f = getFilteredPosition();
						float error = getWrappedError(target_pos_f, actual_pos_f);
						float iq_pid = arm_pid_f32(&pid_soft, error);

						// Feedforward from the finalized active_tbl
						float cog_comp = 0.0f;
						float angle_rad = actual_pos_f * 2.0f * PI;
						for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++) {
							if (active_tbl[h].amplitude > 0.0f) {
								cog_comp += active_tbl[h].amplitude * arm_sin_f32(angle_rad * active_tbl[h].order + active_tbl[h].phase);
							}
						}
						float iq_applied = iq_pid + cog_comp;
						iq_applied = clip<float,float>(iq_applied, -max_test_torque, max_test_torque);
						applySafeTorque(iq_applied);

						captureDebug(TMC4671CoggingDebugPhase::Validation,
							target_rpm, target_pos_f, actual_pos_f, error,
							iq_pid, 0.0f, cog_comp, iq_pid, iq_applied,
							0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

						// Only count distance and bin during the first full
						// revolution of steady-state motion (after warmup).
						// This matches the main DFT's pattern exactly:
						// integrated_distance is incremented INSIDE the gate,
						// so it only counts steps that are actually binned.
						if (integrated_distance < 1.0f && (HAL_GetTick() - calibStartTime > cogging_warmup_ms)) {
							integrated_distance += fabs(step);
							dft_decimation_counter++;
							if (dft_decimation_counter % dft_decimation_ratio == 0) {
								float iq = iq_pid;
								// Spatial binning — same rationale as the main DFT.
								// Verification is even more sensitive to noise because
								// the residual iq_pid is small and the PID noise floor
								// is unchanged, so SNR is poor without binning.
								uint32_t bin = (uint32_t)(actual_pos_f * (float)COGGING_DFT_BIN_COUNT);
								if (bin >= COGGING_DFT_BIN_COUNT) bin = COGGING_DFT_BIN_COUNT - 1;
								iq_bin_sum[bin] += iq;
								if (iq_bin_count[bin] != 0xFFFF) iq_bin_count[bin]++;
								ver_samples++;
							}
						}

						refreshWatchdog();
#ifdef TIM_CALIBRATION
						if (this->calibTimer != nullptr) {
							this->WaitForNotification();
						} else
#endif
						{
							while ((micros() - next_tick) & 0x80000000) { }
						}
					}
					stopCalibTimers();
					applySafeTorque(0);

					if (ver_samples > 0) {
						struct { float mag; float phase; uint16_t order; } top[20];
						memset(top, 0, sizeof(top));
						float norm = 2.0f / (float)COGGING_DFT_BIN_COUNT;
						for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
							float re = 0.0f, im = 0.0f;
							float s_start, c_start;
							arm_sin_cos_f32((float)k * 180.0f / (float)COGGING_DFT_BIN_COUNT, &s_start, &c_start);
							float s1, c1;
							arm_sin_cos_f32((float)k * 360.0f / (float)COGGING_DFT_BIN_COUNT, &s1, &c1);
							float cur_s = s_start, cur_c = c_start;
							for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++) {
								if (iq_bin_count[b] > 0) {
									float mean = iq_bin_sum[b] / (float)iq_bin_count[b];
									re += mean * cur_c;
									im += mean * cur_s;
								}
								float next_c = cur_c * c1 - cur_s * s1;
								float next_s = cur_c * s1 + cur_s * c1;
								cur_c = next_c; cur_s = next_s;
							}
							re *= norm;
							im *= norm;
							float mag = sqrtf(re*re + im*im);
							for (int n = 0; n < 20; n++) {
								if (mag > top[n].mag) {
									for (int s = 19; s > n; s--) top[s] = top[s-1];
									top[n].mag = mag;
									top[n].phase = atan2f(re, im);
									top[n].order = (uint16_t)k;
									break;
								}
							}
						}
						// Sort by order
						for (int i = 0; i < 19; i++) {
							for (int j = i + 1; j < 20; j++) {
								if (top[i].order > top[j].order && top[j].order > 0) {
									auto tmp = top[i]; top[i] = top[j]; top[j] = tmp;
								}
							}
						}

						// Save per-bin means to ver_cw_bins / ver_ccw_bins so the
						// configurator can read them via coggingBins adr 2-3.
						{
							float* dst = (p == 1) ? this->ver_cw_bins : this->ver_ccw_bins;
							for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++)
								dst[b] = (iq_bin_count[b] > 0) ? (iq_bin_sum[b] / (float)iq_bin_count[b]) : 0.0f;
						}
						// Save top-20 DFT harmonics to ver_cw_top / ver_ccw_top
						// so the configurator can read them via coggingBins adr 4-5.
						{
							Harmonic* dst = (p == 1) ? this->ver_cw_top : this->ver_ccw_top;
							for (int n = 0; n < 20; n++) {
								dst[n].order = top[n].order;
								dst[n].amplitude = top[n].mag;
								dst[n].phase = top[n].phase;
							}
						}

						broadcastCalibLog(0, "VERIFY %s residual (order : amplitude : phase_rad):", dir_names[di]);
						for (int n = 0; n < 20; n++) {
							if (top[n].order == 0 && top[n].mag <= 0.0f) continue;
							broadcastCalibLog(0, "%u : %.1f : %.4f",
								top[n].order, top[n].mag, top[n].phase);
						}
						// Broadcast verify harmonics so they appear in the
						// Harmonic Editor tab / calibration debug text box.
						{
							std::string bs = "VERIFY:";
							bs += dir_names[di];
							bs += ":";
							bool any = false;
							for (int n = 0; n < 20; n++) {
								if (top[n].order == 0 && top[n].mag <= 0.0f) continue;
								if (any) bs += ",";
								any = true;
								bs += std::to_string(top[n].order) + ":";
								bs += std::to_string((int16_t)top[n].mag) + ":";
								bs += std::to_string((int16_t)(top[n].phase * 1000.0f));
							}
							if (!any) bs += "0:0:0";
							CommandHandler::broadcastCommandReply(CommandReply(bs, 0),
								(uint32_t)TMC4671_commands::coggingCwCcw, CMDtype::get);
						}
					}
				}
			}
		}

#ifdef COGGING_ACCEL_BASED_DFT
			// --- ACCELERATION-BASED DFT STEP (Copper/Piccoli method - continuous ramp) ---
			if (!emergency && hasPower() && total_samples > 0) {
				// FF is now applied inline by ffWait() and measureTravel() —
				// no background thread dependency. Works for all encoder types.
				this->calib_max_torque = max_test_torque;
				this->calib_ff_base_torque = 0.0f;
				applySafeTorque(0); Delay(500);

				// --- Step A: measure stationary velocity noise → motion threshold ---
				float vel_thresh = 0.005f;
				{
					float v_noise[3] = {};
					for (int vi = 0; vi < 3 && !emergency && hasPower(); vi++) {
						float vsum = 0.0f; uint32_t vn = 0;
						uint32_t v0 = HAL_GetTick();
						float prev_pos = getFilteredPosition();
						uint32_t prev_us = micros();
						while (HAL_GetTick() - v0 < 500 && !emergency) {
							float pos = getFilteredPosition();
							uint32_t now_us = micros();
							float dt = (float)(now_us - prev_us) / 1e6f;
							float dv = 0.0f;
							if (dt > 0.0001f && dt < 0.1f)
								dv = fabsf(getWrappedError(prev_pos, pos) / dt);
							vsum += dv; vn++;
							g_tmc4671_cogging_debug.accel_vel_now = dv * 60.0f;
							g_tmc4671_cogging_debug.accel_vel_rpm = dv * 60.0f;
							g_tmc4671_cogging_debug.accel_vel_turns_s = dv;
							prev_pos = pos; prev_us = now_us;
							Delay(100); refreshWatchdog();
						}
						v_noise[vi] = (vn > 0) ? (vsum / (float)vn) : 0.0f;
					}
					float v_mean = (v_noise[0] + v_noise[1] + v_noise[2]) / 3.0f;
					float v_var = 0.0f;
					for (int vi = 0; vi < 3; vi++) { float d = v_noise[vi] - v_mean; v_var += d*d; }
					float v_std = sqrtf(v_var / 2.0f);
					vel_thresh = fmaxf(0.005f, 9.0f * v_std);
					g_tmc4671_cogging_debug.accel_vel_thresh = vel_thresh;
					g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_NoiseFloor;
					broadcastCalibLog(0, "Accel DFT: vel noise mean=%.5f std=%.5f thresh=%.5f turns/s",
						v_mean, v_std, vel_thresh);
				}


// --- Step B: find dmax/dmin ---
			//   dmax = MINIMUM torque that produces sustained rotation from
			//          rest at ANY rotor detent.
			//   dmin = MINIMUM torque that sustains slow rotation once the
			//          rotor is already moving.
			//
			// dmax ramp: start from the average cogging residual (not peak),
			// apply torque CONTINUOUSLY (never drop to 0 between steps),
			// increment by +10. At each step wait 1 s → if travel > threshold
			// then wait up to an additional 5 s → if the rotor is STILL moving
			// (not just a single cog-jump) that level is dmax.
			float dmax_torque = 0.0f;
			float dmin_torque = 0.0f;
			const uint8_t ACCEL_ITERS = MAX_DFT_ITERATIONS;
			const uint32_t cpr = getEncoder()->getCpr();

			// Apply base torque + cogging feed-forward + update debug.
			// Motor receives t + FF so cogging doesn't fight the probe,
			// but accel_test_current = t (base only) — the logged dmax
			// is the base torque, not the FF-inflated value.
			auto applyTorqueFF = [&](float t) -> void {
				calib_ff_base_torque = t;   // so ffWait()/measureTravel() use the right base
				float p = getFilteredPosition();
				float cog = 0.0f; float ar = p * 2.0f * PI;
				for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++)
					if (active_tbl[h].amplitude > 0.0f)
						cog += active_tbl[h].amplitude * arm_sin_f32(ar * active_tbl[h].order + active_tbl[h].phase);
				float at = clip<float,float>(t + this->cogging_scale * cog, -max_test_torque, max_test_torque);
				applySafeTorque(at);
				g_tmc4671_cogging_debug.accel_test_current = t;
			};

			// Active FF wait: continuously apply (base + FF) while waiting.
			auto ffWait = [&](uint32_t duration_ms) {
				uint32_t wait_end = HAL_GetTick() + duration_ms;
				float prev_pos = getFilteredPosition();
				uint32_t prev_us = micros();
				while (HAL_GetTick() < wait_end && !emergency && hasPower()) {
					float pos = getFilteredPosition();
					float ff = computeCoggingFF(pos);
					float total = clip<float,float>(
						calib_ff_base_torque + ff,
						-max_test_torque, max_test_torque);
					applySafeTorque(total);
					g_tmc4671_cogging_debug.accel_test_current = calib_ff_base_torque;

					uint32_t now_us = micros();
					float dt = (float)(now_us - prev_us) / 1e6f;
					if (dt > 0.0001f && dt < 0.1f) {
						float delta = getWrappedError(prev_pos, pos);
						float v_turns_s = delta / dt;
						float v_rpm = v_turns_s * 60.0f;
						g_tmc4671_cogging_debug.accel_vel_now = fabsf(v_rpm);
						g_tmc4671_cogging_debug.accel_vel_rpm = fabsf(v_rpm);
						g_tmc4671_cogging_debug.accel_vel_turns_s = v_turns_s;
					}
					prev_pos = pos;
					prev_us = now_us;

					refreshWatchdog();
				Delay(1);  // 1 tick = 1ms = ~1kHz FF update (was Delay(5)=200Hz)
			}
		};

			// Cumulative distance travelled (turns) since a given encoder count.
			// Applies FF inline during travel so cogging doesn't fight the probe.
			auto measureTravel = [&](int32_t ref_pos, uint32_t duration_ms) -> float {
				uint32_t t0 = HAL_GetTick();
				int32_t last = ref_pos;
				float cdist = 0.0f;
				float prev_pos_inst = getFilteredPosition();
				uint32_t prev_us_inst = micros();
				while (HAL_GetTick() - t0 < duration_ms && !emergency && hasPower()) {
					int32_t raw = getEncoder()->getPos();
					cdist += fabsf((float)(raw - last) / (float)cpr);
					last = raw;

					// Instantaneous velocity: position delta / time delta.
					// This shows "is it moving right now?" vs cumulative average.
					float pos = getFilteredPosition();
					uint32_t now_us = micros();
					float dt_inst = (float)(now_us - prev_us_inst) / 1e6f;
					if (dt_inst > 0.0001f && dt_inst < 1.0f) {
						float delta_inst = getWrappedError(prev_pos_inst, pos);
						float v_inst_rpm = (delta_inst / dt_inst) * 60.0f;
						g_tmc4671_cogging_debug.accel_vel_now = fabsf(v_inst_rpm);
						g_tmc4671_cogging_debug.accel_vel_rpm = fabsf(v_inst_rpm);
						g_tmc4671_cogging_debug.accel_vel_turns_s = delta_inst / dt_inst;
					}
					prev_pos_inst = pos;
					prev_us_inst = now_us;

					float ff = computeCoggingFF(pos);
					float total = clip<float,float>(
						calib_ff_base_torque + ff,
						-max_test_torque, max_test_torque);
					applySafeTorque(total);
					g_tmc4671_cogging_debug.accel_test_current = calib_ff_base_torque;

					refreshWatchdog();
					Delay(1);
				}
				return cdist;
			};

			{
				float start_torque = fabsf(cw_dc_avg);
				const float DMAX_STEP = 1.00f;
				const float DMAX_CAP = max_test_torque * 0.7f;

				g_tmc4671_cogging_debug.accel_ver_peak = start_torque;
				g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_FindDmax;
				broadcastCalibLog(0, "Accel DFT: finding dmax (ramp from %.0f, step +%.0f, cap %.0f)...",
					start_torque, DMAX_STEP, DMAX_CAP);

				float test = start_torque;
				float prev_dmax = 0.0f;
				uint8_t same_count = 0;
				static constexpr uint8_t CONVERGE_COUNT = 5;
				applySafeTorque(0); g_tmc4671_cogging_debug.accel_test_current = 0.0f;
				// Brief settle before first probe (base=0, FF-only)
				calib_ff_base_torque = 0.0f;
				ffWait(1000);

				while (same_count < CONVERGE_COUNT && test < DMAX_CAP && !emergency && hasPower()) {
					// ---- Ramp up from current test level to find next sustained breakaway ----
					while (test < DMAX_CAP && !emergency && hasPower()) {
						applyTorqueFF(test);

						// Stage 1: wait 1 s with active FF, then measure velocity
						ffWait(1000);
						bool moving = false;
						if (!emergency && hasPower()) {
							(void)measureTravel(getEncoder()->getPos(), 100);
							// Decision from instantaneous velocity (accel_vel_now in RPM).
							// vel_thresh is in turns/s; convert RPM → turns/s.
							moving = (g_tmc4671_cogging_debug.accel_vel_now / 60.0f > vel_thresh);
						}
						broadcastCalibLog(0, "Accel DFT: dmax probe %.0f -> %.0f RPM %s",
							test, g_tmc4671_cogging_debug.accel_vel_now,
							moving ? "moving" : "still");

						if (moving) {
							// Stage 2: wait 2 s with active FF, then measure velocity
							ffWait(2000);
							bool sustained = false;
							if (!emergency && hasPower()) {
								(void)measureTravel(getEncoder()->getPos(), 100);
								sustained = (g_tmc4671_cogging_debug.accel_vel_now / 60.0f > vel_thresh);
							}
							if (sustained) {
								dmax_torque = test;
								broadcastCalibLog(0, "Accel DFT: dmax = %.0f (sustained, %.0f RPM at end of 5s)",
									dmax_torque, g_tmc4671_cogging_debug.accel_vel_now);
								break;
							}
							broadcastCalibLog(0, "Accel DFT: dmax probe %.0f — moved briefly, not sustained. Ramping.", test);
						}
						test += DMAX_STEP;
					}

					if (dmax_torque <= 0.0f) {
						dmax_torque = test >= DMAX_CAP ? DMAX_CAP : test;
						broadcastCalibLog(0, "Accel DFT: dmax = %.0f (no sustained motion found)", dmax_torque);
						break;
					}

					// ---- Convergence check ----
					if (fabsf(dmax_torque - prev_dmax) <= 1.0f) {
						same_count++;
						broadcastCalibLog(0, "Accel DFT: dmax stable at %.0f (%u/%u)",
							dmax_torque, same_count, CONVERGE_COUNT);
					} else {
						same_count = 0;
						prev_dmax = dmax_torque;
						broadcastCalibLog(0, "Accel DFT: dmax increased to %.0f, resetting convergence",
							dmax_torque);
					}

					if (same_count >= CONVERGE_COUNT) break;

					// ---- Spin down: coast with zero torque, poll position-based velocity ----
					applySafeTorque(0); g_tmc4671_cogging_debug.accel_test_current = 0.0f;
					{
						uint32_t spin_down_start = HAL_GetTick();
						float sd_prev_pos = getFilteredPosition();
						uint32_t sd_prev_us = micros();
						while (HAL_GetTick() - spin_down_start < 8000 && !emergency && hasPower()) {
							Delay(150);
							float sd_pos = getFilteredPosition();
							uint32_t sd_now_us = micros();
							float sd_dt = (float)(sd_now_us - sd_prev_us) / 1e6f;
							float v = 0.0f;
							if (sd_dt > 0.0001f && sd_dt < 1.0f)
								v = fabsf(getWrappedError(sd_prev_pos, sd_pos) / sd_dt);
							g_tmc4671_cogging_debug.accel_vel_now = v * 60.0f; // RPM
							g_tmc4671_cogging_debug.accel_vel_rpm = v * 60.0f;
							g_tmc4671_cogging_debug.accel_vel_turns_s = v;
							sd_prev_pos = sd_pos; sd_prev_us = sd_now_us;
							if (v < vel_thresh) break;
							refreshWatchdog();
						}
					}
					

					// Start next ramp from the working dmax.  The rotor is at a
					// new position after spinning down — test the same torque
					// that broke away last time.
					test = dmax_torque;
					broadcastCalibLog(0, "Accel DFT: re-testing dmax from %.0f",
						test);
				}

				// ---------- dmin: per-step spin-up binary search ----------
				// Each candidate torque gets its own independent cycle:
				// rest → spin-up with dmax (guarantee moving) →
				// switch to candidate → short settle → measure speed.
				// Binary search over [5, dmax]; ~log2(N) ≈ 4-6 iterations.
				// Each iteration is self-contained so a stall at one
				// candidate doesn't poison the next.
				g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_FindDmin;

				// Block until the rotor is essentially stationary.
				auto waitForRest = [&](void) -> void {
					uint32_t quietSince = 0;
					uint32_t t0 = HAL_GetTick();
					while (HAL_GetTick() - t0 < 4000 && !emergency && hasPower()) {
						float moved = measureTravel(getEncoder()->getPos(), 40);
						if (moved < 0.0001f) {
							if (quietSince == 0) quietSince = HAL_GetTick();
							else if (HAL_GetTick() - quietSince > 120) { applySafeTorque(0); return; }
						} else {
							quietSince = 0;
						}
					}
					applySafeTorque(0);
				};

				// dmin: linear scan with dmax spin-up per step.
				//   Phase 1 — descend: candidate = dmax-1, step -1 until first coast-down.
				//              Records the stall boundary (one false stop is expected at a cogging peak).
				//   Phase 2 — ascend:  candidate = stall_point+1, step +1 until first sustained.
				//              Re-spins from dmax each step so a transient detent doesn't mask the true floor.
				// dmin = the lowest value that reliably sustains rotation.
				const float DMIN_FLOOR = 5.0f;
				const float DMIN_VEL_THRESH = 0.05f;
				float dmin_torque_local = dmax_torque; // default if descent never stalls

				// Single probe at a candidate torque. Returns true if sustained.
				auto probeCandidate = [&](float candidate) -> bool {
					// 1. Rest the rotor so the spin-up starts from standstill.
					applySafeTorque(0); g_tmc4671_cogging_debug.accel_test_current = 0.0f;
					waitForRest();

				// 2. Spin up with BREAKAWAY torque (dmax only sustains — it can't reliably
				//    free a rotor parked at the worst cogging detent from standstill).
					float spinup = dmax_torque * 1.5f;
					if (spinup > max_test_torque * 0.6f) spinup = max_test_torque * 0.6f;
					applyTorqueFF(spinup);
					(void)measureTravel(getEncoder()->getPos(), 500);

					// 3. Drop to the candidate torque and let the transient settle.
					applyTorqueFF(candidate);
					int32_t p_settle = getEncoder()->getPos();
					(void)measureTravel(p_settle, 200);

					// 4. Measure: still moving after settle means candidate sustains.
					(void)measureTravel(getEncoder()->getPos(), 250);
					float v_rpm = g_tmc4671_cogging_debug.accel_vel_now;
					bool sustains = (v_rpm / 60.0f > DMIN_VEL_THRESH);
					broadcastCalibLog(0, "Accel DFT: dmin probe %5.0f -> %.0f RPM %s",
						candidate, v_rpm, sustains ? "sustain" : "coast-down");
					return sustains;
				};

				// --- Phase 1: descend from dmax-1 until first coast-down ---
				broadcastCalibLog(0, "Accel DFT: finding dmin (descend from %.0f, step -1)...", dmax_torque);
				float stall_point = dmax_torque;
				for (float candidate = dmax_torque - 1.0f;
					 candidate >= DMIN_FLOOR && !emergency && hasPower();
					 candidate -= 1.0f) {
					if (!probeCandidate(candidate)) {
						stall_point = candidate;
						break;
					}
				}

				// --- Phase 2: ascend from stall_point+1 until first sustain ---
				if (stall_point < dmax_torque) {
					broadcastCalibLog(0, "Accel DFT: dmin verifying ascent from %.0f...", stall_point + 1.0f);
					for (float candidate = stall_point + 1.0f;
						 candidate <= dmax_torque && !emergency && hasPower();
						 candidate += 1.0f) {
						if (probeCandidate(candidate)) {
							dmin_torque_local = candidate;
							break;
						}
					}
				}

				dmin_torque = dmin_torque_local;
				if (dmin_torque < DMIN_FLOOR) dmin_torque = DMIN_FLOOR;
				if (dmin_torque > dmax_torque) dmin_torque = dmax_torque;
				broadcastCalibLog(0, "Accel DFT: dmin = %.0f", dmin_torque);

				applySafeTorque(0); g_tmc4671_cogging_debug.accel_test_current = 0.0f;
				g_tmc4671_cogging_debug.accel_dmax = dmax_torque;
				g_tmc4671_cogging_debug.accel_dmin = dmin_torque;
				broadcastCalibLog(0, "Accel DFT: dmax=%.0f dmin=%.0f, %u iterations", dmax_torque, dmin_torque, ACCEL_ITERS);
			}

				// --- ACCEL DFT ITERATION LOOP ---
				// Each iteration: sweep CW+CCW with current FF table → extract vel DFT →
				// convert to torque → update table → broadcast. Residual shrinks each pass.
				for (uint8_t a_iter = 0; a_iter < ACCEL_ITERS && !emergency && hasPower(); a_iter++) {
					if (a_iter > 0) {
						// Residual shrinks each iteration → can decrease dmin
						dmin_torque *= 0.8f;
						if (dmin_torque < 5.0f) dmin_torque = 5.0f;
						dmax_torque = dmin_torque * 1.5f;
						g_tmc4671_cogging_debug.accel_dmin = dmin_torque;
						g_tmc4671_cogging_debug.accel_dmax = dmax_torque;
						broadcastCalibLog(0, "Accel DFT iter %u: scaled dmin=%.0f dmax=%.0f", a_iter, dmin_torque, dmax_torque);
					}

				// --- Step E: re-measure J (first iteration only) ---
				float accel_J = J;
				if (a_iter == 0) {
				float j_pulse_torque = dmax_torque * 1.5f;
				if (j_pulse_torque > max_test_torque * 0.6f) j_pulse_torque = max_test_torque * 0.6f;
				{
					g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_JPulse;
					broadcastCalibLog(0, "Accel DFT: re-measuring J with anti-cog FF (pulse=%.0f)...", j_pulse_torque);
					applySafeTorque(0); Delay(300);
					g_tmc4671_cogging_debug.accel_test_current = 0.0f;

					// Apply anti-cog FF from PID-DFT table
					float j_start_pos = getFilteredPosition();
					uint32_t j_start_us = micros();
					const uint32_t J_PULSE_US = 150000;
					uint32_t j_end_target = j_start_us + J_PULSE_US;
					float j_prev_p = j_start_pos;

					// Apply constant torque + cogging FF for precise pulse
					while (((micros() - j_end_target) & 0x80000000) && !emergency) {
						float jp = getFilteredPosition();
						float j_vel = getWrappedError(j_prev_p, jp) / 0.15f; // turns/s over ~150ms window
						j_prev_p = jp;
						g_tmc4671_cogging_debug.accel_vel_now = 60.0f * fabs(j_vel);
						g_tmc4671_cogging_debug.accel_vel_rpm = 60.0f * fabs(j_vel);
						g_tmc4671_cogging_debug.accel_vel_turns_s = j_vel;
						float j_cog = 0.0f;
						{ float jr = jp * 2.0f * PI;
						  for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++)
							if (active_tbl[h].amplitude > 0.0f)
								j_cog += active_tbl[h].amplitude * arm_sin_f32(jr * active_tbl[h].order + active_tbl[h].phase); }
						float j_torque = j_pulse_torque + (this->cogging_scale * j_cog);
						j_torque = clip<float,float>(j_torque, -max_test_torque, max_test_torque);
						applySafeTorque(j_torque);
						g_tmc4671_cogging_debug.accel_test_current = j_pulse_torque;  // base, not FF-compensated
						refreshWatchdog();
					}
					uint32_t j_end_us = micros();
					float j_end_pos = getFilteredPosition();
					applySafeTorque(0);
					g_tmc4671_cogging_debug.accel_test_current = 0.0f;

					float j_dt = (float)(j_end_us - j_start_us) / 1000000.0f;
					float j_dpos = fabs(getWrappedError(j_start_pos, j_end_pos));
					if (j_dpos > 0.0005f) {
						float j_dpos_rad = j_dpos * 2.0f * PI;
						accel_J = ((j_pulse_torque * j_dt * j_dt) / (2.0f * j_dpos_rad)) * 100.0f;
						g_tmc4671_cogging_debug.accel_J = accel_J;
						broadcastCalibLog(0, "Accel DFT: J_with_FF=%.2f (dt=%.3fs, dpos=%.4f turns, pulse=%.0f)",
							accel_J, j_dt, j_dpos, j_pulse_torque);
					} else {
						broadcastCalibLog(0, "Accel DFT: J pulse failed (dpos=%.4f), using SysId J=%.2f", j_dpos, J);
					}
					Delay(500);
					} // J pulse body
				} // a_iter == 0 (J pulse)

				struct { float mag; float phase; } av_cw[COGGING_CALIB_DFT_HARMONICS];
				struct { float mag; float phase; } av_ccw[COGGING_CALIB_DFT_HARMONICS];
				memset(av_cw, 0, sizeof(av_cw)); memset(av_ccw, 0, sizeof(av_ccw));
				float ao_cw = 0.0f, ao_ccw = 0.0f;

				g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_Sweep;

				for (int8_t pd : {1, -1}) {
					// No retry cap: dmax climbs until the motor breaks free of the worst
					// cogging detent. The only ceiling is DMAX_CAP (max_test_torque*0.7).
					const float SWEEP_DMAX_CAP = max_test_torque * 0.7f;
					uint16_t stall_retries = 0;
					bool sweep_ok = false;

					while (!sweep_ok && dmax_torque < SWEEP_DMAX_CAP && !emergency && hasPower()) {
						applySafeTorque(0); g_tmc4671_cogging_debug.accel_test_current = 0.0f;
						{ uint32_t s0 = HAL_GetTick(); float sp0 = getFilteredPosition(); float cdist_rest = 0.0f;
						  while (HAL_GetTick() - s0 < 1000 && !emergency) { float scp = getFilteredPosition(); cdist_rest += fabs(getWrappedError(sp0, scp)); sp0 = scp; g_tmc4671_cogging_debug.accel_vel_now = 60.0f * cdist_rest / ((float)(HAL_GetTick() - s0 + 1) / 1000.0f); Delay(100); refreshWatchdog(); } }
					float restart_t = dmax_torque * 1.5f * (float)pd;
					float sustain_t = dmin_torque * (float)pd;
						const char* dir = (pd==1)?"CW":"CCW";
						if (stall_retries == 0)
							broadcastCalibLog(0, "Accel DFT: %s sweep dmax=%.0f dmin=%.0f", dir, dmax_torque, dmin_torque);
						else
							broadcastCalibLog(0, "Accel DFT: %s retry %u dmax=%.0f dmin=%.0f", dir, stall_retries, dmax_torque, dmin_torque);
					// Spin-up: continuously apply (breakaway + FF) for 800ms.
					// ffWait applies calib_ff_base_torque + cogging FF every 5ms,
					// tracking position so the detent is canceled as the rotor moves.
					// A one-shot applyTorqueFF() leaves torque stale during the wait.
					calib_ff_base_torque = restart_t;
					ffWait(800);
			// Drop to dmin for sustained slow rotation
			calib_ff_base_torque = sustain_t;

				memset(iq_bin_sum, 0, COGGING_DFT_BIN_COUNT * sizeof(float));
				memset(iq_bin_count, 0, COGGING_DFT_BIN_COUNT * sizeof(uint16_t));

				uint32_t astart = HAL_GetTick();
				uint32_t ap_us = getActualCalibPeriod(TIM_TMC_ARR);
				uint32_t ant = micros(); float adt = (float)ap_us / 1000000.0f;
				uint32_t aec = 0, adc = 0; float adist = 0.0f;
				const uint32_t AWM = 1500, ASTALL = 3000, AREV = 20000;

				// Per-sweep velocity-delta state (resets each direction sweep).
				float prev_ap_accel = 0.0f; bool prev_ap_valid_accel = false;
				float prev_av = 0.0f; bool prev_av_valid = false;

				startCalibTimers(TIM_TMC_ARR);
				while (HAL_GetTick() - astart < AREV && !emergency && hasPower()) {
					ant += ap_us;
					float ap = getFilteredPosition();
					// Velocity from position delta — works for all encoder types
					float av = 0.0f;
					if (prev_ap_valid_accel) {
						av = getWrappedError(prev_ap_accel, ap) / adt;
					} else { prev_ap_valid_accel = true; }
					prev_ap_accel = ap;
					g_tmc4671_cogging_debug.accel_vel_now = 60.0f * av;
					g_tmc4671_cogging_debug.accel_vel_rpm = 60.0f * fabsf(av);
					g_tmc4671_cogging_debug.accel_vel_turns_s = av;
					// Acceleration: change in velocity / delta-t between sweeps.
					{
						float accel = 0.0f;
						if (prev_av_valid) {
							accel = (av - prev_av) / adt;
						}
						prev_av = av; prev_av_valid = true;
						g_tmc4671_cogging_debug.accel_accel_rad = accel * 2.0f * PI;
					}

					// FF from existing PID-DFT table
					float acog = 0.0f;
					{ float ar = ap * 2.0f * PI;
					  for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++)
						if (active_tbl[h].amplitude > 0.0f)
							acog += active_tbl[h].amplitude * arm_sin_f32(ar * active_tbl[h].order + active_tbl[h].phase); }
					float at = sustain_t + (this->cogging_scale * acog);
					at = clip<float,float>(at, -max_test_torque, max_test_torque);
					applySafeTorque(at);
					g_tmc4671_cogging_debug.accel_test_current = at;

					if (HAL_GetTick() - astart > AWM && adist < 1.0f) {
						adist += fabs(av * adt);
						if (adc % 2 == 0) {
							uint32_t ab = (uint32_t)(ap * (float)COGGING_DFT_BIN_COUNT);
							if (ab >= COGGING_DFT_BIN_COUNT) ab = COGGING_DFT_BIN_COUNT - 1;
							iq_bin_sum[ab] += av; if (iq_bin_count[ab] != 0xFFFF) iq_bin_count[ab]++;
						} adc++;
					}
					// Stall check: increase dmax ONLY (dmin stays at its calibrated value).
					// A stall is a breakaway failure — the worst cogging detent needs more
					// torque than dmax currently provides. The sustain (dmin) is unaffected.
					if (HAL_GetTick() - astart > AWM + ASTALL && adist < 0.05f) {
						stopCalibTimers(); applySafeTorque(0);
						dmax_torque += 1.0f;
						if (dmax_torque > SWEEP_DMAX_CAP) dmax_torque = SWEEP_DMAX_CAP;
						g_tmc4671_cogging_debug.accel_dmax = dmax_torque;
						goto sweep_retry;
					}
					aec++;
					refreshWatchdog();
#ifdef TIM_CALIBRATION
					if (this->calibTimer != nullptr) { this->WaitForNotification(); } else
#endif
					{ while ((micros() - ant) & 0x80000000) { } }
				}
				stopCalibTimers(); applySafeTorque(0);
				// If we collected enough data, sweep is OK
				sweep_ok = (adist >= 0.05f);
				if (!sweep_ok) {
					dmax_torque += 1.0f;   // dmax only — sustain torque unchanged
					if (dmax_torque > SWEEP_DMAX_CAP) dmax_torque = SWEEP_DMAX_CAP;
					g_tmc4671_cogging_debug.accel_dmax = dmax_torque;
				}
				sweep_retry:
				stall_retries++;
				} // while retry loop
				if (!sweep_ok) {
					broadcastCalibLog(0, "Accel DFT: %s sweep failed after %u retries", (pd==1)?"CW":"CCW", stall_retries);
					continue; // skip DFT extraction for this direction
				}

					// Extract velocity harmonics via DFT on binned means
					float an = 2.0f / (float)COGGING_DFT_BIN_COUNT;
					for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
						float re=0, im=0, ss, sc, s1, c1;
						arm_sin_cos_f32((float)k*180.0f/(float)COGGING_DFT_BIN_COUNT, &ss, &sc);
						arm_sin_cos_f32((float)k*360.0f/(float)COGGING_DFT_BIN_COUNT, &s1, &c1);
						float cs=ss, cc=sc;
						for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++) {
							if (iq_bin_count[b] > 0) { float m = iq_bin_sum[b]/(float)iq_bin_count[b]; re+=m*cc; im+=m*cs; }
							float nc = cc*c1 - cs*s1, ns = cc*s1 + cs*c1; cc=nc; cs=ns;
						}
						if (pd == 1) { av_cw[k].mag = sqrtf(re*re+im*im)*an; av_cw[k].phase = atan2f(re, im); }
						else         { av_ccw[k].mag = sqrtf(re*re+im*im)*an; av_ccw[k].phase = atan2f(re, im); }
					}
					// Avg speed from DC bin
					{ float sv=0; uint32_t n=0;
					  for (uint32_t b=0; b<COGGING_DFT_BIN_COUNT; b++) if (iq_bin_count[b]>0) { sv+=iq_bin_sum[b]/(float)iq_bin_count[b]; n++; }
					  if (n>0) { if (pd==1) ao_cw = sv/(float)n*2.0f*PI; else ao_ccw = sv/(float)n*2.0f*PI; } }
				}

				// Convert velocity → torque: A_cog = J·ω·k·A_v, φ_cog = φ_v + π/2
				float aom = (ao_cw + ao_ccw) * 0.5f; if (aom < 0.01f) aom = 0.01f;
				g_tmc4671_cogging_debug.accel_omega = aom;
				g_tmc4671_cogging_debug.accel_debug_phase = (uint32_t)TMC4671CoggingDebugPhase::AccelDFT_Convert;

				// J validity check: SysId measures J = physical_J * 100.
				// Typical motors: physical J 0.0005–0.05 → code J 0.05–5.
				// If SysId failed (cogging interference), J may be near zero.
				float J_raw = accel_J;
				if (J_raw < 0.01f) {
					broadcastCalibLog(0, "Accel DFT: WARNING — accel_J=%.2f near zero. Trying SysId J=%.2f as fallback.", J_raw, J);
					J_raw = (J > 0.01f) ? J : 0.1f;
				}
				float Jc = (J_raw > 0.01f) ? (J_raw / 100.0f) : 0.001f;
				broadcastCalibLog(0, "Accel DFT: ω=%.1f rad/s, J_raw=%.2f, J_phys=%.6f, converting...",
					aom, J_raw, Jc);
				for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
					// Convert velocity (turns/s) → torque: T = J·α = J · (A_v·2π) · k · ω
					float s = Jc * aom * (float)k * 2.0f * PI;
					av_cw[k].mag *= s; av_cw[k].phase += PI/2.0f;
					av_ccw[k].mag *= s; av_ccw[k].phase += PI/2.0f;
					while (av_cw[k].phase > PI)  av_cw[k].phase -= 2.0f*PI;
					while (av_cw[k].phase < -PI) av_cw[k].phase += 2.0f*PI;
					while (av_ccw[k].phase > PI)  av_ccw[k].phase -= 2.0f*PI;
					while (av_ccw[k].phase < -PI) av_ccw[k].phase += 2.0f*PI;
				}

				// CW+CCW combination (complex-vector phase averaging)
				struct { float m; uint16_t o; float p; } ab[COGGING_HARMONICS_COUNT]; memset(ab,0,sizeof(ab));
				for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
					float am = (av_cw[k].mag + av_ccw[k].mag)/2.0f;
					float ar = (cosf(av_cw[k].phase)+cosf(av_ccw[k].phase))/2.0f;
					float ai = (sinf(av_cw[k].phase)+sinf(av_ccw[k].phase))/2.0f;
					for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						if (am > ab[n].m) {
							for (int s = COGGING_HARMONICS_COUNT-1; s > n; s--) ab[s]=ab[s-1];
							ab[n].m=am; ab[n].o=(uint16_t)k; ab[n].p=atan2f(ai,ar); break;
						}
					}
				}

				// Merge accel DFT harmonics into existing PID-DFT table.
				// Don't wipe the table — the PID-DFT results have better SNR for many
				// harmonics. For each order: keep the larger amplitude.
				for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
					if (ab[n].m <= 0.0f) continue;
					// Find existing slot for this order
					int slot = -1, empty = -1;
					for (int i = 0; i < COGGING_HARMONICS_COUNT; i++) {
						if (active_tbl[i].order == ab[n].o && active_tbl[i].amplitude > 0.0f) { slot = i; break; }
						if (empty < 0 && active_tbl[i].amplitude <= 0.0f) empty = i;
					}
					if (slot >= 0) {
						// Same order exists: keep the larger amplitude
						if (ab[n].m > active_tbl[slot].amplitude) {
							active_tbl[slot].amplitude = ab[n].m;
							active_tbl[slot].phase = ab[n].p;
						}
					} else if (empty >= 0) {
						// New order, insert into empty slot
						active_tbl[empty].order = ab[n].o;
						active_tbl[empty].amplitude = ab[n].m;
						active_tbl[empty].phase = ab[n].p;
					} else {
						// Table full: replace the weakest harmonic if accel DFT is stronger
						float wmin = 1e9f; int wi = 0;
						for (int i = 0; i < COGGING_HARMONICS_COUNT; i++) {
							if (active_tbl[i].amplitude < wmin) { wmin = active_tbl[i].amplitude; wi = i; }
						}
						if (ab[n].m > wmin) {
							active_tbl[wi].order = ab[n].o;
							active_tbl[wi].amplitude = ab[n].m;
							active_tbl[wi].phase = ab[n].p;
						}
					}
				}

				// Merge CW/CCW stores: only populate empty slots, don't overwrite PID-DFT data.
				// PID-DFT verification passes populated these with CW/CCW harmonic data.
				if (!cwccw_data_valid) {
					struct Top { float m; uint16_t o; float p; };
					Top cw_top[COGGING_HARMONICS_COUNT]; memset(cw_top,0,sizeof(cw_top));
					Top ccw_top[COGGING_HARMONICS_COUNT]; memset(ccw_top,0,sizeof(ccw_top));
					for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
						{ float m = av_cw[k].mag; float p = av_cw[k].phase;
						  for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (m > cw_top[n].m) {
								for (int t = COGGING_HARMONICS_COUNT-1; t > n; t--) cw_top[t]=cw_top[t-1];
								cw_top[n].m=m; cw_top[n].o=(uint16_t)k; cw_top[n].p=p; break;
							} } }
						{ float m = av_ccw[k].mag; float p = av_ccw[k].phase;
						  for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
							if (m > ccw_top[n].m) {
								for (int t = COGGING_HARMONICS_COUNT-1; t > n; t--) ccw_top[t]=ccw_top[t-1];
								ccw_top[n].m=m; ccw_top[n].o=(uint16_t)k; ccw_top[n].p=p; break;
							} } }
					}
					for (int n = 0; n < COGGING_HARMONICS_COUNT; n++) {
						if (cw_top[n].m > 0.0f) { cw_store[n].order=cw_top[n].o; cw_store[n].amplitude=cw_top[n].m; cw_store[n].phase=cw_top[n].p; }
						if (ccw_top[n].m > 0.0f) { ccw_store[n].order=ccw_top[n].o; ccw_store[n].amplitude=ccw_top[n].m; ccw_store[n].phase=ccw_top[n].p; }
					}
					cwccw_data_valid = true;
				}

				// Broadcast
				{ uint8_t pa = (rpm_profile >= 2) ? 2 : rpm_profile;
				  std::string hs = "profile:"; hs += std::to_string(rpm_profile+1); hs += ":";
				  bool any=false;
				  for (uint8_t i=0; i<COGGING_HARMONICS_COUNT; i++) {
					if (active_tbl[i].amplitude > 0.0f || active_tbl[i].order > 0) {
						if (any) hs += ",";
						any = true;
						hs += std::to_string(active_tbl[i].order) + ":";
						hs += std::to_string((int16_t)active_tbl[i].amplitude) + ":";
						hs += std::to_string((int16_t)(active_tbl[i].phase * 1000.0f));
					} }
				  if (!any) hs += "0:0:0";
				  CommandHandler::broadcastCommandReply(CommandReply(hs, pa), (uint32_t)TMC4671_commands::coggingHarmonics, CMDtype::get);
				  broadcastCalibLog(0, "Accel DFT: table replaced and broadcast"); }
				} // accel iter loop
			}
#endif

			} // end DFT iteration loop
			} // end multi-RPM profile loop

		// 3. SAVE & FINALIZE
		if (!emergency && any_profile_succeeded) {
			broadcastCalibLog(0, "Cogging calibration successful");
				refreshWatchdog();
				saveCoggingTable();
				this->bins_data_valid = true;
				// Precompute the combined bin LUT (mode 1) once, here, so runtime
				// turn() doesn't pay for the average on every call.
				for (uint32_t b = 0; b < COGGING_DFT_BIN_COUNT; b++)
					this->cogging_bins_combined[b] = (this->cw_bins[b] + this->ccw_bins[b]) * 0.5f;

#if defined(COGGING_PHASE_SHIFT_CAL) || defined(COGGING_PHASE_SHIFT_MULTIRPM)
			// Phase-shift method: mark scale and phase-advance curves as valid.
			// The curve values were populated per-profile above; saveFlash()
			// (called in the scale calibration block below) will persist them.
			// Independent of blending — both can coexist.
#ifndef COGGING_DISABLE_SCALE_CURVE
			scale_curve_valid = true;
#endif
#ifndef COGGING_DISABLE_SCALE_CURVE
			phase_adv_curve_valid = true;
#endif
			// Fill all 24 RPM breakpoints using linear interpolation between the
			// sparse calibrated points so the configurator shows a smooth curve.
			{
				uint8_t calib_idx[SCALE_CURVE_POINTS];
				uint8_t calib_n = 0;
				for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
#ifdef COGGING_DISABLE_SCALE_CURVE
					if (phase_advance_curve_values[i] != 0.0f || i == 0)
#else
					if (scale_curve_values[i] > 0.0f)
#endif
						calib_idx[calib_n++] = i;
				}
				if (calib_n >= 2) {
					uint8_t lo = 0;
					for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
						while (lo + 1 < calib_n && calib_idx[lo + 1] <= i) lo++;
						uint8_t hi = (lo + 1 < calib_n) ? lo + 1 : lo;
						if (hi == lo && calib_n >= 2) {
							uint8_t a = calib_idx[calib_n - 2], b = calib_idx[calib_n - 1];
							float drpm_i = scale_curve_rpm_points[i] - scale_curve_rpm_points[b];
							float denom = scale_curve_rpm_points[b] - scale_curve_rpm_points[a];
							if (denom > 0.01f) {
#ifndef COGGING_DISABLE_SCALE_CURVE
								float ss = (scale_curve_values[b] - scale_curve_values[a]) / denom;
								scale_curve_values[i] = scale_curve_values[b] + ss * drpm_i;
#endif
								float ps = (phase_advance_curve_values[b] - phase_advance_curve_values[a]) / denom;
								phase_advance_curve_values[i] = phase_advance_curve_values[b] + ps * drpm_i;
							}
						} else {
							float denom = scale_curve_rpm_points[calib_idx[hi]] - scale_curve_rpm_points[calib_idx[lo]];
							if (denom > 0.01f) {
								float t = (scale_curve_rpm_points[i] - scale_curve_rpm_points[calib_idx[lo]]) / denom;
#ifndef COGGING_DISABLE_SCALE_CURVE
								scale_curve_values[i] = scale_curve_values[calib_idx[lo]] + t * (scale_curve_values[calib_idx[hi]] - scale_curve_values[calib_idx[lo]]);
#endif
								phase_advance_curve_values[i] = phase_advance_curve_values[calib_idx[lo]] + t * (phase_advance_curve_values[calib_idx[hi]] - phase_advance_curve_values[calib_idx[lo]]);
							}
						}
					}
#ifdef COGGING_DISABLE_SCALE_CURVE
					for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) scale_curve_values[i] = 1.0f;
#else
					scale_curve_values[0] = 1.0f;
#endif
					phase_advance_curve_values[0] = 0.0f;
				}
			}
			broadcastCalibLog(0, "Phase-shift curves stored (%u RPM profiles)",
				scale_curve_count);
#endif

#ifndef COGGING_DISABLE_BLEND
			// Persist the RPM#2/RPM#3 maps and stamp the blend anchors from the
			// calibrated profile speeds so the runtime blend matches this run.
			// Independent of phase-shift — both can coexist.
			if (this->cogging_calib_count > 0 && this->cogging_calib_rpm[0] > 0.0f)
				this->blend_rpm1 = this->cogging_calib_rpm[0];
			if (this->rpm2_table_valid) {
				if (this->cogging_calib_count > 1 && this->cogging_calib_rpm[1] > 0.0f)
					this->blend_rpm2 = this->cogging_calib_rpm[1];
				saveCoggingTableRpm2();
			}
			if (this->rpm3_table_valid) {
				if (this->cogging_calib_count > 2 && this->cogging_calib_rpm[2] > 0.0f)
					this->blend_rpm3 = this->cogging_calib_rpm[2];
				saveCoggingTableRpm3();
			}
			broadcastCalibLog(0, "Maps stored (blend %.0f/%.0f/%.0f RPM, rpm2:%s rpm3:%s)",
				this->blend_rpm1, this->blend_rpm2, this->blend_rpm3,
				this->rpm2_table_valid ? "yes" : "no",
				this->rpm3_table_valid ? "yes" : "no");
#endif

				// --- 3.5 SCALE CALIBRATION (Position Error Grid Search) ---
				int main_h = -1;
				float max_amp = 0;
				for(int n=0; n<COGGING_HARMONICS_COUNT; n++){
					if(cogging_harmonics[n].amplitude > max_amp){
						max_amp = cogging_harmonics[n].amplitude;
						main_h = n;
					}
				}

				if (main_h != -1) {
					this->cogging_scale = 1.0f;
					this->cogging_enabled = true;
					// Persist anti-cogging metadata (enabled flag + scale) to EEPROM
					// The harmonic tables were already saved to the flash sector above.
					saveFlash();
				}

				calib_rpm = this->cogging_calib_rpm[this->cogging_calib_count - 1];
				if (calib_rpm < 3.0f) calib_rpm = 20.0f;
			pid_soft.Ki = 0.0f;
				pid_soft.Kd = 0.0f;

			// --- PRESERVE CURRENT ANGLE ---
			// Calibration may have left the motor many turns from zero.
			// The Axis out-of-bounds check (abs(scaledEnc) > 0xffff) uses
			// getEncoder()->getPos_f() to compute the angle.  Instead of
			// physically dragging the motor back, we read the current
			// position modulo one turn (= the angle we are at right now)
			// and set that as the new multi-turn position via the same
			// setPos() path the Axis "pos" command uses.
			{
				Encoder* enc = getEncoder();  // same encoder the Axis reads
				float frac_pos = getFilteredPosition(); // 0..1 turns
				int32_t cpr = enc->getCpr();
				if (cpr > 0) {
					int32_t new_pos = (int32_t)(frac_pos * (float)cpr);
					enc->setPos(new_pos);
					accelPosWasReset = true;
					broadcastCalibLog(0, "Calibration complete. Angle preserved: %.1f deg (pos=%d).",
						(double)(frac_pos * 360.0), (int)new_pos);
				} else {
					// CPR unknown — fall back to old setTmcPos(0)
					setTmcPos(0);
					accelPosWasReset = true;
					broadcastCalibLog(0, "Calibration complete. Position reset to 0 (CPR=0).");
				}
			}
			goto cleanup;
		}
	}

cleanup:
	// Stop background cogging FF before restoring hardware state.
	this->calib_ff_active = false;
	this->calib_ff_base_torque = 0.0f;
	this->active_tbl = nullptr;
	applySafeTorque(0);

	dbg.phase = static_cast<uint32_t>((errorMessage != nullptr || emergency) ? TMC4671CoggingDebugPhase::Aborted : TMC4671CoggingDebugPhase::Completed);
	if (errorMessage) {
		broadcastCalibLog(1, errorMessage);
	} else {
		broadcastCalibLog(1, "Cogging detection finished");
	}
	
	setTargetVelocity(0); // Ensure motor stops

	if(iq_acc_cos) vPortFree(iq_acc_cos);
	if(iq_acc_sin) vPortFree(iq_acc_sin);
	if(iq_bin_sum) vPortFree(iq_bin_sum);
	if(iq_bin_count) vPortFree(iq_bin_count);
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
	if(id_acc_cos) vPortFree(id_acc_cos);
	if(id_acc_sin) vPortFree(id_acc_sin);
#endif

	// Restore hardware state (match FullCalibration logic)
	setPids(prevPids);
	curFilters.flux.params.enable = true;
	setBiquadFlux(curFilters.flux);
	setMotionMode(prevCalibMode, true);

	allowStateChange = true;
	// Transition to a stable state; skip EncoderInit if position was reset
	// (encoderInit would undo setTmcPos by calling setTmcPos(getPosAbs()-offset))
	if (errorMessage) {
		changeState(hasPower() ? TMC_ControlState::Running : TMC_ControlState::waitPower);
	} else if (accelPosWasReset) {
		changeState(hasPower() ? TMC_ControlState::Running : TMC_ControlState::waitPower);
	} else {
		changeState(TMC_ControlState::EncoderInit);
	}
}
#endif

/**
 * Iterative tuning function for tuning the torque mode PI values (Asynchronous version)
 */
void TMC4671::handleStatePidAutoTune() {
	if (!hasPower() || emergency) {
		pidTuneState = PidTuneState::Init;
		this->postPowerState = TMC_ControlState::Pidautotune;
		changeState(TMC_ControlState::waitPower);
		return;
	}

	// 1. Initial Setup
	allowStateChange = false;
	curFilters.flux.params.enable = false;
	setBiquadFlux(curFilters.flux);
	lastPidTunePhiE = getPhiEtype();
	lastPidTuneMode = getMotionMode();
	setPhiE_ext(getPhiE());
	setPhiEtype(PhiE::ext); // Fixed phase for testing flux response
	setMotionMode(MotionMode::torque, true);
	setFluxTorque(0, 0);
	
	int16_t targetflux = std::min<int16_t>(this->curLimits.pid_torque_flux, bangInitPower);
	int16_t targetflux_p = targetflux * 0.75;
	
	tuneFluxI = 0;
	tuneFluxP = 100;
	writeReg(0x54, tuneFluxI | (tuneFluxP << 16));
	
	// 2. Atomic P-Gain Tuning (Ramp)
	while (tuneFluxP < 20000 && !emergency && hasPower()) {
		setFluxTorque(targetflux, 0);
		Delay(50); // Stabilization delay
		int32_t flux = getActualFlux();
		if (flux > targetflux_p) break;
		
		// Adaptive steps
		tuneFluxP += (flux > targetflux * 0.5) ? 50 : 250; 
		writeReg(0x54, tuneFluxI | (tuneFluxP << 16));
	}
	
	// 3. Atomic I-Gain Tuning (Overshoot detection)
	tuneFluxI = 100;
	while (tuneFluxI < 20000 && !emergency && hasPower()) {
		setFluxTorque(0, 0);
		Delay(100); // Settle current to zero
		
		tuneMeasurePeak = 0;
		writeReg(0x54, tuneFluxI | (tuneFluxP << 16));
		setFluxTorque(targetflux, 0); // Pulse
		
		// Ultra-fast sampling loop for peak detection (Atomic)
		uint32_t start = HAL_GetTick();
		while(HAL_GetTick() - start < 50) {
			tuneMeasurePeak = std::max<int32_t>(tuneMeasurePeak, getActualFlux());
		}
		
		// Check for overshoot target (default 4%)
		if (tuneMeasurePeak > (targetflux + (targetflux * TMC4671_ITUNE_CUTOFF))) {
			if (tuneMeasurePeak > targetflux) tuneFluxI -= (tuneFluxI > 20) ? 20 : 0; // Back off
			break;
		}
		
		tuneFluxI += (tuneMeasurePeak < targetflux * 0.95) ? 150 : 25;
	}

	// 4. Finalization & Cleanup
	setFluxTorque(0, 0);
	curFilters.flux.params.enable = true;
	setBiquadFlux(curFilters.flux);
	
	if (tuneFluxP < 20000 && tuneFluxI < 20000 && hasPower() && !emergency) {
		pidTuneNewPids = curPids;
		pidTuneNewPids.fluxP = tuneFluxP;
		pidTuneNewPids.torqueP = tuneFluxP;
		pidTuneNewPids.fluxI = tuneFluxI;
		pidTuneNewPids.torqueI = tuneFluxI;
		setPids(pidTuneNewPids);
		CommandHandler::broadcastCommandReply(CommandReply("PID Autotune successful", 1), (uint32_t)TMC4671_commands::pidautotune, CMDtype::get);
	} else {
		CommandHandler::broadcastCommandReply(CommandReply("PID Autotune failed or interrupted", 0), (uint32_t)TMC4671_commands::pidautotune, CMDtype::get);
	}

	setPhiEtype(lastPidTunePhiE);
	setMotionMode(lastPidTuneMode, true);
	allowStateChange = true;
	pidTuneState = PidTuneState::Init;
	changeState(laststate, false);
}

#endif
