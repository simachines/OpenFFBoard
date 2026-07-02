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
}


TMC4671::~TMC4671() {
	enablePin.reset();
	//recordSpiAddrUsed(0);
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
	setPwm(0,conf.pwmcnt,conf.bbmL,conf.bbmH); // Set FOC @ 25khz but turn off pwm for now
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

	int32_t flux = 0;
	int32_t totalPower = power;

	// Anticogging is enabled in firmware
#ifdef COGGING_TABLE_FLASH_START_ADDRESS
	if (cogging_enabled && !this->isCalibrationInProgress()) {
		float pos_f = this->getFilteredPosition();

		// Measure RPM from position delta (runs in turn() at firmware speed)
		uint32_t now = micros();
		float signed_rpm = 0.0f;
		if (last_vel_tick > 0) {
			float dt_sec = (float)(now - last_vel_tick) / 1000000.0f;
			float delta = pos_f - prev_filtered_pos;
			if (delta > 0.5f) delta -= 1.0f;
			if (delta < -0.5f) delta += 1.0f;
			signed_rpm = (delta / dt_sec) * 60.0f;
			measured_rpm = fabsf(signed_rpm);
			measured_rpm_signed = signed_rpm;
		}
		prev_filtered_pos = pos_f;
		last_vel_tick = now;

		// Velocity-based phase advance: shift the lookup position forward in the
		// direction of motion to compensate for high-speed lag of the cogging pattern.
		if (phase_adv_curve_valid) {
			float adv_deg = interpolatePhaseAdvance(measured_rpm);
			float dir = (signed_rpm >= 0.0f) ? 1.0f : -1.0f;
			pos_f += dir * adv_deg / 360.0f;
			// Wrap back into [0,1)
			pos_f = pos_f - floorf(pos_f);
		}

		// Fourier series compensation with per-RPM harmonic blending.
		// Three tables were calibrated at different RPMs; blend between them
		// based on measured_rpm: below blend_rpm1 uses only cogging_harmonics,
		// between blend_rpm1..blend_rpm2 uses blend of cogging_harmonics + cogging_harmonics_rpm2,
		// above blend_rpm2 uses blend of cogging_harmonics_rpm2 + cogging_harmonics_rpm3.
		// Fourier series compensation.
#ifdef COGGING_DISABLE_BLEND
		// Blending disabled: use the base cogging table (profile 0) directly.
		// Phase advance (above) may still be active independently.
		Harmonic* blended = this->cogging_harmonics;
#else
		// Per-RPM harmonic blending: three tables were calibrated at different
		// RPMs; blend between them based on measured_rpm.
		Harmonic blended[COGGING_HARMONICS_COUNT];
		this->blendHarmonicTables(measured_rpm, blended);
#endif

		float compensation = 0;
		float angle_rad = pos_f * 2.0f * PI;

		// Track the dominant harmonic (largest amplitude) for waveshaping.
		float dom_amp = 0.0f, dom_order = 1.0f, dom_phase = 0.0f;
		for (uint8_t i = 0; i < COGGING_HARMONICS_COUNT; i++) {
			if (blended[i].amplitude > 0.0f) {
				compensation += blended[i].amplitude * arm_sin_f32(angle_rad * blended[i].order + blended[i].phase);
				if (blended[i].amplitude > dom_amp) {
					dom_amp = blended[i].amplitude;
					dom_order = (float)blended[i].order;
					dom_phase = blended[i].phase;
				}
			}
		}

		// Cogging waveshaping: subtract/add a harmonic of the DOMINANT cogging
		// order to reshape the peak/trough profile ("thin peaks / steep slopes"),
		// since the raw Fourier sum can sit slightly off the physical tooth geometry.
		//   shaped = compensation - h3_shaping * dom_amp * sin(mult*(dom_order*theta + dom_phase) + phase_trim)
		// mult=3 and h3_shaping>0 thins the peaks; h3_shaping<0 flattens them.
		if (h3_shaping != 0.0f && dom_amp > 0.0f) {
			float shaped_arg = this->h3_mult * (dom_order * angle_rad + dom_phase) + this->h3_phase_trim;
			compensation -= this->h3_shaping * dom_amp * arm_sin_f32(shaped_arg);
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

int32_t TMC4671::getPos(){

	int32_t pos = (int32_t)readReg(0x6B);
	this->cached_pos = pos;
	return pos;
}

int32_t TMC4671::getPosAbs(){
	int16_t pos;
	if(this->conf.motconf.enctype == EncoderType_TMC::abn){
		pos = (int16_t)readReg(0x2A) & 0xffff; // read phiM
	}else if(this->conf.motconf.enctype == EncoderType_TMC::hall){
		pos = (int16_t)readReg(0x3A); // read phiM
	}else if(this->conf.motconf.enctype == EncoderType_TMC::sincos || this->conf.motconf.enctype == EncoderType_TMC::uvw){
		pos = (int16_t)readReg(0x46) & 0xffff; // read phiM
	}else{
		pos = getPos(); // read phiM
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
	maxcnt = clip(maxcnt, 255, 4095);
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
			// Append scale then position
			replyStr += ":" + std::to_string((int16_t)(this->last_cogging_scale * 100.0f));
			replyStr += ":" + std::to_string((int32_t)(this->getFilteredPosition() * 10000.0f));
			replyStr += ":" + std::to_string((int16_t)(this->measured_rpm));
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
	Encoder* activeEnc = usingExternalEncoder() ? drvEncoder.get() : static_cast<Encoder*>(this);
	return activeEnc->getPos_f();
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
	
	// Integer modulo BEFORE float conversion guarantees zero precision loss
	// By ignoring full turns, we maintain 100% of the encoder's fractional resolution forever.
	int32_t remainder = activeEnc->getPos() % cpr;
	if (remainder < 0) remainder += cpr; // Mathematical modulo (always positive)
	
	return (float)remainder / (float)cpr;
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
	float* iq_acc_cos = nullptr;
	float* iq_acc_sin = nullptr;
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
	const uint32_t GAIN_SWEEP_SETTLE_MS = 250; // (sweep disabled — preserved)

	arm_pid_instance_f32 pid_soft;
	char dbg_buf[128];
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
			float iq_friction,
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
		
		dbg.positionErrorDeg = error_turns * 360.0f;
		//dbg.iqPid = iq_pid;
		//dbg.iqFriction = iq_friction;
		//dbg.iqInertia = iq_inertia;
		dbg.iqCompensation = iq_compensation;
		dbg.iqCmd = iq_cmd;
		dbg.Appliediq = iq_applied;
		//dbg.currentVelTurns = current_vel_turns;
		//dbg.currentAccelRad = current_accel_rad;
		//dbg.dynamicFriction = dynamic_friction;
	};

	resetDebugWatch();

	broadcastCalibLog(0, "Starting Cogging Calibration: Continuous DFT...");
	
	if (this->getCpr() == 0) { 
		errorMessage = "Abort: CPR is 0"; 
		goto cleanup; 
	}
	
	allowStateChange = false;
	
	// Reset cogging table and scale/phase curves for fresh calibration
	clearCoggingTable();
	scale_curve_valid = false;
	phase_adv_curve_valid = false;
	memset(scale_curve_values, 0, sizeof(scale_curve_values));
	memset(phase_advance_curve_values, 0, sizeof(phase_advance_curve_values));
	// Plateau (index 0): scale=1.0 (full compensation at low RPM), phase=0.0°.
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
	iq_acc_cos = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	iq_acc_sin = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
	id_acc_cos = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	id_acc_sin = (float*)pvPortMalloc(COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#endif
	
	if(!iq_acc_cos || !iq_acc_sin 
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
		|| !id_acc_cos || !id_acc_sin
#endif
	) {
		errorMessage = "Abort: Memory fail (Heap)";
		goto cleanup;
	}

	memset(iq_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
	memset(iq_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
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

			pid_soft.Kp = clip<float,float>(kp, 50.0f, 250000.0f);
			pid_soft.Ki = clip<float,float>(ki, 0.0f, 100000.0f);
			pid_soft.Kd = coggingSpeedD;
			arm_pid_init_f32(&pid_soft, 1);

			float target_pos_turns = getAbsolutePosition();
			uint32_t eval_start = HAL_GetTick();
			uint32_t next_tick = micros();
			uint32_t act_period = getActualCalibPeriod(1000);
			float dt_sec = (float)act_period / 1000000.0f;
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
					float iq_ff = dynamic_friction * 1.0f;
					float iq_cmd = clip<float,float>(iq_pid + iq_ff, -max_test_torque, max_test_torque);

					captureDebug(phase, calib_rpm, target_pos_turns, actual_pos_turns, err, iq_pid, iq_ff, 0.0f, 0.0f, iq_cmd, iq_cmd, 0.0f, 0.0f, J, B, dynamic_friction);
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

			if (coggingSpeedP == 0.0f && coggingSpeedI == 0.0f) {
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
				// Yield thread via timer interrupt notifications to avoid busy-waiting, falling back if timer is disabled
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
			// Scale by 100 to map physical B to the abstract TMC integer units
			// Subtract Coulomb friction (breakout torque) to isolate true viscous damping
			dynamic_friction = b_sum_torque / (float)b_samples;
			B = (dynamic_friction / b_target_vel_rad) * 100.0f;
			B = clip<float, float>(B, 0.0f, 100000.0f); // Ensure B is positive
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
				errorMessage = emergency ? "Abort: IMC validation interrupted" : (!hasPower() ? "Abort: Power lost during IMC validation" : "Abort: IMC validation did not cover all quarters");
				goto cleanup;
			}
			logQuarterErrorStats("IMC", imc_kp, imc_ki, imc_stats);

			// PI sweep disabled — using IMC gains directly to avoid over-driving the motor
			broadcastCalibLog(0, "Using IMC gains directly (sweep disabled)");
			float best_sweep_kp = imc_kp;
			float best_sweep_ki = 0.0f;  // I=0 enforced — auto-tuning is P-only

			pid_soft.Kp = best_sweep_kp;
			pid_soft.Ki = best_sweep_ki;
			pid_soft.Kd = coggingSpeedD;
			// Store IMC Kp into profile 0 so the auto-tuning can use it as baseline.
			// I is forced to zero — the auto-tuning tune is P-only.
			this->cogging_calib_pidP[0] = (uint32_t)best_sweep_kp;
			this->cogging_calib_pidI[0] = 0;
			this->cogging_calib_pidD[0] = 0;
			} else {
				pid_soft.Kp = coggingSpeedP;
				pid_soft.Ki = coggingSpeedI;
				pid_soft.Kd = coggingSpeedD;
				imc_kp = coggingSpeedP;  // manual P as baseline for auto-tuning
				broadcastCalibLog(0, "Manual PID Override -> Kp:%.0f Ki:%.0f", pid_soft.Kp, pid_soft.Ki);
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
				 rpm_profile <= this->cogging_calib_count && !emergency && hasPower();
				 rpm_profile++) {

				bool is_return_tune = (rpm_profile == this->cogging_calib_count);
				if (is_return_tune) {
					calib_rpm = this->cogging_calib_rpm[this->cogging_calib_count - 1];
					if (calib_rpm < 3.0f) calib_rpm = 20.0f;
				} else {
					calib_rpm = this->cogging_calib_rpm[rpm_profile];
					if (calib_rpm <= 0.0f) calib_rpm = 60.0f / (float)COGGING_CALIB_TIME_PER_REV_S;
				}

				const uint8_t MAX_DFT_ITERATIONS = is_return_tune ? 0 : this->cogging_calib_iters[rpm_profile];
#endif
				if (MAX_DFT_ITERATIONS < 1 && !is_return_tune) continue;

				// Recompute acquisition duration so each CW and CCW sweep
				// is exactly 1 revolution, regardless of the RPM target.
				const uint32_t rev_ms = (uint32_t)((60.0f / calib_rpm) * 1500.0f);
				const uint32_t REVOLUTION_TIME_MS = rev_ms + cogging_warmup_ms;

				if (!is_return_tune) {
				broadcastCalibLog(0, "RPM profile %u/%u: target %.1f RPM, %u iterations (%.1f s/rev)",
					rpm_profile + 1, this->cogging_calib_count, calib_rpm, MAX_DFT_ITERATIONS,
					(float)REVOLUTION_TIME_MS / 1000.0f);
				}

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
					uint8_t src = is_return_tune ? (this->cogging_calib_count - 1) : 0;
					pidP = this->cogging_calib_pidP[src];
					pidI = this->cogging_calib_pidI[src];
					pidD = this->cogging_calib_pidD[src];
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
			pid_soft.Kd = this->coggingSpeedD;
			arm_pid_init_f32(&pid_soft, 1);

// --- P-GAIN AUTO-TUNING SEQUENCE ---
			// Run per RPM profile when cogging_calib_autoPid is enabled.
			// Uses trapezoidal velocity sweeps (accel → cruise → decel) to find
			// the optimal proportional gain that minimizes position tracking error
			// without inducing oscillation. I and D are held at zero during tuning.
			if (this->cogging_calib_autoPid) {
				broadcastCalibLog(0, "Auto-tuning Kp for %.1f RPM...", calib_rpm);

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
				static constexpr float KP_TUNE_CEILING = 10000000.0f;
				const char* stop_reason = "limit";

				// Fixed I=0, D=0 during P-only tuning
				pid_soft.Ki = 0.0f;
				pid_soft.Kd = 0.0f;

				uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR);
				float dt_sec = (float)period_us / 1000000.0f;

				// --- KINEMATIC TRAJECTORY SETUP ---
				// Calculate a safe maximum acceleration using the measured inertia (J)
				// J was scaled by 100. Restore physical J: J_phys = J / 100
				// Torque = J * alpha -> alpha_rad = Torque / J_phys
				// We use 25% of max torque for acceleration, leaving 75% headroom for the PID to fight cogging.
				float j_phys = J / 100.0f;
				if (j_phys < 0.001f) j_phys = 0.001f; // Failsafe
				float max_accel_turns_s2 = (max_test_torque * 0.25f) / j_phys / (2.0f * PI);
				if (max_accel_turns_s2 < 1.0f) max_accel_turns_s2 = 1.0f;

				float target_vel_turns = calib_rpm / 60.0f;
				// Distance needed to ramp up: d = v^2 / (2*a)
				float ramp_dist = (target_vel_turns * target_vel_turns) / (2.0f * max_accel_turns_s2);
				if (ramp_dist < 0.25f) ramp_dist = 0.25f;
				float cruise_dist = target_vel_turns * 0.6f;
				if (cruise_dist < 0.35f) cruise_dist = 0.35f;
				float total_dist = (ramp_dist * 2.0f) + cruise_dist;
				// Cap sweep time to ~3s (avoid 40s sweeps at 3 RPM)
				float max_total = target_vel_turns * 3.0f;
				if (total_dist > max_total) total_dist = max_total;
				if (total_dist < 0.5f) total_dist = 0.5f;

				float target_pos_f = getFilteredPosition();

				while (test_kp < KP_TUNE_CEILING && step_count < MAX_TUNE_STEPS && !tuning_done && !emergency && hasPower()) {
					step_count++;
					pid_soft.Kp = test_kp;
					arm_pid_init_f32(&pid_soft, 1);

				// Settle: actively stop the motor before each P-tuner sweep.
				// This is critical between profiles where the motor may still
				// have residual motion from DFT or the previous sweep.
				{
					float hold_pos = getFilteredPosition();
					uint32_t settleStart = HAL_GetTick();
					while (HAL_GetTick() - settleStart < 500 && !emergency && hasPower()) {
						float err = getWrappedError(hold_pos, getFilteredPosition());
						float iq_hold = arm_pid_f32(&pid_soft, err);
						iq_hold = clip<float>(iq_hold, -max_test_torque * 0.3f, max_test_torque * 0.3f);
						applySafeTorque(iq_hold);
						refreshWatchdog();
						Delay(1);
					}
					applySafeTorque(0);
					Delay(50);
					arm_pid_init_f32(&pid_soft, 1);
				}

					float max_err_deg = -999.0f;
					float min_err_deg = 999.0f;
					bool clamp_hit = false;

					float current_vel_turns = 0.0f;
					float dist_traveled = 0.0f;
					uint8_t phase = 0; // 0 = Accel, 1 = Cruise, 2 = Decel

					uint32_t next_tick = micros();
					startCalibTimers(TIM_TMC_ARR);

					while (dist_traveled < total_dist && !emergency && hasPower()) {
						next_tick += period_us;

						// Trapezoidal Velocity Profile Generator
						if (phase == 0) {
							current_vel_turns += max_accel_turns_s2 * dt_sec * test_dir;
							if (fabsf(current_vel_turns) >= target_vel_turns) {
								current_vel_turns = target_vel_turns * test_dir;
								phase = 1; // Reached target velocity, start cruising
							}
						} else if (phase == 1) {
							if (total_dist - dist_traveled <= ramp_dist) {
								phase = 2; // Time to decelerate
							}
						} else if (phase == 2) {
							current_vel_turns -= max_accel_turns_s2 * dt_sec * test_dir;
							// Stop exactly at 0
							if ((test_dir > 0 && current_vel_turns <= 0.0f) || (test_dir < 0 && current_vel_turns >= 0.0f)) {
								current_vel_turns = 0.0f;
								break;
							}
						}

						float step = current_vel_turns * dt_sec;
						target_pos_f += step;
						if (target_pos_f >= 1.0f) target_pos_f -= 1.0f;
						if (target_pos_f < 0.0f) target_pos_f += 1.0f;

						dist_traveled += fabsf(step);

						float actual_pos_f = getFilteredPosition();
						float err = getWrappedError(target_pos_f, actual_pos_f);
						float err_deg = err * 360.0f;

						float iq_pid = arm_pid_f32(&pid_soft, err);
						// Add slight friction feedforward to help tracking without altering tuning dynamics
						float iq_ff = (current_vel_turns > 0.0f) ? dynamic_friction * 0.5f : ((current_vel_turns < 0.0f) ? -dynamic_friction * 0.5f : 0.0f);

						float iq_cmd = clip<float,float>(iq_pid + iq_ff, -max_test_torque, max_test_torque);
						applySafeTorque(iq_cmd);

						// ONLY measure error during the Cruise phase (ignores accel/decel transients)
						if (phase == 1) {
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
					Delay(25); // Brief rest to let rotor settle before reversing

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
					broadcastCalibLog(0, "Selected Kp:%.0f (Ceiling %.0f, Lowest P2P:%.2f\xC2\xB0)", best_kp, KP_TUNE_CEILING, lowest_p2p);
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
			pid_soft.Kd = this->coggingSpeedD;
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
			memset(active_tbl, 0, COGGING_HARMONICS_COUNT * sizeof(Harmonic));
			this->cogging_scale = 1.0f;

			float prev_iter_err = 999.0f; // track best error for degradation detection
			Harmonic prev_harmonics[COGGING_HARMONICS_COUNT]; // backup of best table

			// DFT retry: if the acquisition clamps (Kp too high for this RPM),
			// lower Kp by one step and restart the DFT from scratch.
			static constexpr uint8_t DFT_MAX_CLAMP_RETRIES = 3;
			for (uint8_t dft_retry = 0; dft_retry < DFT_MAX_CLAMP_RETRIES && !emergency && hasPower(); dft_retry++) {
			bool dft_clamped = false;

			for (uint8_t dft_iter = 0; dft_iter < MAX_DFT_ITERATIONS && !emergency && hasPower(); dft_iter++) {
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
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
					memset(id_acc_cos, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
					memset(id_acc_sin, 0, COGGING_CALIB_DFT_HARMONICS * sizeof(float));
#endif
					uint32_t dir_samples = 0;

					// Active settle: hold position to bring the motor to a full stop
					// before reversing direction. Scales with RPM — higher speeds
					// need more time and more braking torque to arrest momentum.
					{
						float hold_pos = getFilteredPosition();
						// Settle duration: at least 1.5s, up to 1 revolution at this RPM
						float rev_s = 60.0f / fabsf(calib_rpm);
						uint32_t settle_ms = (uint32_t)(rev_s * 1500.0f); // ~1.5 rev worth
						if (settle_ms < 1500) settle_ms = 1500;
						if (settle_ms > 5000) settle_ms = 5000;
						// Torque limit scales with RPM: 30% base, up to 70% at high RPM
						float settle_tq_frac = 0.3f + (fabsf(calib_rpm) / 300.0f);
						if (settle_tq_frac > 0.7f) settle_tq_frac = 0.7f;
						float settle_tq = max_test_torque * settle_tq_frac;

						uint32_t settleStart = HAL_GetTick();
						while(HAL_GetTick() - settleStart < settle_ms && !emergency && hasPower()) {
							float err = getWrappedError(hold_pos, getFilteredPosition());
							float iq_hold = arm_pid_f32(&pid_soft, err);
							iq_hold = clip<float>(iq_hold, -settle_tq, settle_tq);
							applySafeTorque(iq_hold);
							refreshWatchdog();
							Delay(1);
						}
						applySafeTorque(0);
						Delay(50);
					}
					// Re-init PID after settle — the position-hold may have
					// accumulated integrator windup and the motor can drift
					// during the 50ms torque-off gap. A fresh PID prevents
					// a torque spike when the sweep starts.
					arm_pid_init_f32(&pid_soft, 1);

					if (!emergency && hasPower()) {
						float target_pos_f = getFilteredPosition();

						calibStartTime = HAL_GetTick();
						uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR); 
						uint32_t next_tick = micros();
						float dt_sec = (float)period_us / 1000000.0f;
						
						float prev_actual_pos_f = getFilteredPosition();
						float prev_vel_turns = 0.0f; // start from rest
						float full_vel_turns = target_rpm / 60.0f;
						float ramp_vel_turns = 0.0f; // ramped velocity
						// Ramp-up time equals warmup: accelerate smoothly from 0 to target
						float ramp_rate = full_vel_turns / (float)cogging_warmup_ms * 1000.0f; // turns/s²
						
						uint32_t enc_decimation_counter = 0;
						// DFT decimation: keep DFT accumulation ≤ ~4 kHz to prevent
						// 32-bit float overflow
						uint32_t dft_decimation_ratio = 4;
						uint32_t dft_decimation_counter = 0;
						float iq_cmd = 0.0f;
						float iq_inertia = 0.0f;
						float iq_pid = 0.0f;
						int32_t actual_iq_raw = 0;
						
						startCalibTimers(TIM_TMC_ARR);
						while (HAL_GetTick() - calibStartTime < REVOLUTION_TIME_MS && !dft_clamped && !emergency && hasPower()) {
							next_tick += period_us;
							
							// Ramp velocity from 0 to full during warmup to avoid torque spikes
							// at direction reversals and startup
							float elapsed = (float)(HAL_GetTick() - calibStartTime);
							if (elapsed < (float)cogging_warmup_ms) {
								ramp_vel_turns = ramp_rate * elapsed * 0.001f; // turns/s
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
								
								iq_inertia = (J / 100.0f) * current_accel_rad;
								if (this->cogging_calib_inertiaCorr) {
									iq_pid += iq_inertia;
								}
								// Friction FF scaled down: dynamic_friction was measured at 30 RPM,
								// but calib_rpm is much slower (3-12 RPM). Using full value over-compensates
								// and creates a DC bias in position error that flips with direction.
								// 0.5x is a conservative estimate; integrator handles the rest.
								float iq_ff = (ramp_vel_turns > 0) ? dynamic_friction * 0.1f : -dynamic_friction * 0.1f;
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
								captureDebug(TMC4671CoggingDebugPhase::Acquisition, target_rpm, target_pos_f, actual_pos_f, error, iq_pid, iq_ff, iq_inertia, this->cogging_scale * cog_comp, iq_cmd, iq_applied, current_vel_turns, current_accel_rad, J, B, dynamic_friction);
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
#else
								float iq = (float)actual_iq_raw;
								if (this->cogging_calib_inertiaCorr) {
									iq -= iq_inertia;
								}
#endif
#ifdef COGGING_CALIB_ENABLE_ID_DIAG
								float id = (float)getActualFlux();
#endif
								float s1, c1;
								arm_sin_cos_f32(actual_pos_f * 360.0f, &s1, &c1);

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

					// Extract per-direction harmonics immediately after this sweep
					if (dir_samples > 0) {
						float norm = 2.0f / (float)dir_samples;
						for (int k = 1; k < COGGING_CALIB_DFT_HARMONICS; k++) {
							float re = iq_acc_cos[k] * norm;
							float im = iq_acc_sin[k] * norm;
							if (p == 1) { // CW
								cw_harms[k].mag = sqrtf(re*re + im*im);
								cw_harms[k].phase = atan2f(re, im);
							} else { // CCW
								ccw_harms[k].mag = sqrtf(re*re + im*im);
								ccw_harms[k].phase = atan2f(re, im);
							}
						}
						total_samples += dir_samples;
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

					for (int k = 5; k < COGGING_CALIB_DFT_HARMONICS; k++) {
						// Average magnitude (friction is AC-neutral, doesn't affect magnitude)
						float avg_mag = (cw_harms[k].mag + ccw_harms[k].mag) / 2.0f;

						// Phase unwrapping: avoid averaging +179° and -179° to 0°
						float cw_phase = cw_harms[k].phase;
						float ccw_phase = ccw_harms[k].phase;
						float phase_diff = cw_phase - ccw_phase;
						if (phase_diff > PI) ccw_phase += 2.0f * PI;
						if (phase_diff < -PI) ccw_phase -= 2.0f * PI;

						// Average phase (cancels tracking lag delta between CW and CCW)
						float avg_phase = (cw_phase + ccw_phase) / 2.0f;

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
				prev_iter_err = iter_max_err_deg;
			}

			// --- DFT CLAMP RETRY ---
			// If the acquisition saturated torque, Kp is too high for this RPM.
			// Lower it by one step, clear the table, and restart the DFT.
			if (dft_clamped && dft_retry + 1 < DFT_MAX_CLAMP_RETRIES && !emergency && hasPower()) {
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
				continue;  // restart DFT for this profile
			}
			// If we exhausted retries, fall through and use whatever we got.
			if (dft_clamped) {
				broadcastCalibLog(0, "DFT clamp retries exhausted at Kp:%.0f.", pid_soft.Kp);
			} else {
				break;  // no clamp — DFT succeeded, exit retry loop
			}

			} // end DFT retry loop

			// Settle after DFT: motor may have residual motion from the last
			// CCW sweep. Bring to a full stop before phase extraction or the
			// next profile's P-tuner starts.
			{
				float hold_pos = getFilteredPosition();
				float rev_s = 60.0f / fabsf(calib_rpm);
				uint32_t settle_ms = (uint32_t)(rev_s * 1000.0f);
				if (settle_ms < 1000) settle_ms = 1000;
				if (settle_ms > 4000) settle_ms = 4000;
				float settle_tq = max_test_torque * 0.4f;
				uint32_t settleStart = HAL_GetTick();
				while (HAL_GetTick() - settleStart < settle_ms && !emergency && hasPower()) {
					float err = getWrappedError(hold_pos, getFilteredPosition());
					float iq_hold = arm_pid_f32(&pid_soft, err);
					iq_hold = clip<float>(iq_hold, -settle_tq, settle_tq);
					applySafeTorque(iq_hold);
					refreshWatchdog();
					Delay(1);
				}
				applySafeTorque(0);
				Delay(50);
				arm_pid_init_f32(&pid_soft, 1); // fresh PID for next phase
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
		}

			} // end DFT iteration loop
			} // end multi-RPM profile loop

		// 3. SAVE & FINALIZE
		if (!emergency && any_profile_succeeded) {
			broadcastCalibLog(0, "Cogging calibration successful");
			refreshWatchdog();
			saveCoggingTable();

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
				// Get the raw absolute position (no modulo 1.0 wrapping)
				float actual_pos_f = getAbsolutePosition();
				float target_rpm = (actual_pos_f > 0.0f) ? -calib_rpm : calib_rpm;

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

#ifdef COGGING_PHASE_SHIFT_MULTIRPM
				// MULTIRPM: use the Kp that was auto-tuned for the return RPM.
				// Find the nearest scale_curve_rpm_points index to calib_rpm
				// and use the Kp stored there from the P-gain auto-tuner.
				pid_soft.Kp = this->coggingSpeedP; // last profile's Kp as fallback
				{
					float best_dist = 999999.0f;
					uint8_t ret_idx = 0;
					for (uint8_t i = 0; i < SCALE_CURVE_POINTS; i++) {
						float dist = fabsf(scale_curve_rpm_points[i] - calib_rpm);
						if (dist < best_dist) { best_dist = dist; ret_idx = i; }
					}
					// Use stored Kp from the wrapped calib slot that holds this profile's tune
					uint8_t store_src = (ret_idx - 1 < COGGING_MAX_CALIB_PROFILES) ? (ret_idx - 1) : (COGGING_MAX_CALIB_PROFILES - 1);
					uint32_t stored_kp = this->cogging_calib_pidP[store_src];
					if (stored_kp > 0) pid_soft.Kp = (float)stored_kp;
				}
#else
				pid_soft.Kp = (float)this->cogging_calib_pidP[this->cogging_calib_count - 1];
				if (pid_soft.Kp < 50.0f) pid_soft.Kp = this->coggingSpeedP;
#endif
				pid_soft.Ki = 0.0f;
				pid_soft.Kd = 0.0f;
				arm_pid_init_f32(&pid_soft, 1);

				// Get current position
				actual_pos_f = getAbsolutePosition();
				broadcastCalibLog(0, "Return to center: Start pos = %.3f turns", actual_pos_f);

				float target_pos_f = actual_pos_f;
				uint32_t period_us = getActualCalibPeriod(TIM_TMC_ARR); 
				// Fixed 10-second return: calculate RPM from distance
				// RPM = distance_turns / (10s / 60s) = distance_turns * 6
				float distance_turns = fabs(actual_pos_f);
				float ret_rpm = distance_turns * 6.0f; // covers distance in 10 seconds
				if (ret_rpm < 0.5f) ret_rpm = 0.5f;    // minimum speed for very short moves
				const float full_ret_rpm = (actual_pos_f > 0.0f) ? -ret_rpm : ret_rpm;
				target_rpm = full_ret_rpm;
				// Timeout: 10s travel + 5s safety margin
				uint32_t dynamic_timeout_ms = 15000;
				
				uint32_t return_start = HAL_GetTick();
				uint32_t next_tick = micros();
				startCalibTimers(TIM_TMC_ARR);
				while (fabs(actual_pos_f) > 0.005f && !emergency && hasPower()) {
					next_tick += period_us;
					
					// Ramp velocity from 0 to full to prevent torque slam
					float ret_elapsed_ms = (float)(HAL_GetTick() - return_start);
					float ret_ramp = (ret_elapsed_ms < (float)cogging_warmup_ms) ? (ret_elapsed_ms / (float)cogging_warmup_ms) : 1.0f;
					float active_rpm = full_ret_rpm * ret_ramp;
					target_pos_f += (active_rpm / 60.0f) * ((float)period_us / 1000000.0f);

					// Clamp at zero
					if (target_rpm < 0 && target_pos_f < 0.0f) target_pos_f = 0.0f;
					if (target_rpm > 0 && target_pos_f > 0.0f) target_pos_f = 0.0f;

					// Read the raw absolute position
					actual_pos_f = getAbsolutePosition();
					
					// Get raw error towards 0
					float error = target_pos_f - actual_pos_f;
					
					// Calculate torque via the CMSIS PID
					float iq_pid = arm_pid_f32(&pid_soft, error);
					float iq_ff = (target_rpm > 0) ? dynamic_friction * 0.5f : -dynamic_friction * 0.5f;
					float iq_cmd = iq_pid + iq_ff;
					
					// Anti-cogging compensation (uses table extracted by DFT)
					float cog_comp = 0.0f;
					if (this->cogging_enabled) {
						float angle_rad = actual_pos_f * 2.0f * PI;
						for (uint8_t h = 0; h < COGGING_HARMONICS_COUNT; h++) {
							if (cogging_harmonics[h].amplitude > 0.0f) {
								cog_comp += cogging_harmonics[h].amplitude * arm_sin_f32(angle_rad * cogging_harmonics[h].order + cogging_harmonics[h].phase);
							}
						}
						cog_comp *= this->cogging_scale;
					}
					float iq_applied = iq_cmd + cog_comp;
					
					// Use full test torque limit (standard for all phases)
					iq_applied = clip<float,float>(iq_applied, -max_test_torque, max_test_torque);
					captureDebug(TMC4671CoggingDebugPhase::ReturnToCenter, target_rpm, target_pos_f, actual_pos_f, error, iq_pid, iq_ff, 0.0f, cog_comp, iq_cmd, iq_applied, 0.0f, 0.0f, J, B, dynamic_friction);
					applySafeTorque(iq_applied);
					
					refreshWatchdog();
					if (HAL_GetTick() - return_start > dynamic_timeout_ms) {
						broadcastCalibLog(0, "Return to center timeout");
						break; // safety timeout
					}
#ifdef TIM_CALIBRATION
					// Yield thread via timer interrupt notifications to avoid busy-waiting, falling back if timer is disabled
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
				broadcastCalibLog(0, "Centering done. Re-aligning encoder...");
				
				goto cleanup;
		}
	}

cleanup:
	dbg.phase = static_cast<uint32_t>((errorMessage != nullptr || emergency) ? TMC4671CoggingDebugPhase::Aborted : TMC4671CoggingDebugPhase::Completed);
	if (errorMessage) {
		broadcastCalibLog(1, errorMessage);
	} else {
		broadcastCalibLog(1, "Cogging detection finished");
	}
	
	setTargetVelocity(0); // Ensure motor stops

	if(iq_acc_cos) vPortFree(iq_acc_cos);
	if(iq_acc_sin) vPortFree(iq_acc_sin);
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
	// Transition to a stable state instead of potentially jumping to uninitialized
	if (errorMessage) {
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
