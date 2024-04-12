#ifndef STK_P2FLUTE_H
#define STK_P2FLUTE_H

#include "Instrmnt.h"
#include "JetTable.h"
#include "DelayL.h"
#include "OnePole.h"
#include "PoleZero.h"
#include "Noise.h"
#include "ADSR.h"
#include "SineWave.h"
#include <iostream>
#include <fstream>

namespace stk {
	class P2Flute : public Instrmnt {
	public:
		P2Flute();
		~P2Flute();
		void clear(void);
		void setFrequency(StkFloat frequency);
		void setJetReflection(StkFloat coefficient) { jetReflection_ = coefficient; }
		void setEndReflection(StkFloat coefficient) { endReflection_ = coefficient; }
		void setJetDelay(StkFloat aRatio);
		void startBlowing(StkFloat amplitude, StkFloat rate);
		void stopBlowing(StkFloat rate);
		void noteOn(StkFloat frequency, StkFloat amplitude);
		void noteOff(StkFloat amplitude);

		//! Perform the control change specified by \e number and \e value (0.0 - 128.0).
		void controlChange(int number, StkFloat value);

		StkFloat tick(unsigned int channel = 0);
		StkFrames& tick(StkFrames& frames, unsigned int channel = 0);

	protected:
		DelayL   jetDelay_;
		DelayL   boreDelay_;
		JetTable jetTable_;
		OnePole  filter_;
		PoleZero dcBlock_;
		Noise    noise_;
		ADSR     adsr_;
		SineWave vibrato_;
		std::ofstream myfile_;

		StkFloat lastFrequency_;
		StkFloat maxPressure_;
		StkFloat jetReflection_;
		StkFloat endReflection_;
		StkFloat noiseGain_;
		StkFloat vibratoGain_;
		StkFloat outputGain_;
		StkFloat jetRatio_;
	};

	inline StkFloat P2Flute::tick(unsigned int)
	{
		StkFloat pressureDiff;
		StkFloat breathPressure;

		// Calculate the breath pressure (envelope + noise + vibrato)
		breathPressure = maxPressure_ * adsr_.tick();
		breathPressure += breathPressure * (noiseGain_ * noise_.tick() + vibratoGain_ * vibrato_.tick());

		StkFloat temp = -filter_.tick(boreDelay_.lastOut());
		//temp = dcBlock_.tick( temp ); // Block DC on reflection.

		pressureDiff = breathPressure - (jetReflection_ * temp);
		pressureDiff = jetDelay_.tick(pressureDiff);
		//pressureDiff = jetTable_.tick( pressureDiff ) + (endReflection_ * temp);
		//pressureDiff = - 0.5 * sin(3.1415926535 * pressureDiff);
		//pressureDiff = 0.2 -0.5 * tanh(pressureDiff);

		//myfile_ << pressureDiff << std::endl;
		
		StkFloat a = 3.0;
		pressureDiff = 1.0/3.0 * tanh((pressureDiff - ((a + 0.5) / a)) * 2 * a) + 0.5;
		//pressureDiff = 3 * pressureDiff - 3;
		if (pressureDiff > 1.0) pressureDiff = 1.0;
		if (pressureDiff < -1.0) pressureDiff = -1.0;
		
		pressureDiff = dcBlock_.tick(pressureDiff) + (endReflection_ * temp); // moved the DC blocker to after the jet non-linearity (GPS, 29 Jan. 2020)
		lastFrame_[0] = (StkFloat)0.3 * boreDelay_.tick(pressureDiff);

		lastFrame_[0] *= outputGain_;
		return lastFrame_[0];
	}

	inline StkFrames& P2Flute::tick(StkFrames& frames, unsigned int channel)
	{
		unsigned int nChannels = lastFrame_.channels();
#if defined(_STK_DEBUG_)
		if (channel > frames.channels() - nChannels) {
			oStream_ << "Flute::tick(): channel and StkFrames arguments are incompatible!";
			handleError(StkError::FUNCTION_ARGUMENT);
		}
#endif

		StkFloat* samples = &frames[channel];
		unsigned int j, hop = frames.channels() - nChannels;
		if (nChannels == 1) {
			for (unsigned int i = 0; i < frames.frames(); i++, samples += hop)
				*samples++ = tick();
		}
		else {
			for (unsigned int i = 0; i < frames.frames(); i++, samples += hop) {
				*samples++ = tick();
				for (j = 1; j < nChannels; j++)
					*samples++ = lastFrame_[j];
			}
		}

		return frames;
	}
}


#endif