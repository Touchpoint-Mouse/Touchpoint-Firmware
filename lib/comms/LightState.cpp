#include "LightState.h"

#include <math.h>

LightState::SolidEffect::SolidEffect(RgbColor color) : color(color) {}

void LightState::OffEffect::apply(Adafruit_NeoPixel& pixel, uint32_t) {
	pixel.setPixelColor(0, 0, 0, 0);
}

void LightState::SolidEffect::apply(Adafruit_NeoPixel& pixel, uint32_t) {
	pixel.setPixelColor(0, pixel.Color(color.red, color.green, color.blue));
}

LightState::RainbowEffect::RainbowEffect(uint8_t brightness, float speedHz, uint8_t saturation, uint16_t initialHue)
	: hue(static_cast<float>(initialHue)),
	  speedHz(speedHz > 0.0f ? speedHz : 1.0f),
	  lastMs(0),
	  saturation(saturation),
	  brightness(brightness) {}

void LightState::RainbowEffect::apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) {
	if (lastMs == 0) {
		lastMs = nowMs;
	}

	const uint32_t deltaMs = nowMs - lastMs;
	lastMs = nowMs;

	const float hueDelta = speedHz * 65535.0f * (static_cast<float>(deltaMs) / 1000.0f);
	hue += hueDelta;
	while (hue >= 65535.0f) {
		hue -= 65535.0f;
	}

	pixel.setPixelColor(0, pixel.gamma32(pixel.ColorHSV(static_cast<uint16_t>(hue), saturation, brightness)));
}

LightState::WaveEffect::WaveEffect(uint8_t amplitude, float frequencyHz, RgbColor tint, float initialPhase)
	: amplitude(amplitude),
	  frequencyHz(frequencyHz > 0.0f ? frequencyHz : 1.5f),
	  phase(initialPhase),
	  lastMs(0),
	  tint(tint) {}

void LightState::WaveEffect::apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) {
	if (lastMs == 0) {
		lastMs = nowMs;
	}

	const uint32_t deltaMs = nowMs - lastMs;
	lastMs = nowMs;

	phase += frequencyHz * (static_cast<float>(deltaMs) / 1000.0f);
	while (phase >= 1.0f) {
		phase -= 1.0f;
	}

	const float wave = 0.5f * (waveSample(phase * 2.0f * PI) + 1.0f);
	const uint8_t value = static_cast<uint8_t>(wave * amplitude);

	const uint8_t r = static_cast<uint8_t>((static_cast<uint16_t>(tint.red) * value) / 255);
	const uint8_t g = static_cast<uint8_t>((static_cast<uint16_t>(tint.green) * value) / 255);
	const uint8_t b = static_cast<uint8_t>((static_cast<uint16_t>(tint.blue) * value) / 255);
	pixel.setPixelColor(0, pixel.Color(r, g, b));
}

float LightState::WaveEffect::phase01() const {
	return phase;
}

LightState::PulseEffect::PulseEffect(uint8_t amplitude, float frequencyHz, RgbColor tint, float initialPhase)
	: WaveEffect(amplitude, frequencyHz, tint, initialPhase) {}

float LightState::PulseEffect::waveSample(float phaseRadians) const {
	return sinf(phaseRadians);
}

LightState::BlinkEffect::BlinkEffect(uint8_t amplitude, float frequencyHz, float dutyCycle, RgbColor tint, float initialPhase)
	: WaveEffect(amplitude, frequencyHz, tint, initialPhase),
	  dutyCycle(dutyCycle < 0.0f ? 0.0f : (dutyCycle > 1.0f ? 1.0f : dutyCycle)) {}

float LightState::BlinkEffect::waveSample(float) const {
	return phase01() < dutyCycle ? 1.0f : -1.0f;
}

LightState::LightState(uint8_t dataPin, uint16_t pixelCount, neoPixelType type)
	: pixel(pixelCount, dataPin, type),
		activeEffectImpl(nullptr),
		lastUpdateMs(0) {}

void LightState::begin() {
	pixel.begin();
	pixel.clear();
	pixel.show();
	lastUpdateMs = millis();
}

void LightState::setEffect(Effect& effect) {
	activeEffectImpl = &effect;
}

LightState::Effect& LightState::getEffect() const {
	return *activeEffectImpl;
}

LightState::RgbColor LightState::colorFromPreset(LightColor color) {
	return COLOR_TABLE[static_cast<size_t>(color)];
}

void LightState::update() {
	const uint32_t nowMs = millis();
	if ((nowMs - lastUpdateMs) < 5) {
		return;
	}
	lastUpdateMs = nowMs;

	if (activeEffectImpl != nullptr) {
		activeEffectImpl->apply(pixel, nowMs);
		pixel.show();
	}
}
