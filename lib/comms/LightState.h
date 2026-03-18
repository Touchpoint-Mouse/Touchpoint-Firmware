#ifndef LIGHT_STATE_H
#define LIGHT_STATE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#include <array>

class LightState {
public:
	enum class LightColor : uint8_t {
		Off = 0,
		Red,
		Green,
		Blue,
		Cyan,
		Magenta,
		Yellow,
		White,
		Count
	};

	struct RgbColor {
		uint8_t red;
		uint8_t green;
		uint8_t blue;
	};

	class Effect {
	public:
		virtual ~Effect() = default;
		virtual void apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) = 0;
	};

	class OffEffect final : public Effect {
	public:
		void apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) override;
	};

	class SolidEffect final : public Effect {
	public:
		explicit SolidEffect(RgbColor color = {0, 255, 255});
		void apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) override;

	private:
		RgbColor color;
	};

	class RainbowEffect final : public Effect {
	public:
		RainbowEffect(
			uint8_t brightness = 64,
			float speedHz = 1.0f,
			uint8_t saturation = 255,
			uint16_t initialHue = 0
		);
		void apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) override;

	private:
		float hue = 0.0f;
		float speedHz = 1.0f;
		uint32_t lastMs = 0;
		uint8_t saturation = 255;
		uint8_t brightness = 64;
	};

	class WaveEffect : public Effect {
	public:
		WaveEffect(
			uint8_t amplitude = 180,
			float frequencyHz = 1.5f,
			RgbColor tint = {255, 64, 32},
			float initialPhase = 0.0f
		);
		void apply(Adafruit_NeoPixel& pixel, uint32_t nowMs) override;

	protected:
		virtual float waveSample(float phaseRadians) const = 0;
		float phase01() const;

	private:
		uint8_t amplitude = 180;
		float frequencyHz = 1.5f;
		float phase = 0.0f;
		uint32_t lastMs = 0;
		RgbColor tint = {255, 64, 32};
	};

	class PulseEffect final : public WaveEffect {
	public:
		PulseEffect(
			uint8_t amplitude = 180,
			float frequencyHz = 1.5f,
			RgbColor tint = {255, 64, 32},
			float initialPhase = 0.0f
		);

	protected:
		float waveSample(float phaseRadians) const override;
	};

	class BlinkEffect final : public WaveEffect {
	public:
		BlinkEffect(
			uint8_t amplitude = 180,
			float frequencyHz = 1.5f,
			float dutyCycle = 0.5f,
			RgbColor tint = {255, 64, 32},
			float initialPhase = 0.0f
		);

	protected:
		float waveSample(float phaseRadians) const override;

	private:
		float dutyCycle = 0.5f;
	};

	explicit LightState(uint8_t dataPin, uint16_t pixelCount = 1, neoPixelType type = NEO_GRB + NEO_KHZ800);

	void begin();
	void setEffect(Effect& effect);
	Effect& getEffect() const;
	static RgbColor colorFromPreset(LightColor color);
	void update();

private:
	static constexpr size_t COLOR_COUNT = static_cast<size_t>(LightColor::Count);
	static constexpr std::array<RgbColor, COLOR_COUNT> COLOR_TABLE{{
		{0, 0, 0},
		{255, 0, 0},
		{0, 255, 0},
		{0, 0, 255},
		{0, 255, 255},
		{255, 0, 255},
		{255, 255, 0},
		{255, 255, 255}
	}};

	Adafruit_NeoPixel pixel;
	Effect* activeEffectImpl;
	uint32_t lastUpdateMs;
};

#endif // LIGHT_STATE_H
