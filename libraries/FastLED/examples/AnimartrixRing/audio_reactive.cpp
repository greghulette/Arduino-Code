// audio_reactive.cpp
#include "audio_reactive.h"
#include "fl/audio/detector/vibe.h"

void AudioReactive::begin(fl::UIAudio &audio) {
    auto input = audio.audioInput();
    if (input) {
        processor = FastLED.add(input);
        autoPump = true;
    }
    if (!processor) {
        processor = fl::make_shared<fl::audio::Processor>();
    }
}

void AudioReactive::connectToEngine(fl::FxEngine &fxEngine,
                                    fl::UICheckbox &enableVibe,
                                    fl::UISlider &speedMultiplier,
                                    fl::UISlider &baseSpeed,
                                    fl::UISlider &timeSpeed) {
    processor->onVibeLevels(
        [&fxEngine, &enableVibe, &speedMultiplier, &baseSpeed,
         &timeSpeed](const fl::audio::detector::VibeLevels &vibe) {
            if (enableVibe.value()) {
                float bassBoost = (vibe.bass - 1.0f) * speedMultiplier.value();
                float speed = baseSpeed.value() + bassBoost;
                speed *= timeSpeed.value();
                fxEngine.setSpeed(speed);
            }
        });
}

void AudioReactive::pump(fl::UIAudio &audio, fl::UICheckbox &enableVibe) {
    if (autoPump)
        return;
    fl::audio::Sample sample = audio.next();
    if (!sample.isValid())
        return;
    sampleCount++;
    if (sampleCount == 1) {
        printf("AnimartrixRing: First audio sample received! "
               "enableVibeReactive=%d\n",
               (int)enableVibe.value());
    } else if (sampleCount % 172 == 0) {
        printf("AnimartrixRing: %u audio samples processed, "
               "enableVibeReactive=%d\n",
               (unsigned)sampleCount, (int)enableVibe.value());
    }
    if (enableVibe.value()) {
        processor->update(sample);
    }
}
