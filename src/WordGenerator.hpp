#pragma once
#include "plugin.hpp"

struct WordGenerator : rack::Module {
    enum ParamIds {
        // Single params
        CLOCK_RATE_PARAM,           // 0
        CLOCK_MODE_PARAM,           // 1 (0: Internal, 1: External)
        WORD_LENGTH_PARAM,          // 2 (Defines sequence length)
        MODE_PARAM,                 // 3 (0: 16-bit, 1: 32-bit)
        RUN_MODE_PARAM,             // 4 (0: Auto, 1: Single)
        PRN_PROB_PARAM,             // 5 (Formerly RZ_NRZ_A_PARAM)
        SINGLE_CYCLE_BUTTON_PARAM,  // 6 (Formerly RZ_NRZ_B_PARAM)
        INVERT_A_PARAM,             // 7 (Formerly PRN_PROB_PARAM)
        INVERT_B_PARAM,             // 8 (Formerly SINGLE_CYCLE_BUTTON_PARAM)
        RATE_ATTEN_PARAM,           // New: Attenuverter for Clock Rate CV

        // Bit params
        BIT_A_PARAMS,               // Starts at 9 (after renumbering)
        BIT_A_PARAMS_LAST = BIT_A_PARAMS + 15,
        BIT_B_PARAMS,               // Starts at BIT_A_PARAMS + 16
        BIT_B_PARAMS_LAST = BIT_B_PARAMS + 15,

        NUM_PARAMS
    };
    
    enum InputIds {
        CLOCK_INPUT,
        RESET_INPUT,
        MANUAL_TRIGGER_INPUT,     // External trigger for single cycle start
        PRN_PROBABILITY_INPUT,    // CV for PRN probability
        INVERT_A_FLIP_TRIGGER_INPUT, // Changed from INVERT_A_INPUT (CV) to trigger
        INVERT_B_FLIP_TRIGGER_INPUT, // Changed from INVERT_B_INPUT (CV) to trigger
        WORD_LENGTH_CV_INPUT,     // New: CV for Word Length
        MODE_FLIP_TRIGGER_INPUT,  // New: Trigger to flip 16/32 bit mode
        RATE_CV_INPUT,            // New: CV for Clock Rate
        NUM_INPUTS
    };
    
    enum OutputIds {
        WORD_A_GATE_OUTPUT,     // Renamed from WORD_A_OUTPUT
        WORD_B_GATE_OUTPUT,     // Renamed from WORD_B_OUTPUT
        WORD_A_CV_OUTPUT,       // New CV output for Word A
        WORD_B_CV_OUTPUT,       // New CV output for Word B
        FIRST_BIT_OUTPUT,
        LAST_BIT_OUTPUT,
        CLOCK_OUTPUT,
        NUM_OUTPUTS
    };
    
    enum LightIds {
        // WORD_A_LIGHT, // These seem unused if BIT_A/B_LIGHTS are present
        // WORD_B_LIGHT,
        BIT_A_LIGHTS,
        BIT_A_LIGHTS_LAST = BIT_A_LIGHTS + 15,
        BIT_B_LIGHTS,
        BIT_B_LIGHTS_LAST = BIT_B_LIGHTS + 15,
        NUM_LIGHTS
    };
    
    // Module state
    uint16_t wordA_switches = 0;
    uint16_t wordB_switches = 0;
    int currentStep = 0;
    bool clockInputState = false;
    float clockPhase = 0.f;
    
    bool isRunning = true;
    bool singleCycleComplete = false;
    bool forceSingleCycle = false;
    
    uint32_t prnState = 0x1; // For generating random bits
    uint16_t currentPrnPatternA = 0; // Stores the 16-bit random pattern for Word A for the current step
    uint16_t currentPrnPatternB = 0; // Stores the 16-bit random pattern for Word B for the current step
    uint16_t stepFinalWordA = 0;     // Word A content (switches or PRN) decided at the last clock tick
    uint16_t stepFinalWordB = 0;     // Word B content (switches or PRN) decided at the last clock tick
    
    // Trigger outputs & helpers
    dsp::PulseGenerator firstBitPulse;
    dsp::PulseGenerator lastBitPulse;
    dsp::PulseGenerator wordAGatePulse; // New for Word A Gate output
    dsp::PulseGenerator wordBGatePulse; // New for Word B Gate output
    // dsp::PulseGenerator manualClockPulse; // REMOVED as manual clock mode is removed
    dsp::SchmittTrigger clockTrigger;
    dsp::SchmittTrigger singleCycleButtonTrigger;
    dsp::SchmittTrigger resetTrigger;
    dsp::SchmittTrigger manualExtTrigger; // For single cycle via MANUAL_TRIGGER_INPUT
    dsp::SchmittTrigger invertAFlipTrigger; // New: for INVERT_A_FLIP_TRIGGER_INPUT
    dsp::SchmittTrigger invertBFlipTrigger; // New: for INVERT_B_FLIP_TRIGGER_INPUT
    dsp::SchmittTrigger modeFlipTrigger;    // New: for MODE_FLIP_TRIGGER_INPUT
    
    WordGenerator();
    void process(const ProcessArgs& args) override;
    
private:
    uint8_t computeOutputByte(uint16_t word, int startBit);
    void advanceSequence();
    uint32_t generatePRN(uint32_t state); // Still useful for getting raw random bits
};

extern rack::Model* modelWordGenerator; 