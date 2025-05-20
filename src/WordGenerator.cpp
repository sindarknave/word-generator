#include "WordGenerator.hpp"

using namespace rack;

WordGenerator::WordGenerator() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
    
    auto clockRateParamQuantity = configParam(CLOCK_RATE_PARAM, log2f(0.1f), log2f(1000.f), log2f(10.f), "Clock Rate", " Hz");
    clockRateParamQuantity->displayBase = 2.f;

    configSwitch(CLOCK_MODE_PARAM, 0, 1, 0, "Clock Mode", {"Internal", "External"});
    
    auto wordLengthParamQuantity = configParam(WORD_LENGTH_PARAM, 1.f, 16.f, 16.f, "Word Length", " bits");
    wordLengthParamQuantity->snapEnabled = true;

    configSwitch(MODE_PARAM, 0, 1, 0, "Mode", {"16-bit", "32-bit"});
    configSwitch(RUN_MODE_PARAM, 0, 1, 0, "Run Mode", {"Auto", "Single"});
    configParam(PRN_PROB_PARAM, 0.f, 1.f, 0.f, "PRN Probability", "");
    configButton(SINGLE_CYCLE_BUTTON_PARAM, "Single Cycle Trigger");
    configSwitch(INVERT_A_PARAM, 0, 1, 0, "Invert Word A");
    configSwitch(INVERT_B_PARAM, 0, 1, 0, "Invert Word B");
    configParam(RATE_ATTEN_PARAM, -1.f, 1.f, 0.f, "Clock Rate Attenuverter");
    
    wordA_switches = 0;
    wordB_switches = 0;
    for (int i = 0; i < 16; i++) {
        configSwitch(BIT_A_PARAMS + i, 0, 1, 0, string::f("Word A Bit %d", i + 1));
        if (params[BIT_A_PARAMS + i].getValue() > 0.5f) wordA_switches |= (1 << i);
        configSwitch(BIT_B_PARAMS + i, 0, 1, 0, string::f("Word B Bit %d", i + 1));
        if (params[BIT_B_PARAMS + i].getValue() > 0.5f) wordB_switches |= (1 << i);
    }
    
    configInput(CLOCK_INPUT, "Clock");
    configInput(RESET_INPUT, "Reset");
    configInput(MANUAL_TRIGGER_INPUT, "Single Cycle Ext. Trigger");
    configInput(PRN_PROBABILITY_INPUT, "PRN Probability CV");
    configInput(INVERT_A_FLIP_TRIGGER_INPUT, "Flip Invert A Trigger");
    configInput(INVERT_B_FLIP_TRIGGER_INPUT, "Flip Invert B Trigger");
    configInput(WORD_LENGTH_CV_INPUT, "Word Length CV");
    configInput(MODE_FLIP_TRIGGER_INPUT, "Flip 16/32 Mode Trigger");
    configInput(RATE_CV_INPUT, "Clock Rate CV");
    
    configOutput(WORD_A_GATE_OUTPUT, "Word A Gate");
    configOutput(WORD_B_GATE_OUTPUT, "Word B Gate");
    configOutput(WORD_A_CV_OUTPUT, "Word A CV");
    configOutput(WORD_B_CV_OUTPUT, "Word B CV");
    configOutput(FIRST_BIT_OUTPUT, "First Bit");
    configOutput(LAST_BIT_OUTPUT, "Last Bit");
    configOutput(CLOCK_OUTPUT, "Clock");
    
    prnState = rack::random::u32(); if (prnState == 0) prnState = 0xabcdef12;
    currentPrnPatternA = prnState & 0xFFFF; currentPrnPatternB = (prnState >> 16) & 0xFFFF;
    stepFinalWordA = wordA_switches;
    stepFinalWordB = wordB_switches;
}

void WordGenerator::process(const ProcessArgs& args) {
    int runMode = std::round(params[RUN_MODE_PARAM].getValue());
    int wordLength;
    int wordLengthParamValue = (int)std::round(params[WORD_LENGTH_PARAM].getValue());
    if (inputs[WORD_LENGTH_CV_INPUT].isConnected()) {
        float cvValue = inputs[WORD_LENGTH_CV_INPUT].getVoltage();
        int cvSteps = static_cast<int>(std::round(cvValue * 1.5f));
        wordLength = std::max(1, std::min(16, wordLengthParamValue + cvSteps));
    } else {
        wordLength = wordLengthParamValue;
    }
    wordLength = std::max(1, wordLength);

    int clockMode = std::round(params[CLOCK_MODE_PARAM].getValue());
    if (modeFlipTrigger.process(inputs[MODE_FLIP_TRIGGER_INPUT].getVoltage())) {
        params[MODE_PARAM].setValue(1.0f - params[MODE_PARAM].getValue());
    }
    bool is16BitMode = params[MODE_PARAM].getValue() < 0.5f;

    uint16_t current_wordA_switches = 0;
    uint16_t current_wordB_switches = 0;
    for (int i = 0; i < 16; i++) {
        if (params[BIT_A_PARAMS + i].getValue() > 0.5f) current_wordA_switches |= (1 << i);
        if (params[BIT_B_PARAMS + i].getValue() > 0.5f) current_wordB_switches |= (1 << i);
    }
    wordA_switches = current_wordA_switches;
    wordB_switches = current_wordB_switches;

    // Triggers & State Management (Invert, Reset, Single Cycle)
    if (invertAFlipTrigger.process(inputs[INVERT_A_FLIP_TRIGGER_INPUT].getVoltage())) {
        params[INVERT_A_PARAM].setValue(1.0f - params[INVERT_A_PARAM].getValue());
    }
    if (invertBFlipTrigger.process(inputs[INVERT_B_FLIP_TRIGGER_INPUT].getVoltage())) {
        params[INVERT_B_PARAM].setValue(1.0f - params[INVERT_B_PARAM].getValue());
    }
    bool resetPressed = resetTrigger.process(inputs[RESET_INPUT].getVoltage());
    bool singleCycleBtnPressed = singleCycleButtonTrigger.process(params[SINGLE_CYCLE_BUTTON_PARAM].getValue());
    bool manualExtGate = manualExtTrigger.process(inputs[MANUAL_TRIGGER_INPUT].getVoltage());

    if (resetPressed) {
        currentStep = 0; clockPhase = 0.f; forceSingleCycle = false; singleCycleComplete = false;
        prnState = rack::random::u32(); if (prnState == 0) prnState = 0xabcdef12;
        currentPrnPatternA = prnState & 0xFFFF; currentPrnPatternB = (prnState >> 16) & 0xFFFF;
        stepFinalWordA = wordA_switches; stepFinalWordB = wordB_switches;
    }
    if (singleCycleBtnPressed || manualExtGate) {
        currentStep = 0; forceSingleCycle = true; singleCycleComplete = false;
        if (clockMode == 1) { // EXTERNAL: immediate first step processing
            prnState = generatePRN(prnState);
            currentPrnPatternA = prnState & 0xFFFF; currentPrnPatternB = (prnState >> 16) & 0xFFFF;
            float prnProbKnobValue = params[PRN_PROB_PARAM].getValue();
            float prnProbCv = inputs[PRN_PROBABILITY_INPUT].isConnected() ? (inputs[PRN_PROBABILITY_INPUT].getVoltage() / 10.f) : 0.f;
            float finalPrnProbability = clamp(prnProbKnobValue + prnProbCv, 0.f, 1.f);
            uint16_t firstStepWordA_imm = 0; uint16_t firstStepWordB_imm = 0;
            for (int i = 0; i < 16; ++i) {
                if (random::uniform() < finalPrnProbability) { if (currentPrnPatternA & (1 << i)) firstStepWordA_imm |= (1 << i); }
                else { if (wordA_switches & (1 << i)) firstStepWordA_imm |= (1 << i); }
                if (random::uniform() < finalPrnProbability) { if (currentPrnPatternB & (1 << i)) firstStepWordB_imm |= (1 << i); }
                else { if (wordB_switches & (1 << i)) firstStepWordB_imm |= (1 << i); }
            }
            stepFinalWordA = firstStepWordA_imm; stepFinalWordB = firstStepWordB_imm;
            firstBitPulse.trigger(0.001f);
            int effectiveSeqLen = is16BitMode ? wordLength : (wordLength * 2);
            if (0 == effectiveSeqLen - 1) { lastBitPulse.trigger(0.001f); singleCycleComplete = true; }
        }
    }

    if (forceSingleCycle) { isRunning = !singleCycleComplete; if(singleCycleComplete) forceSingleCycle = false; }
    else { isRunning = (runMode == 0); }
    
    bool clockTick = false;
    if (clockMode == 0) { // Internal Clock
        float clockRateValue = params[CLOCK_RATE_PARAM].getValue();
        float modulatedClockRate = clockRateValue;
        if (inputs[RATE_CV_INPUT].isConnected()) {
            float cvVoltage = inputs[RATE_CV_INPUT].getVoltage();
            float attenuverterValue = params[RATE_ATTEN_PARAM].getValue();
            modulatedClockRate += (cvVoltage / 5.0f) * attenuverterValue;
        }
        // Clamp the modulated rate to the original knob's log range
        modulatedClockRate = clamp(modulatedClockRate, log2f(0.1f), log2f(1000.f));
        float clockFreq = powf(2.f, modulatedClockRate);
        clockPhase += clockFreq * args.sampleTime;
        if (clockPhase >= 1.f) { clockTick = true; clockPhase -= floorf(clockPhase); }
        clockInputState = clockPhase < 0.5f;
        outputs[CLOCK_OUTPUT].setVoltage(clockInputState ? 10.f : 0.f);
    } else { // External Clock
        if (clockTrigger.process(inputs[CLOCK_INPUT].getVoltage())) { clockTick = true; }
        clockInputState = inputs[CLOCK_INPUT].getVoltage() >= 1.f;
        outputs[CLOCK_OUTPUT].setVoltage(inputs[CLOCK_INPUT].getVoltage());
    }
    
    if (clockTick && isRunning) {
        prnState = generatePRN(prnState);
        currentPrnPatternA = prnState & 0xFFFF; currentPrnPatternB = (prnState >> 16) & 0xFFFF;
        float prnProbKnobValue = params[PRN_PROB_PARAM].getValue();
        float prnProbCv = inputs[PRN_PROBABILITY_INPUT].isConnected() ? (inputs[PRN_PROBABILITY_INPUT].getVoltage() / 10.f) : 0.f;
        float finalPrnProbability = clamp(prnProbKnobValue + prnProbCv, 0.f, 1.f);
        uint16_t nextStepWordA = 0; uint16_t nextStepWordB = 0;
        for (int i = 0; i < 16; ++i) {
            if (random::uniform() < finalPrnProbability) { if (currentPrnPatternA & (1 << i)) nextStepWordA |= (1 << i); }
            else { if (wordA_switches & (1 << i)) nextStepWordA |= (1 << i); }
            if (random::uniform() < finalPrnProbability) { if (currentPrnPatternB & (1 << i)) nextStepWordB |= (1 << i); }
            else { if (wordB_switches & (1 << i)) nextStepWordB |= (1 << i); }
        }
        stepFinalWordA = nextStepWordA; stepFinalWordB = nextStepWordB;

        // Determine bit states for the current step to trigger gates
        bool triggerGateA = false;
        bool triggerGateB = false;
        int stepInSegmentForGateTrig;
        bool currentEffectiveInvertA_forGate = params[INVERT_A_PARAM].getValue() > 0.5f;
        bool currentEffectiveInvertB_forGate = params[INVERT_B_PARAM].getValue() > 0.5f;
        uint16_t displayWordA_forGate = currentEffectiveInvertA_forGate ? (~stepFinalWordA & 0xFFFF) : stepFinalWordA;
        uint16_t displayWordB_forGate = currentEffectiveInvertB_forGate ? (~stepFinalWordB & 0xFFFF) : stepFinalWordB;

        if (is16BitMode) {
            stepInSegmentForGateTrig = currentStep % wordLength;
            if (stepInSegmentForGateTrig < wordLength) {
                triggerGateA = (displayWordA_forGate & (1 << stepInSegmentForGateTrig)) != 0;
                triggerGateB = (displayWordB_forGate & (1 << stepInSegmentForGateTrig)) != 0;
            }
        } else { // 32-bit mode
            stepInSegmentForGateTrig = currentStep % wordLength;
            bool firstHalf_forGate = (currentStep / wordLength) % 2 == 0;
            if (stepInSegmentForGateTrig < wordLength) {
                if (firstHalf_forGate) {
                    triggerGateA = (displayWordA_forGate & (1 << stepInSegmentForGateTrig)) != 0;
                    triggerGateB = (displayWordB_forGate & (1 << stepInSegmentForGateTrig)) != 0;
                } else {
                    triggerGateA = (displayWordB_forGate & (1 << stepInSegmentForGateTrig)) != 0;
                    triggerGateB = (displayWordA_forGate & (1 << stepInSegmentForGateTrig)) != 0;
                }
            }
        }
        if (triggerGateA) wordAGatePulse.trigger(0.001f);
        if (triggerGateB) wordBGatePulse.trigger(0.001f);

        if (currentStep == 0) { firstBitPulse.trigger(0.001f); }
        int effectiveSequenceLength = is16BitMode ? wordLength : (wordLength * 2);
        if (currentStep == effectiveSequenceLength - 1) { lastBitPulse.trigger(0.001f); if (forceSingleCycle) { singleCycleComplete = true; } }
        advanceSequence();
    }

    // Output Generation Block
    bool bitA_out = false; bool bitB_out = false; int stepInCurrentSegment = 0;
    bool currentEffectiveInvertA = params[INVERT_A_PARAM].getValue() > 0.5f;
    bool currentEffectiveInvertB = params[INVERT_B_PARAM].getValue() > 0.5f;
    uint16_t displayWordA = currentEffectiveInvertA ? (~stepFinalWordA & 0xFFFF) : stepFinalWordA;
    uint16_t displayWordB = currentEffectiveInvertB ? (~stepFinalWordB & 0xFFFF) : stepFinalWordB;

    if (is16BitMode) {
        stepInCurrentSegment = currentStep % wordLength;
        if (stepInCurrentSegment < wordLength) {
            bitA_out = (displayWordA & (1 << stepInCurrentSegment)) != 0;
            bitB_out = (displayWordB & (1 << stepInCurrentSegment)) != 0;
        }
    } else { // 32-bit mode
        stepInCurrentSegment = currentStep % wordLength;
        bool firstHalf = (currentStep / wordLength) % 2 == 0;
        if (stepInCurrentSegment < wordLength) {
            if (firstHalf) {
                bitA_out = (displayWordA & (1 << stepInCurrentSegment)) != 0;
                bitB_out = (displayWordB & (1 << stepInCurrentSegment)) != 0;
            } else {
                bitA_out = (displayWordB & (1 << stepInCurrentSegment)) != 0;
                bitB_out = (displayWordA & (1 << stepInCurrentSegment)) != 0;
            }
        }
    }
    
    // RZ/NRZ logic removed here
    outputs[WORD_A_CV_OUTPUT].setVoltage(bitA_out ? 10.f : 0.f);    // CV A output
    outputs[WORD_B_CV_OUTPUT].setVoltage(bitB_out ? 10.f : 0.f);    // CV B output
    outputs[WORD_A_GATE_OUTPUT].setVoltage(wordAGatePulse.process(args.sampleTime) ? 10.f : 0.f); // Gate A output
    outputs[WORD_B_GATE_OUTPUT].setVoltage(wordBGatePulse.process(args.sampleTime) ? 10.f : 0.f); // Gate B output

    outputs[FIRST_BIT_OUTPUT].setVoltage(firstBitPulse.process(args.sampleTime) ? 10.f : 0.f);
    outputs[LAST_BIT_OUTPUT].setVoltage(lastBitPulse.process(args.sampleTime) ? 10.f : 0.f);
    
    for (int i = 0; i < 16; ++i) {
        if (i >= wordLength) { lights[BIT_A_LIGHTS + i].setBrightness(0.1f); lights[BIT_B_LIGHTS + i].setBrightness(0.1f); }
        else { lights[BIT_A_LIGHTS + i].setBrightness((displayWordA & (1 << i)) ? 1.f : 0.f); lights[BIT_B_LIGHTS + i].setBrightness((displayWordB & (1 << i)) ? 1.f : 0.f); }
    }
}

void WordGenerator::advanceSequence() {
    int wordLen = std::max(1, (int)std::round(params[WORD_LENGTH_PARAM].getValue())); // Re-read for safety, though wordLength from process() could be passed
    bool is16Bit = params[MODE_PARAM].getValue() < 0.5f;
    int runModeVal = std::round(params[RUN_MODE_PARAM].getValue());
    if ((runModeVal == 1 || forceSingleCycle) && singleCycleComplete) { return; }
    currentStep++;
    int effectiveSeqLen = is16Bit ? wordLen : (wordLen * 2);
    if (currentStep >= effectiveSeqLen) { currentStep = 0; }
}

uint8_t WordGenerator::computeOutputByte(uint16_t word, int startBit) {
    uint8_t result = 0;
    for (int i = 0; i < 8; i++) { if (word & (1 << (startBit + i))) { result |= (1 << i); } }
    return result;
}

uint32_t WordGenerator::generatePRN(uint32_t state) {
    uint32_t bit = ((state >> 0) ^ (state >> 1) ^ (state >> 21) ^ (state >> 31)) & 1;
    return (state >> 1) | (bit << 31);
}

struct WordGeneratorWidget : ModuleWidget {
    WordGeneratorWidget(WordGenerator* module) {
        setModule(module);
        setPanel(createPanel(asset::plugin(pluginInstance, "res/WordGenerator.svg")));
        
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        
        // TOP SECTION - REGROUPED
        // Area 1: Clock Control (Rate Knob, Mode Switch, Rate Attenuverter, Rate CV)
        addParam(createParamCentered<RoundLargeBlackKnob>(mm2px(Vec(20, 20)), module, WordGenerator::CLOCK_RATE_PARAM));
        addParam(createParamCentered<Trimpot>(mm2px(Vec(35, 20)), module, WordGenerator::RATE_ATTEN_PARAM)); // Use a smaller knob (trimpot)
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(50, 20)), module, WordGenerator::RATE_CV_INPUT));      // Moved to y=20
        addParam(createParamCentered<CKSS>(mm2px(Vec(65, 20)), module, WordGenerator::CLOCK_MODE_PARAM));

        // Area 2: Sequence Control (Single Cycle Button + Trig In, Run Mode Switch)
        addParam(createParamCentered<BefacoPush>(mm2px(Vec(20, 40)), module, WordGenerator::SINGLE_CYCLE_BUTTON_PARAM));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(35, 40)), module, WordGenerator::MANUAL_TRIGGER_INPUT));
        addParam(createParamCentered<CKSS>(mm2px(Vec(50, 40)), module, WordGenerator::RUN_MODE_PARAM));

        // Area 3: Word Param Control (Word Length Knob + CV, Mode Switch + Trig In)
        auto wordLengthKnob = createParamCentered<RoundBlackKnob>(mm2px(Vec(105, 20)), module, WordGenerator::WORD_LENGTH_PARAM);
        wordLengthKnob->snap = true;
        addParam(wordLengthKnob);
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(120, 20)), module, WordGenerator::WORD_LENGTH_CV_INPUT));
        addParam(createParamCentered<CKSS>(mm2px(Vec(105, 40)), module, WordGenerator::MODE_PARAM)); // 16/32 bit mode
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(120, 40)), module, WordGenerator::MODE_FLIP_TRIGGER_INPUT));

        // Area 4: PRN Control (PRN Prob Knob + CV)
        addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(20, 60)), module, WordGenerator::PRN_PROB_PARAM));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(35, 60)), module, WordGenerator::PRN_PROBABILITY_INPUT));
        
        // Area 5 & 6: General Inputs & Outputs (CLK IN, RESET, CLK OUT, FIRST, LAST) - Reverted to previous layout
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(65, 60)), module, WordGenerator::CLOCK_INPUT));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(85, 60)), module, WordGenerator::RESET_INPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(105, 60)), module, WordGenerator::CLOCK_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(125, 60)), module, WordGenerator::FIRST_BIT_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(145, 60)), module, WordGenerator::LAST_BIT_OUTPUT));
        
        // Area 7: Main Outputs (Word A CV/Gate, Word B CV/Gate) - Right Column
        // Word A Outputs
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(160, 30)), module, WordGenerator::WORD_A_CV_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(175, 30)), module, WordGenerator::WORD_A_GATE_OUTPUT));
        // Word B Outputs
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(160, 50)), module, WordGenerator::WORD_B_CV_OUTPUT));
        addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(175, 50)), module, WordGenerator::WORD_B_GATE_OUTPUT));

        // BOTTOM SECTION: WORD A (Controls and Bits)
        // Invert A Switch & Flip Trigger Jack
        addParam(createParamCentered<CKSS>(mm2px(Vec(15, 93)), module, WordGenerator::INVERT_A_PARAM));
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(40, 93)), module, WordGenerator::INVERT_A_FLIP_TRIGGER_INPUT));
        
        float bitSwitchStartX_A = 60.f;
        float bitSwitchY_A = 96.f;     // Adjusted Y 
        float bitLightY_A = 98.5f;     // Adjusted Y for lights (2.5mm below switch center, moved up by 2)
        float bitSwitchSpacingX = 8.f;
        for (int i = 0; i < 16; i++) {
            addParam(createParamCentered<CKSS>(mm2px(Vec(bitSwitchStartX_A + i * bitSwitchSpacingX, bitSwitchY_A)), module, WordGenerator::BIT_A_PARAMS + i));
            addChild(createLightCentered<SmallLight<GreenLight>>(mm2px(Vec(bitSwitchStartX_A + i * bitSwitchSpacingX, bitLightY_A)), module, WordGenerator::BIT_A_LIGHTS + i));
        }
        
        // BOTTOM SECTION: WORD B (Controls and Bits)
        // Invert B Switch & Flip Trigger Jack
        addParam(createParamCentered<CKSS>(mm2px(Vec(15, 115)), module, WordGenerator::INVERT_B_PARAM)); // Adjusted Y
        addInput(createInputCentered<PJ301MPort>(mm2px(Vec(40, 115)), module, WordGenerator::INVERT_B_FLIP_TRIGGER_INPUT)); // Adjusted Y

        float bitSwitchStartX_B = 60.f;
        float bitSwitchY_B = 118.f;     // Adjusted Y 
        float bitLightY_B = 120.5f;     // Adjusted Y for lights (2.5mm below switch center)
        for (int i = 0; i < 16; i++) {
            addParam(createParamCentered<CKSS>(mm2px(Vec(bitSwitchStartX_B + i * bitSwitchSpacingX, bitSwitchY_B)), module, WordGenerator::BIT_B_PARAMS + i));
            addChild(createLightCentered<SmallLight<GreenLight>>(mm2px(Vec(bitSwitchStartX_B + i * bitSwitchSpacingX, bitLightY_B)), module, WordGenerator::BIT_B_LIGHTS + i));
        }
    }
};

Model* modelWordGenerator = createModel<WordGenerator, WordGeneratorWidget>("WordGenerator"); 