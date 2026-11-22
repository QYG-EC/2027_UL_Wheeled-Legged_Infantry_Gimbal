#include "input_state.h"

namespace
{
InputState state;
}

void InputState_Reset()
{
    state = {};
}

void InputState_SetTime(uint32_t now_ms)
{
    state.now_ms = now_ms;
}

void InputState_SubmitRemote(const ControlInput &input)
{
    state.remote = input;
}

void InputState_SubmitVtm(const ControlInput &input)
{
    state.vtm = input;
}

void InputState_SubmitKeyboard(const ControlInput &input)
{
    state.keyboard = input;
}

void InputState_SubmitVision(const VisionAimInput &input)
{
    state.vision = input;
}

void InputState_Select(InputSource source, bool vision_enabled, uint32_t now_ms)
{
    if (source != InputSource::Remote && source != InputSource::Vtm &&
        source != InputSource::Keyboard)
    {
        return;
    }
    if (source != state.selected || vision_enabled != state.vision_enabled)
    {
        state.selected_at_ms = now_ms;
    }
    state.selected = source;
    state.vision_enabled = vision_enabled;
}

InputState InputState_Read()
{
    return state;
}

void InputState_EnableGimbalRateSources(uint32_t vision_max_age_ms)
{
    state.gimbal_rate_sources = true;
    state.vision_max_age_ms = vision_max_age_ms;
}

void InputState_SetKeyboardEnabled(bool enabled)
{
    state.keyboard_enabled = enabled;
}
