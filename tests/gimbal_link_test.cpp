#include "Gimbal.h"
#include "Shoot.h"
#include "Shoot_Config.h"
#include "dji_motor.h"
#include "RobotCmd.h"
#include "board_transport.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "dmmotor.h"
#include "gimbal_input.h"
#include "input_state.h"
#include "message_center.h"
#include "referee_26.h"
#include "remote_gimbal.h"
#include "sbus.h"
#include "transport_protocol.h"
#include "vision.h"
#include "gimbal_tuning.h"
#include "bsp_usb.h"
#include "tuning_vectors.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

Struct_DJIMotor_Init_Config dji_config[9];
Struct_DJIMotor_Motion_Snapshot dji_snapshot[9];
bool dji_enabled[9];
float dji_reference[9];
uint32_t test_primask;
UART_HandleTypeDef huart1, huart2, huart3, huart5, huart10;
FDCAN_HandleTypeDef hfdcan1{1}, hfdcan2{2}, hfdcan3{3};
Struct_DMMotor_Snapshot test_motors[256];
float test_torque[256];
bool test_enabled[256];
uint64_t clock_us = 1000000U;
uint32_t clock_ms = 100U; // Intentionally different origins, like the real clocks.
std::vector<Struct_CAN_Tx_Msg> queue;
std::vector<uint8_t> transmitted;
bool submit_ok = true;
uint8_t transmit_status = HAL_OK;
uint16_t robot_id;
Struct_SBUS_Frame sbus;
referee_keyboard_snapshot_t keyboard;
using Receiver = void (*)(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
Receiver upper_rx;
Receiver lower_rx;
Receiver lower_yaw_rx;
std::vector<Struct_CAN_Tx_Msg> periodic;
bool periodic_ok = true;
UART_Callback vision_rx;
USB_Callback tuning_rx;
std::vector<uint8_t> usb_transmitted;
uint8_t usb_status = USBD_OK;
unsigned usb_attempts;

void USB_Init(USB_Callback callback)
{
    tuning_rx = callback;
}
uint8_t USB_Transmit_Data(uint8_t* bytes, uint16_t size)
{
    ++usb_attempts;
    if (usb_status == USBD_OK)
        usb_transmitted.assign(bytes, bytes + size);
    return usb_status;
}

bool Lower_BoardTransport_Init();
void Lower_BoardTransport_Poll();
bool BoardTransport_SendGimbalInput(const GimbalRateInput& input);

extern "C" uint32_t HAL_GetTick()
{
    return clock_ms;
}
extern "C" uint64_t SYS_Timestamp_Get_Microsecond()
{
    return clock_us;
}
int HAL_UART_Init(UART_HandleTypeDef*)
{
    return HAL_OK;
}
void UART_Init(UART_HandleTypeDef*, UART_Callback)
{
}
bool UART_InitInterruptRx(UART_HandleTypeDef*, UART_Callback cb)
{
    vision_rx = cb;
    return true;
}
uint8_t UART_Transmit_Nonblocking(UART_HandleTypeDef*, uint8_t* bytes, uint16_t size)
{
    if (transmit_status == HAL_OK)
        transmitted.assign(bytes, bytes + size);
    return transmit_status;
}
bool BSP_CAN_RegisterCallback(uint32_t id, FDCAN_HandleTypeDef* bus, Receiver receive, void*)
{
    if (bus == &hfdcan3 && id == GIMBAL_LINK_YAW_STATE_ID)
        lower_yaw_rx = receive;
    else
        (bus == &hfdcan2 ? upper_rx : lower_rx) = receive;
    return true;
}
bool CAN_Tx_Submit(Struct_CAN_Tx_Msg* frame)
{
    if (submit_ok)
        queue.push_back(*frame);
    return submit_ok;
}
bool CAN_Tx_Perform(Struct_CAN_Tx_Msg* frame)
{
    if (periodic_ok)
        periodic.push_back(*frame);
    return periodic_ok;
}
extern "C" bool SBUS_Init(UART_HandleTypeDef*)
{
    return true;
}
extern "C" bool SBUS_ReadLatest(Struct_SBUS_Frame* frame)
{
    *frame = sbus;
    return true;
}
extern "C" uint8_t RefereeReadKeyboardSnapshot(referee_keyboard_snapshot_t* state)
{
    *state = keyboard;
    return keyboard.received;
}
extern "C" uint8_t RefereeReadRobotId(uint16_t* id)
{
    *id = robot_id;
    return robot_id != 0U;
}

void Advance(uint32_t ms)
{
    clock_ms += ms;
    clock_us += static_cast<uint64_t>(ms) * 1000U;
}

void Tick()
{
    GimbalInput_Update();
    RobotCmd_Update();
    Gimbal_Update();
    Shoot_Update();
    GimbalInput_SendMouseCommand();
}

void Deliver(uint8_t* bytes)
{
    upper_rx(&hfdcan2, 0x80, bytes, 8U, nullptr);
}

void Near(float actual, float expected, float tolerance = 1.0e-5f)
{
    assert(std::fabs(actual - expected) < tolerance);
}

float WireFloat(const uint8_t* bytes)
{
    const uint32_t bits = static_cast<uint32_t>(bytes[0]) << 24U |
                          static_cast<uint32_t>(bytes[1]) << 16U |
                          static_cast<uint32_t>(bytes[2]) << 8U | bytes[3];
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
}

std::array<uint8_t, 24> VisionFrame()
{
    // Independent fixture from the supplied protocol: yaw .5, wire pitch -.25, distance 1.25.
    return {0xFF, 1, 0x3F, 0, 0, 0, 0xBE, 0x80, 0, 0, 0x3F, 0xA0, 0, 0,
            3, 0x3E, 0, 0, 0, 1, 0, 0, 0, 0x0D};
}

void TestVision()
{
    Struct_Vision_Reference reference;
    reference.target_id = 99U;
    assert(!Vision_ReadLatest(reference) && reference.target_id == 99U);
    assert(Vision_Init(&huart1) && huart1.Init.BaudRate == 921600U);
    auto frame = VisionFrame();
    frame[20] = 0xFF; // Marker bytes in a payload are not delimiters.
    frame[21] = 0x0D;
    for (unsigned i = 0U; i < 23U; ++i)
        vision_rx(frame.data() + i, 1U);
    assert(!Vision_ReadLatest(reference));
    vision_rx(frame.data() + 23, 1U);
    assert(Vision_ReadLatest(reference) && reference.valid && reference.fire_advice);
    Near(reference.yaw_rad, .5f);
    Near(reference.pitch_rad, .25f);
    Near(reference.distance_m, 1.25f);
    Near(reference.target_spin_rad_s, .125f);
    assert(reference.target_id == 3U);
    auto sequence = reference.sequence;
    const uint32_t original_time = reference.received_ms;
    Advance(5);
    assert(Vision_ReadLatest(reference) && reference.received_ms == original_time);
    auto malformed = VisionFrame();
    malformed[23] = 0;
    Vision_Receive(malformed.data(), malformed.size());
    for (unsigned offset : {2U, 6U, 10U, 15U})
    {
        auto nan = VisionFrame();
        nan[offset] = 0x7F;
        nan[offset + 1] = 0xC0;
        Vision_Receive(nan.data(), nan.size());
    }
    assert(Vision_ReadLatest(reference) && reference.sequence == sequence);
    std::array<uint8_t, 48> joined{};
    frame = VisionFrame();
    std::memcpy(joined.data(), frame.data(), 24);
    frame[19] = 0;
    std::memcpy(joined.data() + 24, frame.data(), 24);
    Vision_Receive(joined.data(), joined.size());
    assert(Vision_ReadLatest(reference) && reference.sequence == sequence + 2U);
    assert(!reference.valid && !reference.fire_advice && reference.distance_m == -1.0f);
    assert(reference.yaw_rad == 0 && reference.pitch_rad == 0);
    for (uint8_t distance_high : {uint8_t{0}, uint8_t{0xBF}})
    {
        frame = VisionFrame();
        frame[10] = distance_high;
        frame[11] = distance_high == 0 ? 0 : 0x80;
        Vision_Receive(frame.data(), frame.size());
        assert(Vision_ReadLatest(reference) && !reference.valid);
    }
    // Every possible split of the valid frame must recover after malformed traffic.
    for (unsigned split = 0; split <= 24; ++split)
    {
        frame = VisionFrame();
        Vision_Receive(frame.data(), split);
        Vision_Receive(frame.data() + split, 24 - split);
        assert(Vision_ReadLatest(reference) && reference.valid);
    }

    Struct_Vision_Feedback feedback;
    feedback.mode = 1;
    feedback.roll_rad = -.125f;
    feedback.pitch_rad = -.25f;
    feedback.yaw_rad = 1;
    feedback.bullet_speed_m_s = 16;
    feedback.sampled_ms = 0x12345678U;
    assert(Vision_SendFeedback(feedback) == HAL_OK);
    const std::vector<uint8_t> expected{0xFF, 1, 0xBE, 0, 0, 0, 0x3E, 0x80, 0, 0,
                                        0x3F, 0x80, 0, 0, 0x41, 0x80, 0, 0, 0x12, 0x34, 0x56, 0x78, 0, 0x0D};
    assert(transmitted == expected);
    transmit_status = HAL_BUSY;
    assert(Vision_SendFeedback(feedback) == HAL_BUSY && transmitted == expected);
    transmit_status = HAL_OK;
    feedback.yaw_rad = std::numeric_limits<float>::infinity();
    assert(Vision_SendFeedback(feedback) == HAL_ERROR && transmitted == expected);
    feedback.yaw_rad = 0;
    feedback.mode = 6;
    assert(Vision_SendFeedback(feedback) == HAL_ERROR && transmitted == expected);
    feedback.mode = 5;
    feedback.sampled_ms = 0;
    assert(Vision_SendFeedback(feedback) == HAL_OK);
    for (unsigned i = 18; i < 22; ++i)
        assert(transmitted[i] == 0);
    std::cout << "PASS 24-byte protocol fixtures, all splits, markers, invalid floats/targets, recovery and feedback\n";
}

void TestLinkAndControl()
{
    assert(BoardTransport_Init() && Lower_BoardTransport_Init() && RemoteGimbal_Init());
    assert(Gimbal_Default_Config().yaw.control_enabled);
    assert(Gimbal_Default_Config().yaw.motor.bus == &hfdcan2);
    assert(Gimbal_Default_Config().pitch.motor.bus == &hfdcan1);
    auto config = Gimbal_Default_Config();
    config.yaw.moving_friction = 0; // Isolate state feedback for exact integration assertions.
    config.pitch.static_friction = config.pitch.moving_friction = config.pitch.gravity_constant = config.pitch.gravity_linear = 0;
    assert(Gimbal_Init(config));
    GimbalInput_Init();
    static LocalPublisher<GimbalCmd> gimbal(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot(MessageCenter::Shoot_Command_Topic);
    assert(RobotCmd_Init(gimbal.Bind(), chassis.Bind(), shoot.Bind(), true));
    sbus.timestamp_ms = clock_ms;
    sbus.channels[0] = 392;
    sbus.channels[1] = -392;
    sbus.channels[7] = 1401 - SBUS_CHANNEL_OFFSET;
    RemoteGimbal_Update();
    assert(queue.size() == 1 && queue[0].hfdcan == &hfdcan3 && queue[0].id == 0x80);
    const uint8_t expected[] = {0, 0x19, 0xDC, 0x05, 0x24, 0xFA, 0, 0};
    assert(std::memcmp(queue[0].data, expected, 8) == 0);
    Deliver(queue[0].data);
    uint8_t feedback_bytes[8];
    assert(TransportProtocol::EncodeChassisFeedback(ChassisFeedback{}, 250, feedback_bytes));
    Deliver(feedback_bytes); // Back-to-back same CAN identifier, separate pending slots.
    BoardTransport_Poll();
    auto rate = MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta();
    assert(rate.valid && MessageCenter::Chassis_Feedback_Topic.ReadWithMeta().valid);
    Near(rate.data.yaw_speed_rad_s, 1.5f);
    Near(rate.data.pitch_speed_rad_s, -1.5f);
    const uint64_t received_at = rate.timestamp_us;
    const uint32_t rate_sequence = rate.sequence;
    INS_State ins{};
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Tick(); // Request enable before ready can become true.
    assert(test_enabled[0x61] && test_enabled[0x60]);
    Tick(); // Independent ready capture, not integration.
    for (unsigned i = 0; i < 5; ++i)
    {
        Advance(1);
        Tick();
    }
    Near(test_torque[0x61], config.yaw.k_angle * 0.0075f + config.yaw.k_speed * 1.5f);
    Near(test_torque[0x60], config.pitch.k_angle * -0.0075f + config.pitch.k_speed * -1.5f);
    // Repeated CAN sequence and feedback traffic do not refresh the command age.
    Deliver(queue[0].data);
    Deliver(feedback_bytes);
    BoardTransport_Poll();
    rate = MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta();
    assert(rate.sequence == rate_sequence && rate.timestamp_us == received_at);

    GimbalRateInput zero{};
    zero.enabled = true;
    uint8_t bytes[8];
    assert(TransportProtocol::EncodeGimbalRateInput(zero, 1U, bytes));
    Deliver(bytes);
    BoardTransport_Poll();
    Tick();
    Near(test_torque[0x61], config.yaw.k_angle * 0.0075f);
    const float hold_torque = test_torque[0x61];
    Advance(1);
    Tick();
    Near(test_torque[0x61], hold_torque);

    // Pitch position constraint stops outward integration; reverse works at once.
    test_motors[0x60].feedback.position = config.pitch_max;
    GimbalRateInput outward{0, 1.0f, true};
    assert(TransportProtocol::EncodeGimbalRateInput(outward, 2U, bytes));
    Deliver(bytes);
    BoardTransport_Poll();
    Tick();
    assert(test_torque[0x60] <= 0.0f);
    outward.pitch_speed_rad_s = -1.0f;
    assert(TransportProtocol::EncodeGimbalRateInput(outward, 3U, bytes));
    Deliver(bytes);
    BoardTransport_Poll();
    Tick();
    assert(test_torque[0x60] < 0.0f);

    test_motors[0x61].online = false;
    Tick(); // An offline yaw does not prevent pitch control.
    assert(test_torque[0x60] < 0.0f);
    test_motors[0x61].online = true;
    ins.yaw_rad = 0.5f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Tick();
    Near(test_torque[0x61], 0.0f); // Recovery captures current attitude.

    Advance(51);
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Tick();
    assert(!test_enabled[0x61] && !test_enabled[0x60]);
    queue.clear();
    RemoteGimbal_Update();
    assert(queue.size() == 1 && queue[0].data[1] == 0x18);
    for (unsigned i = 2; i < 8; ++i)
        assert(queue[0].data[i] == 0);

    // Queue rejection must not consume the lower sender's independent sequence.
    const uint8_t last_sequence = queue.back().data[0];
    submit_ok = false;
    assert(!BoardTransport_SendGimbalInput(zero));
    submit_ok = true;
    assert(BoardTransport_SendGimbalInput(zero));
    assert(queue.back().data[0] == static_cast<uint8_t>(last_sequence + 1U));
    assert(GimbalLink_SequenceNewer(0, 255) && !GimbalLink_SequenceNewer(255, 0));
    Advance(101);
    for (uint8_t value : {254U, 255U, 0U})
    {
        assert(TransportProtocol::EncodeGimbalRateInput(zero, value, bytes));
        Deliver(bytes);
        BoardTransport_Poll();
        assert(MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta().data.enabled);
    }
    const auto wrapped = MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta();
    for (unsigned i = 0; i < 11; ++i)
    {
        Advance(10);
        Deliver(bytes);
        BoardTransport_Poll();
    }
    assert(MessageCenter::Remote_Gimbal_Input_Topic.ReadWithMeta().timestamp_us == wrapped.timestamp_us);
    bytes[7] = 1;
    GimbalRateInput output{};
    uint8_t sequence;
    assert(!TransportProtocol::DecodeGimbalRateInput(bytes, 8, output, sequence));

    // Cross +pi while the measured attitude wraps to -pi: no long-way torque.
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    ins = {};
    ins.yaw_rad = 3.141f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd turn;
    turn.mode = GimbalMode::RATE;
    turn.yaw_speed_rad_s = 1.0f;
    MessageCenter::Gimbal_Command_Topic.Publish(turn);
    Gimbal_Update();
    Gimbal_Update();
    ins.yaw_rad -= 6.283185307179586f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x61], config.yaw.k_angle * 0.001f + config.yaw.k_speed, 1.0e-4f);
    // Keep later source tests starting from the actual expired remote state.
    Tick();
    std::cout << "PASS actual lower CAN3 -> upper CAN2 -> RobotCmd -> dual-axis integration/LQR, age, recovery and limits\n";
}

void TestFeedbackIsolation()
{
    keyboard = {};
    GimbalInput_Init();
    transmitted.clear();
    MessageCenter::Gimbal_INS_State_Topic = {};
    Advance(10);
    GimbalInput_Update();
    assert(transmitted.empty());
    INS_State ins{};
    ins.roll_rad = .2f;
    ins.pitch_rad = .1f;
    ins.yaw_rad = .3f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    robot_id = 3;
    Advance(10);
    GimbalInput_Update();
    assert(transmitted.size() == 24 && transmitted[1] == 1);
    const auto first = transmitted;
    Near(WireFloat(first.data() + 2), .2f);
    Near(WireFloat(first.data() + 6), .1f);
    Near(WireFloat(first.data() + 10), .3f);
    Near(WireFloat(first.data() + 14), 16);
    Advance(10);
    GimbalInput_Update();
    assert(transmitted == first); // Repeated sample preserves original timestamp.
    Advance(981);                 // Sample age is now 1001ms.
    transmitted.clear();
    GimbalInput_Update();
    assert(transmitted.empty());

    Struct_Gimbal_Input_Config config;
    config.vision_feedback_period_ms = 25;
    config.vision_feedback_max_age_ms = 0;
    config.vision_bullet_speed_m_s = 17;
    GimbalInput_Init(config);
    Advance(24);
    GimbalInput_Update();
    assert(transmitted.empty());
    Advance(1);
    GimbalInput_Update();
    assert(transmitted.size() == 24);
    Near(WireFloat(transmitted.data() + 14), 17);
    const auto old = transmitted;
    Advance(2000);
    GimbalInput_Update();
    assert(transmitted == old); // Timeout disabled, original sample timestamp retained.
    robot_id = 0;
    Advance(25);
    GimbalInput_Update();
    assert(transmitted[1] == 0);
    assert(!InputState_Read().vision_enabled && MessageCenter::Shoot_Event_Queue.Empty());
    std::cout << "PASS feedback snapshot, configurable period/speed/age, disabled timeout and identity\n";
}

void ResetCommandOwner()
{
    static LocalPublisher<GimbalCmd> gimbal(MessageCenter::Gimbal_Command_Topic);
    static LocalPublisher<ChassisCmd> chassis(MessageCenter::Chassis_Command_Topic);
    static LocalPublisher<ShootCmd> shoot(MessageCenter::Shoot_Command_Topic);
    assert(RobotCmd_Init(gimbal.Bind(), chassis.Bind(), shoot.Bind(), true));
}

void TestEnableSwitch()
{
    ResetCommandOwner();
    GimbalInput_Init(); // Clear the keyboard ownership from the preceding test.
    sbus.channels[0] = 392;
    sbus.channels[1] = -392;
    struct Case
    {
        int16_t raw;
        uint8_t lost;
        uint8_t failsafe;
        uint32_t age_ms;
        bool enabled;
    };
    const Case cases[] = {
        {1399, 0, 0, 0, false},
        {1400, 0, 0, 0, false},
        {1401, 0, 0, 0, true},
        {1399, 0, 0, 0, false}, // Closing after enabling must send zero speeds.
        {1401, 1, 0, 0, false},
        {1401, 0, 1, 0, false},
        {1401, 0, 0, 51, false}};
    for (const auto& item : cases)
    {
        Advance(10);
        sbus.channels[7] = item.raw - SBUS_CHANNEL_OFFSET;
        sbus.frame_lost = item.lost;
        sbus.failsafe = item.failsafe;
        sbus.timestamp_ms = clock_ms - item.age_ms;
        queue.clear();
        RemoteGimbal_Update();
        assert(queue.size() == 1);
        GimbalRateInput rate;
        uint8_t sequence;
        assert(TransportProtocol::DecodeGimbalRateInput(queue[0].data, 8, rate, sequence));
        assert(rate.enabled == item.enabled);
        Near(rate.yaw_speed_rad_s, item.enabled ? 1.5f : 0.0f);
        Near(rate.pitch_speed_rad_s, item.enabled ? -1.5f : 0.0f);
        assert(g_remote_ch8 == item.raw);
        assert(g_remote_gimbal_enabled == static_cast<uint8_t>(item.enabled));
        Deliver(queue[0].data);
        BoardTransport_Poll();
        MessageCenter::Gimbal_INS_State_Topic.Publish(INS_State{});
        Tick();
        const auto command = MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data;
        assert(command.mode == (item.enabled ? GimbalMode::RATE : GimbalMode::DISABLED));
        assert(test_enabled[0x61] == item.enabled && test_enabled[0x60] == item.enabled);
    }
    std::cout << "PASS CH8 raw 1399/1400/1401, switch-off zero speeds, frame loss/failsafe/stale and upper enable\n";
}

// Real lower mapping -> wire codec -> upper transport -> arbitration -> torque output.
void SendAutoAimSwitches(int16_t ch8, int16_t ch7)
{
    Advance(10);
    sbus.channels[7] = ch8 - SBUS_CHANNEL_OFFSET;
    sbus.channels[6] = ch7 - SBUS_CHANNEL_OFFSET;
    sbus.timestamp_ms = clock_ms;
    sbus.frame_lost = sbus.failsafe = 0;
    queue.clear();
    RemoteGimbal_Update();
    assert(queue.size() == 1);
    GimbalRateInput input{};
    uint8_t sequence;
    assert(TransportProtocol::DecodeGimbalRateInput(queue[0].data, 8, input, sequence));
    assert(input.enabled == (ch8 > 1400));
    assert(input.autoaim_requested == (ch8 > 1400 && ch7 > 1400));
    Deliver(queue[0].data);
    BoardTransport_Poll();
    MessageCenter::Gimbal_INS_State_Topic.Publish(INS_State{});
    Tick();
}

void TestAutoAim()
{
    ResetCommandOwner();
    keyboard = {};
    GimbalInput_Init();
    auto config = Gimbal_Default_Config();
    config.yaw.k_angle = config.pitch.k_angle = 1;
    config.yaw.k_speed = config.pitch.k_speed = 0;
    config.yaw.moving_friction = 0;
    config.pitch.static_friction = config.pitch.moving_friction = config.pitch.gravity_constant = config.pitch.gravity_linear = 0;
    test_motors[0x61] = test_motors[0x60] = {};
    test_enabled[0x61] = test_enabled[0x60] = false;
    assert(Gimbal_Init(config));
    sbus.channels[0] = sbus.channels[1] = 0;
    auto frame = VisionFrame();
    for (int16_t ch8 : {1399, 1400, 1401})
    {
        for (int16_t ch7 : {1399, 1400, 1401})
        {
            SendAutoAimSwitches(ch8, ch7);
            Vision_Receive(frame.data(), frame.size());
            Tick();
            Tick();
            auto command = MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data;
            assert(command.mode == (ch8 <= 1400 ? GimbalMode::DISABLED : ch7 > 1400 ? GimbalMode::IMU
                                                                                    : GimbalMode::RATE));
        }
    }
    Near(test_torque[0x61], .5f);
    Near(test_torque[0x60], .25f);
    const auto unchanged = MessageCenter::Gimbal_Command_Topic.ReadWithMeta().sequence;
    for (unsigned i = 0; i < 15; ++i)
        Tick();
    Near(test_torque[0x61], .5f); // Absolute reference is not integrated repeatedly.
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().sequence == unchanged);

    // Driver recovers while command values and RobotCmd publication sequence stay unchanged.
    test_motors[0x61].online = false;
    Tick();
    test_motors[0x61].online = true;
    Tick();
    Near(test_torque[0x61], .5f);
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().sequence == unchanged);

    frame[19] = 0;
    Vision_Receive(frame.data(), frame.size());
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::RATE);
    Near(test_torque[0x61], 0); // Manual handover captures measured angle, not last visual target.
    frame = VisionFrame();
    Vision_Receive(frame.data(), frame.size());
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::IMU);
    assert(MessageCenter::Shoot_Event_Queue.Empty());

    const uint32_t received = InputState_Read().vision.received_ms;
    Advance(1000);
    MessageCenter::Remote_Gimbal_Input_Topic.Publish(GimbalRateInput{0, 0, true, true});
    MessageCenter::Gimbal_INS_State_Topic.Publish(INS_State{});
    Tick();
    assert(InputState_Read().vision.received_ms == received);
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::IMU);
    Advance(1);
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::RATE);

    SendAutoAimSwitches(1401, 1400);
    Vision_Receive(frame.data(), frame.size());
    SendAutoAimSwitches(1401, 1401);
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::RATE); // Old frame.
    Vision_Receive(frame.data(), frame.size());
    Tick();
    SendAutoAimSwitches(1400, 1401);
    assert(!InputState_Read().vision_enabled && !test_enabled[0x61] && !test_enabled[0x60]);
    // Even an inconsistent remote packet cannot use its autoaim bit without enable permission.
    MessageCenter::Remote_Gimbal_Input_Topic.Publish(GimbalRateInput{0, 0, false, true});
    Tick();
    assert(!InputState_Read().vision_enabled);

    SendAutoAimSwitches(1401, 1401);
    Vision_Receive(frame.data(), frame.size());
    Tick();
    keyboard.received = 1;
    keyboard.received_ms = clock_ms;
    keyboard.sequence = 1;
    keyboard.keyboard = (1U << 5U) | (1U << 1U);
    Tick();
    assert(InputState_Read().keyboard_enabled && !InputState_Read().vision_enabled);
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::RATE);
    keyboard.keyboard = 0;
    keyboard.mouse_right = 1;
    for (unsigned i = 0; i <= 10; ++i)
    {
        keyboard.received_ms = clock_ms;
        ++keyboard.sequence;
        Tick();
        if (i < 10) Advance(30);
    }
    Tick(); // Establish keyboard autoaim boundary first.
    Vision_Receive(frame.data(), frame.size());
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::IMU);
    keyboard.mouse_right = 0;
    keyboard.received_ms = clock_ms;
    ++keyboard.sequence;
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::RATE);

    keyboard = {};
    Struct_Gimbal_Input_Config input_config;
    input_config.vision_target_max_age_ms = 0;
    GimbalInput_Init(input_config);
    SendAutoAimSwitches(1401, 1401);
    Vision_Receive(frame.data(), frame.size());
    Tick();
    for (unsigned i = 0; i < 150; ++i)
        SendAutoAimSwitches(1401, 1401);
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::IMU);
    Advance(51); // Remote still has its original timeout, independent of vision timeout being disabled.
    Tick();
    assert(MessageCenter::Gimbal_Command_Topic.ReadWithMeta().data.mode == GimbalMode::DISABLED);

    // Absolute yaw across +/-pi uses the shortest angle.
    INS_State ins{};
    ins.yaw_rad = -3.13f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd absolute{};
    absolute.mode = GimbalMode::IMU;
    absolute.yaw_angle_rad = 3.13f;
    MessageCenter::Gimbal_Command_Topic.Publish(absolute);
    Gimbal_Update();
    Gimbal_Update();
    Near(test_torque[0x61], 6.26f - 6.283185307179586f);
    std::cout << "PASS autoaim CH8/CH7 matrix, absolute/recovery/shortest yaw, stale/manual, keyboard and no fire\n";
}

void TestPitchFeedbackMapping()
{
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update(); // Reset both axes' target capture before this independent case.
    auto config = Gimbal_Default_Config();
    config.yaw.moving_friction = 0;
    config.pitch.static_friction = config.pitch.moving_friction = config.pitch.gravity_constant = config.pitch.gravity_linear = 0;
    test_motors[0x61] = {};
    test_motors[0x60] = {};
    assert(Gimbal_Init(config));
    INS_State ins{};
    ins.roll_rad = 0.25f;
    ins.pitch_rad = -0.1f;
    ins.yaw_rad = 0.3f;
    ins.gyro_x_rad_s = -0.9f;
    test_motors[0x60].feedback.velocity = 0.4f;
    ins.gyro_y_rad_s = 0.7f;
    ins.gyro_z_rad_s = 0.2f;
    test_motors[0x61].feedback.velocity = 0.2f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd hold{};
    hold.mode = GimbalMode::RATE;
    MessageCenter::Gimbal_Command_Topic.Publish(hold);
    for (unsigned i = 0; i < 12; ++i)
        Gimbal_Update();
    const auto feedback = MessageCenter::Gimbal_Feedback_Topic.ReadWithMeta().data;
    Near(feedback.pitch_rad, 0.1f);
    Near(feedback.pitch_speed_rad_s, 0.4f);
    Near(feedback.yaw_rad, 0.3f);
    Near(feedback.yaw_speed_rad_s, 0.2f);
    Near(test_torque[0x60], -config.pitch.k_speed * 0.4f);
    const float yaw_torque = test_torque[0x61];

    ins.roll_rad = -0.55f;
    ins.gyro_y_rad_s = -1.2f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x60], -config.pitch.k_speed * 0.4f);
    ins.pitch_rad = -0.12f; // Raising the measured pitch reduces torque toward the held target.
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x60], -config.pitch.k_angle * 0.02f - config.pitch.k_speed * 0.4f);
    ins.gyro_x_rad_s = 0.3f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x60], -config.pitch.k_angle * 0.02f - config.pitch.k_speed * 0.4f);
    test_motors[0x60].feedback.velocity = -0.3f;
    Gimbal_Update();
    Near(test_torque[0x60], -config.pitch.k_angle * 0.02f + config.pitch.k_speed * 0.3f);
    Near(test_torque[0x61], yaw_torque);
    Advance(15);
    Gimbal_Update();
    assert(test_enabled[0x61] && test_enabled[0x60]);
    Advance(5);
    Gimbal_Update(); // ReadFresh retains its existing inclusive boundary at 20 ms.
    assert(test_enabled[0x61] && test_enabled[0x60]);
    Advance(1);
    MessageCenter::INS_State_Topic.Publish(ins);
    Gimbal_Update();
    assert(!test_enabled[0x61] && !test_enabled[0x60]);
    std::cout << "PASS Pitch inertial angle/motor speed, independent gyros, Yaw isolation and stale IMU gate\n";
}

void TestYawFeedbackMapping()
{
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    auto config = Gimbal_Default_Config();
    config.yaw.motor_per_axis = -2.0f;
    config.yaw.moving_friction = 0;
    config.pitch.static_friction = config.pitch.gravity_constant = config.pitch.gravity_linear = 0;
    test_motors[0x61] = {};
    test_motors[0x60] = {};
    test_motors[0x61].feedback.velocity = .8f;
    test_motors[0x60].feedback.velocity = .1f;
    assert(Gimbal_Init(config));
    INS_State ins{};
    ins.yaw_rad = .3f;
    ins.gyro_z_rad_s = 9.0f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd hold{};
    hold.mode = GimbalMode::RATE;
    MessageCenter::Gimbal_Command_Topic.Publish(hold);
    for (unsigned i = 0; i < 12; ++i)
        Gimbal_Update();
    Near(MessageCenter::Gimbal_Feedback_Topic.ReadWithMeta().data.yaw_speed_rad_s, -.4f);
    Near(test_torque[0x61], config.yaw.k_speed * .4f);
    const float pitch_torque = test_torque[0x60];
    ins.gyro_z_rad_s = -7.0f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x61], config.yaw.k_speed * .4f);
    test_motors[0x61].feedback.velocity = -.6f;
    for (unsigned i = 0; i < 12; ++i)
        Gimbal_Update();
    Near(MessageCenter::Gimbal_Feedback_Topic.ReadWithMeta().data.yaw_speed_rad_s, .3f);
    Near(test_torque[0x61], -config.yaw.k_speed * .3f);
    ins.yaw_rad += .02f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x61], -config.yaw.k_angle * .02f - config.yaw.k_speed * .3f);
    Near(test_torque[0x60], pitch_torque);
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    std::cout << "PASS Yaw motor rate/negative ratio, gyro independence, inertial position and Pitch isolation\n";
}

void TestPitchDefaults()
{
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    const auto config = Gimbal_Default_Config();
    const auto& pitch = config.pitch;
    Near(pitch.k_angle, 9.29481325f);
    Near(pitch.k_speed, 1.99631282f);
    Near(pitch.gravity_constant, .210217638139f);
    Near(pitch.gravity_linear, -.714052927197f);
    Near(pitch.moving_friction, 0);
    Near(pitch.static_friction, .15f);
    Near(config.yaw.k_angle, 9.89562444f);
    Near(config.yaw.k_speed, .469133869f);
    Near(config.yaw.static_friction, 0);
    test_motors[0x60] = {};
    test_motors[0x61] = {};
    test_motors[0x60].feedback.velocity = .2f;
    assert(Gimbal_Init(config));
    Gimbal_Update();
    assert(!test_enabled[0x60] && !test_enabled[0x61]);

    INS_State ins{};
    ins.pitch_rad = -.1f;
    ins.gyro_x_rad_s = 9.0f; // 陀螺值刻意与电机速度不同。
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd hold;
    hold.mode = GimbalMode::RATE;
    MessageCenter::Gimbal_Command_Topic.Publish(hold);
    Gimbal_Update();
    Gimbal_Update();
    constexpr float degree = .017453292519943295f;
    for (float error_deg : {-3.0f, -2.0f, -1.0f, 0.0f, 1.0f, 2.0f, 3.0f})
    {
        const float error = error_deg * degree;
        ins.pitch_rad = -(.1f - error);
        MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
        Gimbal_Update();
        const float expected_assist = .15f * std::fmax(-1.0f, std::fmin(error_deg / 2.0f, 1.0f));
        const float expected = pitch.gravity_constant + pitch.gravity_linear * error +
                               pitch.k_angle * error - pitch.k_speed * .2f + expected_assist;
        Near(test_torque[0x60], expected);
    }

    // 非零参考速度不叠加静态到位辅助。
    hold.pitch_speed_rad_s = .2f;
    MessageCenter::Gimbal_Command_Topic.Publish(hold);
    Gimbal_Update();
    const float error = 3.0f * degree + .0002f;
    Near(test_torque[0x60], pitch.gravity_constant + pitch.gravity_linear * error + pitch.k_angle * error);
    float tuned[10];
    assert(Gimbal_GetTuningParameters(tuned));
    tuned[5] = 3;
    assert(Gimbal_SetTuningParameters(tuned));
    assert(Gimbal_GetTuningParameters(tuned));
    Near(tuned[5], 3);
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    assert(Gimbal_Init(Gimbal_Default_Config()));
    assert(Gimbal_GetTuningParameters(tuned));
    Near(tuned[5], pitch.k_angle);
    Near(tuned[6], pitch.k_speed);
    Gimbal_Update();
    assert(!test_enabled[0x60]);
    auto reversed = config;
    reversed.pitch.motor_per_axis = -2.0f;
    reversed.pitch.static_friction = reversed.pitch.gravity_constant = reversed.pitch.gravity_linear = 0;
    assert(Gimbal_Init(reversed));
    test_motors[0x60].feedback.velocity = .6f;
    hold.pitch_speed_rad_s = 0;
    MessageCenter::Gimbal_Command_Topic.Publish(hold);
    for (unsigned i = 0; i < 12; ++i)
        Gimbal_Update();
    Near(MessageCenter::Gimbal_Feedback_Topic.ReadWithMeta().data.pitch_speed_rad_s, -.3f);
    Near(test_torque[0x60], pitch.k_speed * .3f);
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    std::cout << "PASS Pitch defaults, disabled boot, static assist, moving isolation and reboot/hot tuning\n";
}

void TestTuning()
{
    const auto feed = [](const uint8_t* bytes, unsigned size) {
        std::vector<uint8_t> copy(bytes, bytes + size);
        tuning_rx(copy.data(), static_cast<uint16_t>(size));
        GimbalTuning_TransportUpdate();
    };
    Struct_Gimbal_Tuning_Request request{};
    // Every possible USB split must yield exactly one complete request.
    for (unsigned split = 1; split < sizeof(tuning_write); ++split)
    {
        GimbalTuning_Init();
        feed(tuning_write, split);
        assert(!GimbalTuning_TakeRequest(request));
        feed(tuning_write + split, sizeof(tuning_write) - split);
        assert(GimbalTuning_TakeRequest(request));
        assert(!GimbalTuning_TakeRequest(request));
        assert(request.sequence == 12345 && request.operation == 1);
        assert(Gimbal_SetTuningParameters(request.values));
        float actual[10]{};
        assert(Gimbal_GetTuningParameters(actual));
        GimbalTuning_Reply(request, 0, actual);
        GimbalTuning_TransportUpdate();
        assert(usb_transmitted == std::vector<uint8_t>(std::begin(tuning_write_reply), std::end(tuning_write_reply)));
    }
    std::vector<uint8_t> corrupt(std::begin(tuning_write), std::end(tuning_write));
    corrupt[20] ^= 1;
    feed(corrupt.data(), corrupt.size());
    assert(!GimbalTuning_TakeRequest(request));
    const uint8_t noise[] = {0, 1, 'G', 'T', '0'};
    feed(noise, sizeof(noise));
    feed(tuning_query, sizeof(tuning_query));
    assert(GimbalTuning_TakeRequest(request) && request.operation == 2);
    float before[10]{};
    assert(Gimbal_GetTuningParameters(before));
    GimbalTuning_Reply(request, 0, before);
    GimbalTuning_TransportUpdate();
    assert(usb_transmitted == std::vector<uint8_t>(std::begin(tuning_query_reply), std::end(tuning_query_reply)));

    // Any invalid field rejects the whole group, including a preceding valid Yaw edit.
    for (unsigned field = 0; field < 10; ++field)
    {
        float invalid[10];
        std::memcpy(invalid, before, sizeof(before));
        invalid[0] = 42;
        invalid[field] = std::numeric_limits<float>::quiet_NaN();
        assert(!Gimbal_SetTuningParameters(invalid));
        invalid[field] = std::numeric_limits<float>::infinity();
        assert(!Gimbal_SetTuningParameters(invalid));
        float actual[10];
        assert(Gimbal_GetTuningParameters(actual));
        assert(std::memcmp(actual, before, sizeof(before)) == 0);
    }
    for (unsigned field : {0U, 1U, 4U, 5U, 6U, 9U})
    {
        float invalid[10];
        std::memcpy(invalid, before, sizeof(before));
        invalid[field] = -1;
        assert(!Gimbal_SetTuningParameters(invalid));
    }
    // Failed USB submission is attempted once, without automatic resend.
    usb_status = USBD_BUSY;
    const unsigned attempts = usb_attempts;
    GimbalTuning_Reply(request, 0, before);
    GimbalTuning_TransportUpdate();
    GimbalTuning_TransportUpdate();
    assert(usb_attempts == attempts + 1);
    usb_status = USBD_OK;
    test_primask = 1;
    GimbalTuning_Reply(request, 0, before);
    GimbalTuning_TransportUpdate();
    assert(test_primask == 1);
    test_primask = 0;

    // Hot updates change the next torque while retaining the previously held target.
    MessageCenter::Gimbal_Command_Topic.Publish(GimbalCmd{});
    Gimbal_Update();
    INS_State ins{};
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    GimbalCmd lock{};
    lock.mode = GimbalMode::LOCK;
    MessageCenter::Gimbal_Command_Topic.Publish(lock);
    Gimbal_Update();
    Gimbal_Update();
    ins.yaw_rad = .01f;
    ins.pitch_rad = -.02f;
    ins.gyro_z_rad_s = .03f;
    test_motors[0x61].feedback.velocity = .03f;
    ins.gyro_x_rad_s = -.9f;
    test_motors[0x60].feedback.velocity = .04f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(ins);
    Gimbal_Update();
    Near(test_torque[0x61], -20 * .01f - .03f);
    before[0] = 10;
    before[5] = 15;
    assert(Gimbal_SetTuningParameters(before));
    Gimbal_Update();
    Near(test_torque[0x61], -10 * .01f - .03f);
    Near(test_torque[0x60], -15 * .02f - 2 * .04f + .25f + (-.57f) * (-.02f));
    assert(test_enabled[0x61] && test_enabled[0x60]);
    std::cout << "PASS tuning: Python CRC/endian fixtures, all USB splits, corrupt frames, atomic rejection, no retry and hot target continuity\n";
}

void MouseFrame(uint32_t dt, uint16_t keys, bool left, bool right)
{
    Advance(dt);
    keyboard.received = 1;
    keyboard.received_ms = clock_ms;
    ++keyboard.sequence;
    keyboard.keyboard = keys;
    keyboard.mouse_left = left;
    keyboard.mouse_right = right;
    Tick();
}

uint8_t LastMouse()
{
    assert(!queue.empty());
    const auto& frame = queue.back();
    assert(frame.hfdcan == &hfdcan2 && frame.id == 0x81 && frame.data[1] == 0x1C);
    uint8_t value=255, sequence=0;
    assert(GimbalLink_DecodeMouseCommand(frame.data, 8, &value, &sequence));
    return value;
}

void TestMouseAndFriction()
{
    ResetCommandOwner();
    keyboard = {};
    GimbalInput_Init();
    assert(Shoot_Init());
    assert(dji_config[1].hfdcan == &hfdcan1 && dji_config[2].hfdcan == &hfdcan1);
    assert(!dji_config[1].reverse && dji_config[2].reverse);
    assert(dji_config[8].hfdcan == nullptr);
    Near(dji_config[1].gear_ratio, 1);
    MouseFrame(1, (1U<<5) | (1U<<1), false, false);
    keyboard.mouse_x=10;
    keyboard.mouse_y=20;
    MouseFrame(1, 1U<<9, false, false);
    Near(InputState_Read().keyboard.gimbal.yaw_speed_rad_s, -.06f);
    Near(InputState_Read().keyboard.gimbal.pitch_speed_rad_s, .12f);
    keyboard.mouse_y=-20;
    MouseFrame(1, 1U<<9, false, false);
    Near(InputState_Read().keyboard.gimbal.pitch_speed_rad_s, -.12f);
    Near(InputState_Read().keyboard.gimbal.yaw_speed_rad_s, -.06f);
    keyboard.mouse_y=20;
    assert(dji_enabled[1] && dji_enabled[2]);
    Near(dji_reference[1], 25);
    Near(dji_reference[2], 25);
    MouseFrame(1, 1U<<9, false, false);
    assert(dji_enabled[1]); // Held F toggles once.
    dji_snapshot[1].online=dji_snapshot[2].online=true;
    dji_snapshot[1].output_speed=23.75f;
    dji_snapshot[2].output_speed=26.25f;
    MouseFrame(1, 0, false, false);
    assert(Shoot_FrictionReady()); // Inclusive ±5% boundary.
    dji_snapshot[2].output_speed=26.251f;
    MouseFrame(1, 0, true, false);
    assert(!Shoot_FrictionReady());
    dji_snapshot[2].output_speed=25;
    for (unsigned i=0; i<11; ++i) MouseFrame(30,0,true,false);
    assert(LastMouse()==0); // Press begun before ready is rejected.
    MouseFrame(1,0,false,false);
    MouseFrame(1,0,true,false);
    MouseFrame(50,0,false,false);
    assert(LastMouse()==1);
    queue.clear();
    MouseFrame(1,0,true,false);
    for(unsigned i=0;i<9;++i) MouseFrame(30,0,true,false);
    MouseFrame(29,0,true,false);
    assert(LastMouse()==0);
    MouseFrame(1,0,true,false);
    assert(LastMouse()==2);
    dji_snapshot[1].output_speed=-25;
    MouseFrame(1,0,true,false);
    assert(LastMouse()==0 && !Shoot_FrictionReady());
    dji_snapshot[1].output_speed=25;
    MouseFrame(1,0,true,false);
    assert(LastMouse()==2);
    dji_snapshot[2].online=false;
    MouseFrame(1,0,true,false);
    assert(LastMouse()==0);
    dji_snapshot[2].online=true;
    MouseFrame(1,0,true,false);
    assert(LastMouse()==2);
    MouseFrame(1,0,true,true);
    for(unsigned i=0;i<9;++i) MouseFrame(30,0,true,true);
    MouseFrame(29,0,true,true);
    assert(!InputState_Read().vision_enabled);
    MouseFrame(1,0,true,true);
    assert(InputState_Read().vision_enabled && LastMouse()==0);
    MouseFrame(1,0,true,false);
    assert(!InputState_Read().vision_enabled && LastMouse()==0);
    MouseFrame(1,0,false,false);
    MouseFrame(1,0,true,false);
    for(unsigned i=0;i<10;++i) MouseFrame(30,0,true,false);
    assert(LastMouse()==2);
    Advance(101); Tick();
    assert(LastMouse()==0 && !Shoot_FrictionReady() && !dji_enabled[1]);
    MouseFrame(1,1U<<9,false,false);
    MouseFrame(1,0,false,false);
    MouseFrame(1,1U<<9,false,false);
    assert(!dji_enabled[1]); // F turns off.
    MouseFrame(1,0,false,false);
    MouseFrame(1,1U<<9,false,false);
    MouseFrame(1,(1U<<5)|(1U<<1),false,false);
    assert(!InputState_Read().keyboard_enabled && !dji_enabled[1]);

    queue.clear();
    assert(BoardTransport_SendMouseCommand(1));
    const uint8_t previous=queue.back().data[0];
    submit_ok=false;
    assert(!BoardTransport_SendMouseCommand(1));
    submit_ok=true;
    assert(BoardTransport_SendMouseCommand(0));
    assert(queue.back().data[0]==static_cast<uint8_t>(previous+1));
    uint8_t bytes[8]{};
    assert(!GimbalLink_EncodeMouseCommand(3,0,bytes));
    assert(GimbalLink_EncodeMouseCommand(2,255,bytes));
    assert(bytes[0]==255 && bytes[1]==0x1C && bytes[2]==2 && bytes[7]==0);
    bytes[7]=1;
    uint8_t value=99, seq=99;
    assert(!GimbalLink_DecodeMouseCommand(bytes,8,&value,&seq) && value==99 && seq==99);
    std::cout << "PASS mouse axes, F friction, dual ready boundaries, click/hold, pause/resume, autoaim and frame expiry\n";
}

void TestYawDownlink()
{
    uint8_t bytes[8]{};
    const GimbalLink_YawState source{1.0f, -2.5f};
    const uint8_t expected[8]{0, 0, 0x80, 0x3f, 0, 0, 0x20, 0xc0};
    assert(GimbalLink_EncodeYawState(&source, bytes));
    assert(std::memcmp(bytes, expected, 8) == 0);
    GimbalLink_YawState decoded{};
    assert(GimbalLink_DecodeYawState(bytes, 8, &decoded));
    assert(decoded.yaw_rad == 1 && decoded.yaw_speed_rad_s == -2.5f);
    for (uint32_t length : {0U, 7U, 9U})
        assert(!GimbalLink_DecodeYawState(bytes, length, &decoded));
    assert(!GimbalLink_DecodeYawState(nullptr, 8, &decoded));
    assert(!GimbalLink_DecodeYawState(bytes, 8, nullptr));
    assert(!GimbalLink_EncodeYawState(nullptr, bytes));
    assert(!GimbalLink_EncodeYawState(&source, nullptr));
    for (float invalid : {std::numeric_limits<float>::infinity(),
                          -std::numeric_limits<float>::infinity(),
                          std::numeric_limits<float>::quiet_NaN()})
    {
        for (unsigned field = 0; field < 2; ++field)
        {
            GimbalLink_YawState bad = source;
            (field == 0 ? bad.yaw_rad : bad.yaw_speed_rad_s) = invalid;
            assert(!GimbalLink_EncodeYawState(&bad, bytes));
            assert(std::memcmp(bytes, expected, 8) == 0);
            std::memcpy(bytes + 4 * field, &invalid, 4);
            assert(!GimbalLink_DecodeYawState(bytes, 8, &decoded));
            assert(decoded.yaw_rad == 1 && decoded.yaw_speed_rad_s == -2.5f);
            std::memcpy(bytes, expected, 8);
        }
    }
    assert(!BoardTransport_SendGimbalYawState()); // 尚未初始化。
    assert(BoardTransport_Init() && Lower_BoardTransport_Init());
    assert(lower_rx && lower_yaw_rx);
    assert(!BoardTransport_SendGimbalYawState()); // 外置姿态尚未发布。
    INS_State onboard{};
    onboard.yaw_rad = 2;
    onboard.gyro_z_rad_s = 99;
    MessageCenter::INS_State_Topic.Publish(onboard);
    assert(!BoardTransport_SendGimbalYawState()); // 板载姿态不能替代外置姿态。
    INS_State external{};
    external.yaw_rad = 7.0f;
    external.gyro_z_rad_s = -2.5f;
    MessageCenter::Gimbal_INS_State_Topic.Publish(external);
    const uint64_t sampled_us = clock_us;
    periodic.clear();
    queue.clear();
    for (unsigned tick = 0; tick < 21; ++tick)
    {
        clock_us = sampled_us + tick * 1000U;
        assert(BoardTransport_SendGimbalYawState());
    }
    assert(periodic.size() == 21 && queue.empty()); // 包括恰好20ms，走最新值槽。
    auto frame = periodic.back();
    assert(frame.id == 0x82 && frame.len == 8 && frame.hfdcan == &hfdcan2);
    assert(GimbalLink_DecodeYawState(frame.data, 8, &decoded));
    assert(std::fabs(decoded.yaw_rad - std::remainder(7.0f, 6.283185307179586f)) < 1e-6f);
    assert(decoded.yaw_speed_rad_s == -2.5f);
    ++clock_us;
    assert(!BoardTransport_SendGimbalYawState() && periodic.size() == 21);
    external.yaw_rad = -7;
    MessageCenter::Gimbal_INS_State_Topic.Publish(external);
    periodic_ok = false;
    assert(!BoardTransport_SendGimbalYawState() && periodic.size() == 21);
    periodic_ok = true;
    assert(BoardTransport_SendGimbalYawState());
    frame = periodic.back();
    ChassisCmd command{};
    command.mode = ChassisMode::NO_FOLLOW;
    uint8_t command_bytes[8]{};
    assert(TransportProtocol::EncodeChassisCmd(command, 1, command_bytes));
    const uint64_t received_us = ++clock_us;
    lower_rx(&hfdcan3, 0x81, command_bytes, 8, nullptr);
    lower_yaw_rx(&hfdcan3, 0x82, frame.data, 8, nullptr);
    clock_us += 1000;
    Lower_BoardTransport_Poll();
    auto snapshot = MessageCenter::Gimbal_Yaw_State_Topic.ReadWithMeta();
    assert(snapshot.valid && snapshot.timestamp_us == received_us);
    assert(std::fabs(snapshot.data.yaw_rad - std::remainder(-7.0f, 6.283185307179586f)) < 1e-6f);
    assert(snapshot.data.yaw_speed_rad_s == -2.5f);
    assert(MessageCenter::Chassis_Command_Topic.ReadWithMeta().timestamp_us == received_us);
    lower_yaw_rx(&hfdcan2, 0x82, frame.data, 8, nullptr);
    lower_yaw_rx(&hfdcan3, 0x81, frame.data, 8, nullptr);
    lower_yaw_rx(&hfdcan3, 0x82, frame.data, 7, nullptr);
    Lower_BoardTransport_Poll();
    assert(MessageCenter::Gimbal_Yaw_State_Topic.Sequence() == snapshot.sequence);
    float invalid = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(frame.data, &invalid, 4);
    lower_yaw_rx(&hfdcan3, 0x82, frame.data, 8, nullptr);
    Lower_BoardTransport_Poll();
    assert(MessageCenter::Gimbal_Yaw_State_Topic.Sequence() == snapshot.sequence);
    assert(MessageCenter::Gimbal_Yaw_State_Topic.Timestamp() == received_us);
    std::cout << "PASS 0x82 float fixtures, invalid frames, external gyro, 1ms updates, 20ms boundary and isolated receive timestamps\n";
}

int main()
{
    TestVision();
    float initial[10]{};
    assert(!Gimbal_GetTuningParameters(initial));
    assert(!Gimbal_SetTuningParameters(initial));
    TestYawDownlink();
    TestLinkAndControl();
    TestFeedbackIsolation();
    TestEnableSwitch();
    TestAutoAim();
    TestPitchFeedbackMapping();
    TestTuning();
    TestPitchDefaults();
    TestYawFeedbackMapping();
    TestMouseAndFriction();
    assert(test_primask == 0);
    std::cout << "All host integration checks passed (hardware calls stubbed).\n";
}
