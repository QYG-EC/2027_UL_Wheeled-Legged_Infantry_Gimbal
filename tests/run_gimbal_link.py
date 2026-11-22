#!/usr/bin/env python3
"""Host regression of real upper/lower transport, input, control and 24-byte vision parser.

Hardware calls are replaced with deterministic stubs; no target is accessed.
"""

import argparse
from pathlib import Path
import re
import subprocess
import struct
import zlib

ROOT = Path(__file__).resolve().parents[1]

def check_tuning_model():
    """Independently solve the two-state Riccati equation without MATLAB/SciPy."""
    source = (ROOT / 'sysid/gimbal_tune.m').read_text()
    def matrix(name):
        body = re.search(rf'{name} = \[([^\]]+)\]', source).group(1)
        return [[float(x) for x in row.replace(',', ' ').split()] for row in body.split(';')]
    def multiply(a, b):
        return [[sum(x * y for x, y in zip(row, column)) for column in zip(*b)] for row in a]
    def transpose(a):
        return list(map(list, zip(*a)))
    for axis, weights, expected in [('yaw', [220, .8], [14.5215994, .934219941]),
                                     ('pitch', [1090, .8], [27.6671015, .536113934])]:
        a, b = matrix(axis + '_Ad'), matrix(axis + '_Bd')
        if axis == 'pitch': expected=matrix('pitch_reference_K')[0]
        q = [[weights[0], 0], [0, weights[1]]]
        p = [row[:] for row in q]
        for _ in range(20000):
            btp = multiply(transpose(b), p)
            divisor = 1 + multiply(btp, b)[0][0]
            k = [[v / divisor for v in multiply(btp, a)[0]]]
            atp = multiply(transpose(a), p)
            apa = multiply(atp, a)
            correction = multiply(multiply(atp, b), k)
            next_p = [[q[i][j] + apa[i][j] - correction[i][j] for j in range(2)] for i in range(2)]
            change = max(abs(next_p[i][j] - p[i][j]) for i in range(2) for j in range(2))
            p = next_p
            if change < 1e-9:
                break
        else:
            raise AssertionError('Riccati iteration did not converge')
        assert all(abs(x - y) < 1e-7 for x, y in zip(k[0], expected)), (axis, k, expected)
    print('PASS independent two-axis discrete gain calculation from MATLAB model constants', flush=True)

check_tuning_model()
parser = argparse.ArgumentParser()
parser.add_argument("--lower", type=Path, default=ROOT.parent / "2027_UL_Wheeled-Legged_Infantry_Chassis")
args = parser.parse_args()
lower = args.lower.resolve()
build = ROOT / "build/gimbal-link-tests"
stubs = build / "stubs"
stubs.mkdir(parents=True, exist_ok=True)

headers = {
    "bsp_usb.h": """
#pragma once
#include "stm32h7xx.h"
#define USB_BUFFER_SIZE 512
enum { USBD_OK, USBD_BUSY, USBD_FAIL };
using USB_Callback = void (*)(uint8_t*, uint16_t);
void USB_Init(USB_Callback);
uint8_t USB_Transmit_Data(uint8_t*, uint16_t);
""",
    "stm32h7xx.h": """
#pragma once
#include <cstdint>
extern uint32_t test_primask;
inline uint32_t __get_PRIMASK() { return test_primask; }
inline void __disable_irq() { test_primask = 1; }
inline void __set_PRIMASK(uint32_t v) { test_primask = v; }
inline void __DMB() {}
extern "C" uint32_t HAL_GetTick();
""",
    "usart.h": """
#pragma once
#include "stm32h7xx.h"
struct UART_HandleTypeDef { struct { uint32_t BaudRate; } Init{}; };
extern UART_HandleTypeDef huart1, huart2, huart3, huart5, huart10;
enum { HAL_OK, HAL_ERROR, HAL_BUSY };
int HAL_UART_Init(UART_HandleTypeDef*);
""",
    "fdcan.h": """
#pragma once
#include "stm32h7xx.h"
struct FDCAN_HandleTypeDef { int id; };
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
""",
    "bsp_uart.h": """
#pragma once
#include "usart.h"
using UART_Callback = void (*)(uint8_t*, uint16_t);
void UART_Init(UART_HandleTypeDef*, UART_Callback);
bool UART_InitInterruptRx(UART_HandleTypeDef*, UART_Callback);
uint8_t UART_Transmit_Nonblocking(UART_HandleTypeDef*, uint8_t*, uint16_t);
""",
    "bsp_can.h": """
#pragma once
#include "fdcan.h"
struct Struct_CAN_Tx_Msg { FDCAN_HandleTypeDef* hfdcan; uint32_t id; uint8_t len; uint8_t data[8]; };
using CAN_RxCallback_t = void (*)(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
bool BSP_CAN_RegisterCallback(uint32_t, FDCAN_HandleTypeDef*, CAN_RxCallback_t, void*);
bool CAN_Tx_Submit(Struct_CAN_Tx_Msg*);
bool CAN_Tx_Perform(Struct_CAN_Tx_Msg*);
""",
    "daemon.h": """
#pragma once
#include <cstdint>
class Daemon { public: explicit Daemon(uint32_t) {} void Feed() {} bool IsOnline() const { return true; }
uint32_t OfflineDurationMs() const { return 0; } };
class DaemonManager { public: static bool Register(Daemon&) { return true; } };
""",
    "sys_timestamp.h": """
#pragma once
#include <cstdint>
extern "C" uint64_t SYS_Timestamp_Get_Microsecond();
""",
    "dji_motor.h": """
#pragma once
#include "fdcan.h"
struct PID_InitTypeDef { float K_P, K_I, K_D, I_Out_Max, Out_Max, D_T; };
enum class Enum_DJIMotor_Type { M3508 };
enum Enum_DJIMotor_Loop { DJI_MOTOR_CURRENT_LOOP=1, DJI_MOTOR_SPEED_LOOP=2, DJI_MOTOR_ANGLE_LOOP=4 };
struct Struct_DJIMotor_Init_Config {
FDCAN_HandleTypeDef* hfdcan; uint8_t can_id; Enum_DJIMotor_Type motor_type; float gear_ratio=0;
int close_loop, outer_loop; bool reverse=false; PID_InitTypeDef current_pid{}, speed_pid{}, angle_pid{};
};
struct Struct_DJIMotor_Motion_Snapshot { float output_total_angle=0, output_speed=0; bool ready=false, online=false; };
extern Struct_DJIMotor_Init_Config dji_config[9];
extern Struct_DJIMotor_Motion_Snapshot dji_snapshot[9];
extern bool dji_enabled[9];
extern float dji_reference[9];
class Class_DJIMotor { uint8_t id=0; public:
bool Init(const Struct_DJIMotor_Init_Config& c) { id=c.can_id; dji_config[id]=c; return true; }
bool RequestEnabled(bool e) { dji_enabled[id]=e; return true; }
Struct_DJIMotor_Motion_Snapshot GetMotionSnapshot() { auto s=dji_snapshot[id]; s.ready=s.online && dji_enabled[id]; return s; }
void Set_Outer_Loop(Enum_DJIMotor_Loop) {}
void Control(float value) { dji_reference[id]=value; }
};
class Class_DJIMotor_Group { Class_DJIMotor* a=nullptr; Class_DJIMotor* b=nullptr; public:
bool Init(Class_DJIMotor* x, Class_DJIMotor* y=nullptr) { a=x; b=y; return true; }
bool RequestEnabled(bool e) { a->RequestEnabled(e); if(b) b->RequestEnabled(e); return true; }
void Control(float x, float y=0) { a->Control(x); if(b) b->Control(y); }
};
""",
    "dmmotor.h": """
#pragma once
#include "fdcan.h"
enum class Enum_DMMotor_Mode { MIT };
struct Struct_DMMotor_Snapshot { struct { float position = 0; float velocity = 0; } feedback; bool ready = false;
bool online = true; bool fault = false; };
extern Struct_DMMotor_Snapshot test_motors[256];
extern float test_torque[256];
extern bool test_enabled[256];
class Class_DMMotor { uint8_t id = 0; public:
void SetFeedbackStateCodes(uint8_t, uint8_t) {}
bool Init(FDCAN_HandleTypeDef*, uint8_t n, uint16_t, Enum_DMMotor_Mode, bool, float, float, float) { id = n; return true; }
bool RequestEnabled(bool enabled) { test_enabled[id] = enabled; return true; }
Struct_DMMotor_Snapshot GetFeedbackSnapshot() const { auto s = test_motors[id]; s.ready = s.online && test_enabled[id]; return s; }
bool SetTorque(float torque) { test_torque[id] = torque; return true; }
};
""",
}
for name, source in headers.items():
    (stubs / name).write_text(source)

# Independent wire fixtures: Python struct/zlib vs the actual C++ codec.
values = [20, 1, 0, 0, 0, 30, 2, .25, -.57, 0]
def tuning_frame(operation, data):
    payload = struct.pack('<4sBBHI10f', b'GT01', operation, 0, 0, 12345, *data)
    return payload + struct.pack('<I', zlib.crc32(payload))

fixtures = {'tuning_write': tuning_frame(1, values),
            'tuning_query': tuning_frame(2, [0] * 10),
            'tuning_write_reply': tuning_frame(0x81, values),
            'tuning_query_reply': tuning_frame(0x82, values)}
(stubs / 'tuning_vectors.h').write_text('\n'.join(
    f'const uint8_t {name}[] = {{{",".join(str(b) for b in data)}}};'
    for name, data in fixtures.items()))

includes = [
    stubs,
    ROOT / "User_Config/Board",
    ROOT / "User_File/System/MessageCenter",
    ROOT / "User_File/System/Transport",
    ROOT / "User_File/Application/Input",
    ROOT / "User_File/Application/Shoot",
    ROOT / "User_File/Application/Gimbal",
    ROOT / "User_File/Application/RobotCmd",
    ROOT / "User_File/Device/Peripheral/Referee",
    ROOT / "User_File/Device/Peripheral/Remote",
    ROOT / "User_File/Device/Peripheral/Vision",
    ROOT / "User_File/Device/Peripheral/EricTool",
    lower / "User_File/Application/Input",
]
flags = ["-std=c++17", "-Wall", "-Wextra", "-Werror", "-g", "-fsanitize=undefined,address"]
include_flags = [f"-I{path}" for path in includes]
objects = []

def compile_source(path, extra=()):
    obj = build / f"source-{len(objects)}.o"
    subprocess.run(["g++", *flags, *extra, *include_flags, "-c", str(path), "-o", str(obj)], check=True)
    objects.append(obj)

for rel in [
    "User_Config/Board/gimbal_board_config.cpp",
    "User_File/System/MessageCenter/message_center.cpp",
    "User_File/System/Transport/transport_protocol.cpp",
    "User_File/System/Transport/transport_gimbal.cpp",
    "User_File/Application/Input/input_state.cpp",
    "User_File/Application/Input/source_arbitration.cpp",
    "User_File/Application/Input/gimbal_input.cpp",
    "User_File/Application/Shoot/Shoot.cpp",
    "User_File/Application/RobotCmd/RobotCmd.cpp",
    "User_File/Application/Gimbal/Gimbal.cpp",
    "User_File/Device/Peripheral/Vision/vision.cpp",
    "User_File/Device/Peripheral/EricTool/gimbal_tuning.cpp",
    "tests/gimbal_link_test.cpp",
]:
    compile_source(ROOT / rel)

# The two firmware targets expose the same transport entry points. Rename only
# the lower object's symbols so both real implementations can run in one test.
renames = [f"-D{name}=Lower_{name}" for name in [
    "BoardTransport_Init", "BoardTransport_IsOnline", "BoardTransport_OfflineDurationMs",
    "BoardTransport_Poll", "TransportConfig_Bus",
]]
for rel in ["User_File/System/Transport/transport_chassis.cpp", "User_File/Application/Input/remote_gimbal.cpp"]:
    compile_source(lower / rel, [f"-I{lower / 'User_File/System/Transport'}", *renames])

codec = ROOT / "User_File/System/Transport/gimbal_link_protocol.c"
# Upper adds type 3; lower remains intentionally unchanged in this task.
obj = build / "codec.o"
subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined,address", "-c", str(codec), "-o", str(obj)], check=True)
exe = build / "gimbal-link-test"
subprocess.run(["g++", *flags, *map(str, objects), str(obj), "-o", str(exe)], check=True)
subprocess.run([str(exe)], check=True)

# Exercise the actual lower BSP initialization function with captured HAL calls.
# Isolating this function keeps unrelated RTOS/ARM assembly out of the host test.
bsp = (lower / "User_File/Middleware/BSP/CAN/bsp_can.c").read_text()
function = re.search(r"static void BSP_CAN_ConfigBus\(FDCAN_HandleTypeDef \*hfdcan\)\s*\{.*?\n\}", bsp, re.S)
assert function, "Cannot locate lower CAN initialization function"
hal = (lower / "Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_hal_fdcan.h").read_text()
constants = []
for name in ["FDCAN_STANDARD_ID", "FDCAN_FILTER_DUAL", "FDCAN_FILTER_MASK", "FDCAN_FILTER_TO_RXFIFO0",
             "FDCAN_FILTER_REJECT", "FDCAN_ACCEPT_IN_RX_FIFO0", "FDCAN_REJECT", "FDCAN_REJECT_REMOTE"]:
    value = re.search(rf"#define {name}\s+\(\(uint32_t\)(0x[0-9A-Fa-f]+U)\)", hal)
    assert value, f"Cannot read HAL constant {name}"
    constants.append(f"#define {name} {value[1]}")
filter_test = build / "can-filter-test.c"
filter_test.write_text("""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define HAL_OK 0
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
typedef struct { unsigned number; } FDCAN_HandleTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2; } FDCAN_FilterTypeDef;
FDCAN_HandleTypeDef hfdcan1 = {1}, hfdcan2 = {2}, hfdcan3 = {3};
static FDCAN_FilterTypeDef captured;
static uint32_t standard_action, extended_action;
static unsigned calls;
static bool extended;
static bool BSP_CAN_ExtendedEnabled(FDCAN_HandleTypeDef *bus) { (void)bus; return extended; }
static int HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *bus, const FDCAN_FilterTypeDef *filter)
{ (void)bus; captured = *filter; ++calls; return HAL_OK; }
""" + "\n".join(constants) + """
static int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *bus, uint32_t standard, uint32_t ext, uint32_t rs, uint32_t re)
{ (void)bus; standard_action=standard; extended_action=ext;
  assert(rs==FDCAN_REJECT_REMOTE && re==FDCAN_REJECT_REMOTE); ++calls; return HAL_OK; }
static int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *bus, uint32_t flags, uint32_t buffer)
{ (void)bus; assert(flags==FDCAN_IT_RX_FIFO0_NEW_MESSAGE && buffer==0); ++calls; return HAL_OK; }
static int HAL_FDCAN_Start(FDCAN_HandleTypeDef *bus) { (void)bus; ++calls; return HAL_OK; }
static void Error_Handler(void) { assert(0); }
""" + function[0] + """
int main(void)
{
    FDCAN_HandleTypeDef *buses[] = {&hfdcan1, &hfdcan2, &hfdcan3};
    for (unsigned ext=0; ext<2; ++ext) for (unsigned bus=0; bus<3; ++bus)
    {
        extended=ext; calls=0;
        BSP_CAN_ConfigBus(buses[bus]);
        assert(calls==4 && captured.IdType==FDCAN_STANDARD_ID && captured.FilterIndex==0);
        assert(extended_action==(extended ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_REJECT));
        for (unsigned id=0; id<=0x7FF; ++id)
        {
            const bool match=captured.FilterType==FDCAN_FILTER_DUAL ?
                (id==captured.FilterID1 || id==captured.FilterID2) :
                ((id & captured.FilterID2)==(captured.FilterID1 & captured.FilterID2));
            const bool accepted=match ? captured.FilterConfig==FDCAN_FILTER_TO_RXFIFO0 :
                standard_action==FDCAN_ACCEPT_IN_RX_FIFO0;
            const bool reject=H7_BOARD_CHASSIS && bus==2 && (id==0x61 || id==0x71);
            assert(accepted==!reject);
        }
    }
    puts("PASS actual BSP hardware filter: all 2048 standard IDs, all buses, extended policy and board isolation");
}
""")
for chassis in [0, 1]:
    binary = build / f"can-filter-{chassis}"
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", f"-DH7_BOARD_CHASSIS={chassis}",
                    str(filter_test), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
