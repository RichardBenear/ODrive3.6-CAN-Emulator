#include <Arduino.h>
#include <driver/twai.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CAN_TX_PIN 17
#define CAN_RX_PIN 18
#define CAN_BAUDRATE 500000

#define NUM_AXIS 2

#define ODRIVE_NODE_ID_ALT     0
#define ODRIVE_NODE_ID_AZM     1

#define ALT_INJECT_ERRORS      1
#define AZM_INJECT_ERRORS      1

// simulated errors, once cleared by master, they stay cleared
#define ALT_MOTOR_ERROR        0x00000001
#define ALT_ENCODER_ERROR      0x00000001
#define ALT_CONTROLLER_ERROR   0x00000002

#define AZM_MOTOR_ERROR        0x00000002
#define AZM_ENCODER_ERROR      0x00000002
#define AZM_CONTROLLER_ERROR   0x00000004

#define AXIS_ERROR_INVALID_STATE 0x00000001

#define CMD_GET_VERSION        0x00
#define CMD_HEARTBEAT          0x01
#define CMD_ESTOP              0x02
#define CMD_GET_MOTOR_ERROR    0x03
#define CMD_GET_ENCODER_ERROR  0x04
#define CMD_SET_AXIS_NODE_ID   0x06
#define CMD_SET_AXIS_STATE     0x07
#define CMD_GET_ENCODER        0x09
#define CMD_GET_COUNT          0x0A
#define CMD_SET_CONTROLLER     0x0B
#define CMD_SET_INPUT_POS      0x0C
#define CMD_SET_INPUT_VEL      0x0D
#define CMD_SET_INPUT_TORQUE   0x0E
#define CMD_SET_LIMITS         0x0F
#define CMD_GET_IQ             0x14
#define CMD_GET_TEMP           0x15
#define CMD_REBOOT             0x16
#define CMD_GET_BUS_VI         0x17
#define CMD_CLEAR_ERRORS       0x18
#define CMD_SET_LINEAR_COUNT   0x19
#define CMD_SET_POS_GAIN       0x1A
#define CMD_SET_VEL_GAINS      0x1B
#define CMD_GET_CONTROLLER_ERROR 0x1D
#define CMD_SET_NOTCH          0x1E

#define AXIS_STATE_UNDEFINED                  0x00
#define AXIS_STATE_IDLE                       0x01
#define AXIS_STATE_STARTUP                    0x02
#define AXIS_STATE_FULL_CALIBRATION           0x03
#define AXIS_STATE_MOTOR_CALIBRATION          0x04
#define AXIS_STATE_ENCODER_INDEX_SEARCH       0x05
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 0x06
#define AXIS_STATE_CLOSED_LOOP_CONTROL        0x08
#define AXIS_STATE_LOCKIN_SPIN                0x09
#define AXIS_STATE_ENCODER_HALL               0x0A
#define AXIS_STATE_ERROR                      0x0D

typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    bool enabled;
} NotchFilterState;

typedef struct {
    float b0, b1, b2, a1, a2;
    bool valid;
} PendingNotchCoeffs;

typedef struct {
    float rigid_position;
    float flex_position;
    float flex_velocity;
    float wn_rad;
    float damping;
    float drive_gain;
} ResonanceSimState;

typedef struct {
    float position;           // measured position reported over CAN
    float velocity;           // measured velocity reported over CAN
    float target_position;    // raw commanded target from SET_INPUT_POS
    uint32_t axis_state;
    uint32_t axis_error;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t controller_error;
    bool trajectory_done;
    float iq_measured;
    float iq_setpoint;
    float fet_temp;
    float motor_temp;
    float bus_voltage;
    float bus_current;
    float vel_gain;
    float vel_int_gain;

    NotchFilterState notch;
    PendingNotchCoeffs pending_notch;
    ResonanceSimState sim;
} ODriveAxisState;

ODriveAxisState axis[NUM_AXIS];

unsigned long lastHeartbeat = 0;
unsigned long lastEncoderUpdate = 0;
unsigned long lastTxDiagMs = 0;
unsigned long lastSimUpdate = 0;

const int HEARTBEAT_INTERVAL_MS = 200;
const int ENCODER_UPDATE_INTERVAL_MS = 200;
const int CAN_TX_TIMEOUT_MS = 4;

const float SIM_SLEW_RATE_TURNS_PER_SEC = 0.01f;
const float SIM_POSITION_TOLERANCE_TURNS = 0.00006f; // about 79 arcsec
const float SIM_RES_FREQ_ALT_HZ = 4.5f;
const float SIM_RES_FREQ_AZM_HZ = 6.5f;
const float SIM_RES_DAMPING = 0.08f;
const float SIM_RES_DRIVE_GAIN = 12.0f;

const float ALT_MIN_TURNS = 0.0f;
const float ALT_MAX_TURNS = 0.25f;

const float AZM_MIN_TURNS = -0.5f;
const float AZM_MAX_TURNS = 0.5f;

struct TxCounters {
    uint32_t heartbeat_sent = 0;
    uint32_t heartbeat_failed = 0;
    uint32_t encoder_sent = 0;
    uint32_t encoder_failed = 0;
};

TxCounters txCounters[NUM_AXIS];

constexpr bool kVerboseCommandLog = true;

static void applyInjectedErrors(uint8_t node_id) {
    if (node_id >= NUM_AXIS) return;

    uint32_t motorError = 0;
    uint32_t encoderError = 0;
    uint32_t controllerError = 0;
    bool injectErrors = false;

    if (node_id == ODRIVE_NODE_ID_ALT) {
        injectErrors = (ALT_INJECT_ERRORS != 0);
        motorError = ALT_MOTOR_ERROR;
        encoderError = ALT_ENCODER_ERROR;
        controllerError = ALT_CONTROLLER_ERROR;
    } else if (node_id == ODRIVE_NODE_ID_AZM) {
        injectErrors = (AZM_INJECT_ERRORS != 0);
        motorError = AZM_MOTOR_ERROR;
        encoderError = AZM_ENCODER_ERROR;
        controllerError = AZM_CONTROLLER_ERROR;
    }

    if (!injectErrors) {
        return;
    }

    axis[node_id].motor_error = motorError;
    axis[node_id].encoder_error = encoderError;
    axis[node_id].controller_error = controllerError;

    if (motorError != 0 || encoderError != 0 || controllerError != 0) {
        axis[node_id].axis_error = AXIS_ERROR_INVALID_STATE;
        axis[node_id].axis_state = AXIS_STATE_ERROR;
    }
}

static void resetNotchState(NotchFilterState& nf) {
    nf.x1 = 0.0f;
    nf.x2 = 0.0f;
    nf.y1 = 0.0f;
    nf.y2 = 0.0f;
}

static void setDefaultNotch(NotchFilterState& nf) {
    nf.b0 = 1.0f;
    nf.b1 = 0.0f;
    nf.b2 = 0.0f;
    nf.a1 = 0.0f;
    nf.a2 = 0.0f;
    nf.enabled = false;
    resetNotchState(nf);
}

static float applyNotchFilter(NotchFilterState& nf, float x) {
    if (!nf.enabled) {
        return x;
    }

    const float y =
        nf.b0 * x +
        nf.b1 * nf.x1 +
        nf.b2 * nf.x2 -
        nf.a1 * nf.y1 -
        nf.a2 * nf.y2;

    nf.x2 = nf.x1;
    nf.x1 = x;
    nf.y2 = nf.y1;
    nf.y1 = y;

    return y;
}

void maybePrintTxDiagnostics(unsigned long now) {
    if (now - lastTxDiagMs < 1000) return;
    lastTxDiagMs = now;

    for (uint8_t i = 0; i < NUM_AXIS; i++) {
        if (txCounters[i].heartbeat_failed == 0 && txCounters[i].encoder_failed == 0) {
            continue;
        }
        Serial.printf("TX stats node=%u hb sent=%lu fail=%lu enc sent=%lu fail=%lu\n",
                      static_cast<unsigned>(i),
                      static_cast<unsigned long>(txCounters[i].heartbeat_sent),
                      static_cast<unsigned long>(txCounters[i].heartbeat_failed),
                      static_cast<unsigned long>(txCounters[i].encoder_sent),
                      static_cast<unsigned long>(txCounters[i].encoder_failed));
    }
}

void initAxis() {
    for (int i = 0; i < NUM_AXIS; i++) {
        axis[i].position = 0.0f;
        axis[i].velocity = 0.0f;
        axis[i].target_position = 0.0f;
        axis[i].axis_state = AXIS_STATE_IDLE;
        axis[i].axis_error = 0;
        axis[i].motor_error = 0;
        axis[i].encoder_error = 0;
        axis[i].controller_error = 0;
        axis[i].trajectory_done = true;
        axis[i].iq_measured = 0.0f;
        axis[i].iq_setpoint = 0.0f;
        axis[i].fet_temp = 25.0f;
        axis[i].motor_temp = 25.0f;
        axis[i].bus_voltage = 24.0f;
        axis[i].bus_current = 0.0f;
        axis[i].vel_gain = 12.5f;
        axis[i].vel_int_gain = 24.0f;

        setDefaultNotch(axis[i].notch);

        axis[i].pending_notch.b0 = 1.0f;
        axis[i].pending_notch.b1 = 0.0f;
        axis[i].pending_notch.b2 = 0.0f;
        axis[i].pending_notch.a1 = 0.0f;
        axis[i].pending_notch.a2 = 0.0f;
        axis[i].pending_notch.valid = false;

        axis[i].sim.rigid_position = 0.0f;
        axis[i].sim.flex_position = 0.0f;
        axis[i].sim.flex_velocity = 0.0f;
        axis[i].sim.wn_rad = (i == ODRIVE_NODE_ID_ALT)
            ? (2.0f * static_cast<float>(M_PI) * SIM_RES_FREQ_ALT_HZ)
            : (2.0f * static_cast<float>(M_PI) * SIM_RES_FREQ_AZM_HZ);
        axis[i].sim.damping = SIM_RES_DAMPING;
        axis[i].sim.drive_gain = SIM_RES_DRIVE_GAIN;

        applyInjectedErrors(i);
    }

    axis[ODRIVE_NODE_ID_ALT].position = 0.0f;
    axis[ODRIVE_NODE_ID_ALT].target_position = 0.0f;
    axis[ODRIVE_NODE_ID_ALT].sim.rigid_position = 0.0f;

    axis[ODRIVE_NODE_ID_AZM].position = 0.0f;
    axis[ODRIVE_NODE_ID_AZM].target_position = 0.0f;
    axis[ODRIVE_NODE_ID_AZM].sim.rigid_position = 0.0f;
}

bool sendCanMessage(uint32_t can_id, const uint8_t* data, size_t len) {
    if (len > 8) len = 8;

    twai_message_t msg = {};
    msg.identifier = can_id;
    msg.data_length_code = static_cast<uint8_t>(len);
    msg.flags = 0;

    if (data && len) {
        memcpy(msg.data, data, len);
    }

    return twai_transmit(&msg, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) == ESP_OK;
}

void sendHeartbeat(uint8_t node_id) {
    uint8_t data[8] = {0};
    const uint8_t motorFlags = (axis[node_id].motor_error != 0) ? 1 : 0;
    const uint8_t encoderFlags = (axis[node_id].encoder_error != 0) ? 1 : 0;
    uint8_t controllerFlags = (axis[node_id].controller_error != 0) ? 1 : 0;

    if (axis[node_id].trajectory_done) {
        controllerFlags |= 0x80;
    }

    memcpy(data, &axis[node_id].axis_error, 4);
    data[4] = static_cast<uint8_t>(axis[node_id].axis_state);
    data[5] = motorFlags;
    data[6] = encoderFlags;
    data[7] = controllerFlags;

    uint32_t can_id = (node_id << 5) | CMD_HEARTBEAT;
    bool ok = sendCanMessage(can_id, data, 8);
    txCounters[node_id].heartbeat_sent++;
    if (!ok) {
        txCounters[node_id].heartbeat_failed++;
    }
}

void sendEncoderEstimate(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].position, 4);
    memcpy(data + 4, &axis[node_id].velocity, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_ENCODER;
    bool ok = sendCanMessage(can_id, data, 8);
    txCounters[node_id].encoder_sent++;
    if (!ok) {
        txCounters[node_id].encoder_failed++;
    }
}

void sendMotorErrors(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].motor_error, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_MOTOR_ERROR;
    sendCanMessage(can_id, data, 8);

    if (kVerboseCommandLog) {
        Serial.printf(" Motor errors: node=%d, motor_err=0x%08X\n",
                      node_id, axis[node_id].motor_error);
    }
}

void sendEncoderErrors(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].encoder_error, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_ENCODER_ERROR;
    sendCanMessage(can_id, data, 8);

    if (kVerboseCommandLog) {
        Serial.printf(" Encoder errors: node=%d, encoder_err=0x%08X\n",
                      node_id, axis[node_id].encoder_error);
    }
}

void sendControllerErrors(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].controller_error, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_CONTROLLER_ERROR;
    sendCanMessage(can_id, data, 8);

    if (kVerboseCommandLog) {
        Serial.printf(" Controller errors: node=%d, controller_err=0x%08X\n",
                      node_id, axis[node_id].controller_error);
    }
}

void sendIq(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].iq_setpoint, 4);
    memcpy(data + 4, &axis[node_id].iq_measured, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_IQ;
    sendCanMessage(can_id, data, 8);
}

void sendTemperature(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].fet_temp, 4);
    memcpy(data + 4, &axis[node_id].motor_temp, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_TEMP;
    sendCanMessage(can_id, data, 8);
}

void sendBusVI(uint8_t node_id) {
    uint8_t data[8] = {0};

    memcpy(data, &axis[node_id].bus_voltage, 4);
    memcpy(data + 4, &axis[node_id].bus_current, 4);

    uint32_t can_id = (node_id << 5) | CMD_GET_BUS_VI;
    sendCanMessage(can_id, data, 8);

    if (kVerboseCommandLog) {
        Serial.printf(" Bus V/I: node=%d, V=%.2f, I=%.2f\n",
                      node_id, axis[node_id].bus_voltage, axis[node_id].bus_current);
    }
}

void handleSetAxisState(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    uint32_t requested_state;
    memcpy(&requested_state, data, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetAxisState: node=%d, state=0x%02lX\n",
                      node_id, static_cast<unsigned long>(requested_state));
    }

    axis[node_id].axis_state = requested_state;

    if (requested_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
        if (kVerboseCommandLog) {
            Serial.printf(" Node %d entered CLOSED_LOOP_CONTROL\n", node_id);
        }
    } else if (requested_state == AXIS_STATE_IDLE) {
        axis[node_id].velocity = 0.0f;
        axis[node_id].target_position = axis[node_id].position;
        axis[node_id].iq_setpoint = 0.0f;
        axis[node_id].bus_current = 0.0f;
        if (kVerboseCommandLog) {
            Serial.printf(" Node %d entered IDLE\n", node_id);
        }
    } else if (requested_state == AXIS_STATE_FULL_CALIBRATION) {
        if (kVerboseCommandLog) {
            Serial.printf(" Node %d starting calibration...\n", node_id);
        }
    }
}

void handleSetInputPos(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float motorPosition;
    int16_t vel_ff;
    int16_t curr_ff;

    memcpy(&motorPosition, data, 4);
    memcpy(&vel_ff, data + 4, 2);
    memcpy(&curr_ff, data + 6, 2);

    axis[node_id].axis_error &= ~0x00000002;
    axis[node_id].target_position = motorPosition;
    axis[node_id].trajectory_done = false;

    if (kVerboseCommandLog) {
        Serial.printf(" SetInputPos: node=%d, pos=%.4f, vel_ff=%d, curr_ff=%d\n",
                      node_id, motorPosition, vel_ff, curr_ff);
    }
}

void handleSetInputVel(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float velocity;
    float curr_ff;

    memcpy(&velocity, data, 4);
    memcpy(&curr_ff, data + 4, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetInputVel: node=%d, vel=%.2f, curr_ff=%.2f\n",
                      node_id, velocity, curr_ff);
    }
}

void handleSetInputTorque(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float torque;
    memcpy(&torque, data, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetInputTorque: node=%d, torque=%.2f\n", node_id, torque);
    }
}

void handleSetLimits(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float vel_limit;
    float curr_limit;

    memcpy(&vel_limit, data, 4);
    memcpy(&curr_limit, data + 4, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetLimits: node=%d, vel_limit=%.2f, curr_limit=%.2f\n",
                      node_id, vel_limit, curr_limit);
    }
}

void handleSetControllerModes(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    uint32_t control_mode;
    uint32_t input_mode;

    memcpy(&control_mode, data, 4);
    memcpy(&input_mode, data + 4, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetControllerModes: node=%d, control=%lu, input=%lu\n",
                      node_id,
                      static_cast<unsigned long>(control_mode),
                      static_cast<unsigned long>(input_mode));
    }
}

void handleSetPosGain(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float gain;
    memcpy(&gain, data, 4);

    if (kVerboseCommandLog) {
        Serial.printf(" SetPosGain: node=%d, gain=%.2f\n", node_id, gain);
    }
}

void handleSetVelGains(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    float vel_gain;
    float vel_int_gain;

    memcpy(&vel_gain, data, 4);
    memcpy(&vel_int_gain, data + 4, 4);

    axis[node_id].vel_gain = vel_gain;
    axis[node_id].vel_int_gain = vel_int_gain;

    if (kVerboseCommandLog) {
        Serial.printf(" SetVelGains: node=%d, vel_gain=%.4f, vel_int=%.4f\n",
                      node_id, vel_gain, vel_int_gain);
    }
}

void handleClearErrors(uint8_t node_id) {
    if (node_id >= NUM_AXIS) return;

    axis[node_id].axis_error = 0;
    axis[node_id].motor_error = 0;
    axis[node_id].encoder_error = 0;
    axis[node_id].controller_error = 0;
    axis[node_id].trajectory_done = true;
    axis[node_id].axis_state = AXIS_STATE_IDLE;

    if (kVerboseCommandLog) {
        Serial.printf(" ClearErrors: node=%d\n", node_id);
    }
}

void handleEstop(uint8_t node_id) {
    if (node_id >= NUM_AXIS) return;

    axis[node_id].axis_state = AXIS_STATE_ERROR;
    axis[node_id].axis_error |= 0x00000001;

    if (kVerboseCommandLog) {
        Serial.printf(" ESTOP triggered: node=%d\n", node_id);
    }
}

void handleSetLinearCount(uint8_t node_id, uint8_t* data) {
    if (node_id >= NUM_AXIS) return;

    int32_t count;
    memcpy(&count, data, 4);

    float newPos = static_cast<float>(count);

    axis[node_id].position = newPos;
    axis[node_id].target_position = newPos;
    axis[node_id].velocity = 0.0f;

    axis[node_id].sim.rigid_position = newPos;
    axis[node_id].sim.flex_position = 0.0f;
    axis[node_id].sim.flex_velocity = 0.0f;
    resetNotchState(axis[node_id].notch);

    if (kVerboseCommandLog) {
        Serial.printf(" SetLinearCount: node=%d, count=%d pos=%.4f\n", node_id, count, newPos);
    }
}

void handleSetNotch(uint8_t node_id, uint8_t* data, uint8_t len) {
    if (node_id >= NUM_AXIS || len < 5) return;

    const uint8_t index = data[0];
    float value = 0.0f;
    memcpy(&value, data + 1, sizeof(float));

    switch (index) {
        case 0:
            axis[node_id].pending_notch.b0 = value;
            axis[node_id].pending_notch.valid = true;
            break;
        case 1:
            axis[node_id].pending_notch.b1 = value;
            break;
        case 2:
            axis[node_id].pending_notch.b2 = value;
            break;
        case 3:
            axis[node_id].pending_notch.a1 = value;
            break;
        case 4:
            axis[node_id].pending_notch.a2 = value;
            break;
        case 5: {
            const bool enable = (value != 0.0f);

            if (axis[node_id].pending_notch.valid) {
                axis[node_id].notch.b0 = axis[node_id].pending_notch.b0;
                axis[node_id].notch.b1 = axis[node_id].pending_notch.b1;
                axis[node_id].notch.b2 = axis[node_id].pending_notch.b2;
                axis[node_id].notch.a1 = axis[node_id].pending_notch.a1;
                axis[node_id].notch.a2 = axis[node_id].pending_notch.a2;
            }

            axis[node_id].notch.enabled = enable;
            resetNotchState(axis[node_id].notch);

            if (kVerboseCommandLog) {
                Serial.printf(
                    " SetNotch: node=%d enable=%d b0=%.7f b1=%.7f b2=%.7f a1=%.7f a2=%.7f\n",
                    node_id,
                    enable ? 1 : 0,
                    axis[node_id].notch.b0,
                    axis[node_id].notch.b1,
                    axis[node_id].notch.b2,
                    axis[node_id].notch.a1,
                    axis[node_id].notch.a2
                );
            }
            break;
        }
        default:
            if (kVerboseCommandLog) {
                Serial.printf(" SetNotch: node=%d unknown index=%u\n",
                              node_id, static_cast<unsigned>(index));
            }
            break;
    }
}

void processCanMessage(twai_message_t* msg) {
    uint32_t can_id = msg->identifier;
    uint8_t node_id = (can_id >> 5) & 0x1F;
    uint8_t cmd_id = can_id & 0x1F;

    if (node_id >= NUM_AXIS) {
        Serial.printf(" Invalid node ID: %d\n", node_id);
        return;
    }

    switch (cmd_id) {
        case CMD_SET_AXIS_STATE:
            if (msg->data_length_code >= 4) {
                handleSetAxisState(node_id, msg->data);
            }
            break;

        case CMD_SET_INPUT_POS:
            if (msg->data_length_code >= 8) {
                handleSetInputPos(node_id, msg->data);
            }
            break;

        case CMD_SET_INPUT_VEL:
            if (msg->data_length_code >= 8) {
                handleSetInputVel(node_id, msg->data);
            }
            break;

        case CMD_SET_INPUT_TORQUE:
            if (msg->data_length_code >= 4) {
                handleSetInputTorque(node_id, msg->data);
            }
            break;

        case CMD_SET_LIMITS:
            if (msg->data_length_code >= 8) {
                handleSetLimits(node_id, msg->data);
            }
            break;

        case CMD_SET_CONTROLLER:
            if (msg->data_length_code >= 8) {
                handleSetControllerModes(node_id, msg->data);
            }
            break;

        case CMD_SET_POS_GAIN:
            if (msg->data_length_code >= 4) {
                handleSetPosGain(node_id, msg->data);
            }
            break;

        case CMD_SET_VEL_GAINS:
            if (msg->data_length_code >= 8) {
                handleSetVelGains(node_id, msg->data);
            }
            break;

        case CMD_CLEAR_ERRORS:
            handleClearErrors(node_id);
            break;

        case CMD_GET_MOTOR_ERROR:
            sendMotorErrors(node_id);
            break;

        case CMD_GET_ENCODER_ERROR:
            sendEncoderErrors(node_id);
            break;

        case CMD_GET_CONTROLLER_ERROR:
            sendControllerErrors(node_id);
            break;

        case CMD_ESTOP:
            handleEstop(node_id);
            break;

        case CMD_GET_ENCODER:
            sendEncoderEstimate(node_id);
            break;

        case CMD_GET_IQ:
            sendIq(node_id);
            break;

        case CMD_GET_TEMP:
            sendTemperature(node_id);
            break;

        case CMD_GET_BUS_VI:
            sendBusVI(node_id);
            break;

        case CMD_SET_LINEAR_COUNT:
            if (msg->data_length_code >= 4) {
                handleSetLinearCount(node_id, msg->data);
            }
            break;

        case CMD_SET_NOTCH:
            if (msg->data_length_code >= 5) {
                handleSetNotch(node_id, msg->data, msg->data_length_code);
            }
            break;

        default:
            Serial.printf(" Unknown command: 0x%02X\n", cmd_id);
            break;
    }
}

void updateSimulatedMotor(uint8_t node_id, float dtSec) {
    if (node_id >= NUM_AXIS) return;

    if (axis[node_id].axis_state != AXIS_STATE_CLOSED_LOOP_CONTROL) {
        axis[node_id].velocity = 0.0f;
        axis[node_id].iq_setpoint = 0.0f;
        axis[node_id].bus_current = 0.0f;
        return;
    }

    if (dtSec <= 0.0f) {
        axis[node_id].velocity = 0.0f;
        return;
    }

    ResonanceSimState& sim = axis[node_id].sim;

    const float rawTarget = axis[node_id].target_position;
    const float filteredTarget = applyNotchFilter(axis[node_id].notch, rawTarget);

    const float rigidDiff = filteredTarget - sim.rigid_position;
    float maxStep = SIM_SLEW_RATE_TURNS_PER_SEC * dtSec;
    if (maxStep < 0.0f) maxStep = 0.0f;

    float rigidStep = rigidDiff;
    if (rigidStep > maxStep) rigidStep = maxStep;
    if (rigidStep < -maxStep) rigidStep = -maxStep;

    const float prevMeasuredPos = axis[node_id].position;

    sim.rigid_position += rigidStep;

    const float drive = sim.drive_gain * (filteredTarget - sim.rigid_position);
    const float flexAcc =
        drive
        - (2.0f * sim.damping * sim.wn_rad * sim.flex_velocity)
        - (sim.wn_rad * sim.wn_rad * sim.flex_position);

    sim.flex_velocity += flexAcc * dtSec;
    sim.flex_position += sim.flex_velocity * dtSec;

    axis[node_id].position = sim.rigid_position + sim.flex_position;
    axis[node_id].velocity = (axis[node_id].position - prevMeasuredPos) / dtSec;

    const float trackingErr = filteredTarget - axis[node_id].position;

    axis[node_id].iq_setpoint = fabsf(trackingErr) * 40.0f + fabsf(axis[node_id].velocity) * 5.0f;
    axis[node_id].iq_measured = axis[node_id].iq_setpoint + (random(-15, 16) / 100.0f);
    if (axis[node_id].iq_measured < 0.0f) axis[node_id].iq_measured = 0.0f;
    axis[node_id].bus_current = axis[node_id].iq_measured * 0.2f;

    if (fabsf(trackingErr) <= SIM_POSITION_TOLERANCE_TURNS &&
        fabsf(axis[node_id].velocity) < 1.0e-5f) {
        axis[node_id].iq_setpoint = 0.0f;
        axis[node_id].bus_current = 0.0f;
        axis[node_id].trajectory_done = true;
    }
}

void processCanRx() {
    twai_message_t msg;
    while (twai_receive(&msg, pdMS_TO_TICKS(0)) == ESP_OK) {
        processCanMessage(&msg);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    randomSeed(micros());

    Serial.println("\n=== ODrive 3.6 Emulator Starting ===\n");

    initAxis();

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        static_cast<gpio_num_t>(CAN_TX_PIN),
        static_cast<gpio_num_t>(CAN_RX_PIN),
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 20;
    g_config.rx_queue_len = 30;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            Serial.println("CAN initialized at 500kbps");
            Serial.println("Emulating 2 axis: ALT(0), AZM(1)");
            Serial.println("Waiting for CAN commands...\n");
        } else {
            Serial.println("Failed to start CAN driver");
        }
    } else {
        Serial.println("Failed to install CAN driver");
    }

    lastHeartbeat = millis();
    lastEncoderUpdate = millis();
}

void loop() {
    unsigned long now = millis();

    processCanRx();

    float dtSec = 0.0f;
    if (lastSimUpdate != 0) {
        dtSec = (now - lastSimUpdate) / 1000.0f;
    }
    lastSimUpdate = now;

    for (int i = 0; i < NUM_AXIS; i++) {
        updateSimulatedMotor(i, dtSec);
    }

    if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        for (int i = 0; i < NUM_AXIS; i++) {
            sendHeartbeat(i);
        }
        lastHeartbeat = now;
    }

    if (now - lastEncoderUpdate >= ENCODER_UPDATE_INTERVAL_MS) {
        for (int i = 0; i < NUM_AXIS; i++) {
            if (axis[i].axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
                sendEncoderEstimate(i);
            }
        }
        lastEncoderUpdate = now;
    }

    maybePrintTxDiagnostics(now);

    delay(1);
}
