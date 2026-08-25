#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS_UBLOX.h"

#include <SITL/SITL.h>

using namespace SITL;

/*
  send a UBLOX GPS message
 */
void GPS_UBlox::send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size, uint8_t msgclass)
{
    const uint8_t PREAMBLE1 = 0xb5;
    const uint8_t PREAMBLE2 = 0x62;
    uint8_t hdr[6], chk[2];
    hdr[0] = PREAMBLE1;
    hdr[1] = PREAMBLE2;
    hdr[2] = msgclass;
    hdr[3] = msgid;
    hdr[4] = size & 0xFF;
    hdr[5] = size >> 8;
    chk[0] = chk[1] = hdr[2];
    chk[1] += (chk[0] += hdr[3]);
    chk[1] += (chk[0] += hdr[4]);
    chk[1] += (chk[0] += hdr[5]);
    for (uint16_t i=0; i<size; i++) {
        chk[1] += (chk[0] += buf[i]);
    }
    write_to_autopilot((char*)hdr, sizeof(hdr));
    write_to_autopilot((char*)buf, size);
    write_to_autopilot((char*)chk, sizeof(chk));
}

/*
  send a new set of GPS UBLOX packets
 */
void GPS_UBlox::publish(const GPS_Data *d)
{
    struct PACKED ubx_nav_posllh {
        uint32_t    time; // GPS msToW
        int32_t     longitude;
        int32_t     latitude;
        int32_t     altitude_ellipsoid;
        int32_t     altitude_msl;
        uint32_t    horizontal_accuracy;
        uint32_t    vertical_accuracy;
    } pos {};
    struct PACKED ubx_nav_status {
        uint32_t    time;               // GPS msToW
        uint8_t     fix_type;
        uint8_t     fix_status;
        uint8_t     differential_status;
        uint8_t     flags2;             // bits 4:3 are spoofDetState
        uint32_t    time_to_first_fix;
        uint32_t    uptime;             // milliseconds
    } status {};
    struct PACKED ubx_nav_velned {
        uint32_t    time;               // GPS msToW
        int32_t     ned_north;
        int32_t     ned_east;
        int32_t     ned_down;
        uint32_t    speed_3d;
        uint32_t    speed_2d;
        int32_t     heading_2d;
        uint32_t    speed_accuracy;
        uint32_t    heading_accuracy;
    } velned {};
    struct PACKED ubx_nav_solution {
        uint32_t time;
        int32_t time_nsec;
        int16_t week;
        uint8_t fix_type;
        uint8_t fix_status;
        int32_t ecef_x;
        int32_t ecef_y;
        int32_t ecef_z;
        uint32_t position_accuracy_3d;
        int32_t ecef_x_velocity;
        int32_t ecef_y_velocity;
        int32_t ecef_z_velocity;
        uint32_t speed_accuracy;
        uint16_t position_DOP;
        uint8_t res;
        uint8_t satellites;
        uint32_t res2;
    } sol {};
    struct PACKED ubx_nav_dop {
        uint32_t time;                                  // GPS msToW
        uint16_t gDOP;
        uint16_t pDOP;
        uint16_t tDOP;
        uint16_t vDOP;
        uint16_t hDOP;
        uint16_t nDOP;
        uint16_t eDOP;
    } dop {};
    struct PACKED ubx_nav_pvt {
        uint32_t itow;
        uint16_t year;
        uint8_t month, day, hour, min, sec;
        uint8_t valid;
        uint32_t t_acc;
        int32_t nano;
        uint8_t fix_type;
        uint8_t flags;
        uint8_t flags2;
        uint8_t num_sv;
        int32_t lon, lat;
        int32_t height, h_msl;
        uint32_t h_acc, v_acc;
        int32_t velN, velE, velD, gspeed;
        int32_t head_mot;
        uint32_t s_acc;
        uint32_t head_acc;
        uint16_t p_dop;
        uint8_t reserved1[6];
        uint32_t headVeh;
        uint8_t reserved2[4];
    } pvt {};
    const uint8_t SV_COUNT = 10;
    struct PACKED ubx_nav_svinfo {
        uint32_t itow;
        uint8_t numCh;
        uint8_t globalFlags;
        uint8_t reserved1[2];
        // repeated block
        struct PACKED svinfo_sv {
            uint8_t chn;
            uint8_t svid;
            uint8_t flags;
            uint8_t quality;
            uint8_t cno;
            int8_t elev;
            int16_t azim;
            int32_t prRes;
        } sv[SV_COUNT];
    } svinfo {};
    enum RELPOSNED {
        gnssFixOK          = 1U << 0,
        diffSoln           = 1U << 1,
        relPosValid        = 1U << 2,
        carrSolnFloat      = 1U << 3,

        carrSolnFixed      = 1U << 4,
        isMoving           = 1U << 5,
        refPosMiss         = 1U << 6,
        refObsMiss         = 1U << 7,

        relPosHeadingValid = 1U << 8,
        relPosNormalized   = 1U << 9
    };
    struct PACKED ubx_nav_relposned {
        uint8_t version;
        uint8_t reserved1;
        uint16_t refStationId;
        uint32_t iTOW;
        int32_t relPosN;
        int32_t relPosE;
        int32_t relPosD;
        int32_t relPosLength;
        int32_t relPosHeading;
        uint8_t reserved2[4];
        int8_t relPosHPN;
        int8_t relPosHPE;
        int8_t relPosHPD;
        int8_t relPosHPLength;
        uint32_t accN;
        uint32_t accE;
        uint32_t accD;
        uint32_t accLength;
        uint32_t accHeading;
        uint8_t reserved3[4];
        uint32_t flags;
    } relposned {};

    const uint8_t MSG_POSLLH = 0x2;
    const uint8_t MSG_STATUS = 0x3;
    const uint8_t MSG_DOP = 0x4;
    const uint8_t MSG_VELNED = 0x12;
    const uint8_t MSG_SOL = 0x6;
    const uint8_t MSG_PVT = 0x7;
    const uint8_t MSG_SVINFO = 0x30;
    const uint8_t MSG_RELPOSNED = 0x3c;

    uint32_t _next_nav_sv_info_time = 0;

    const auto gps_tow = gps_time();

    pos.time = gps_tow.ms;
    pos.longitude = d->longitude * 1.0e7;
    pos.latitude  = d->latitude * 1.0e7;
    pos.altitude_ellipsoid = d->altitude * 1000.0f;
    pos.altitude_msl = d->altitude * 1000.0f;
    pos.horizontal_accuracy = d->horizontal_acc*1000;
    pos.vertical_accuracy = d->vertical_acc*1000;

    status.time = gps_tow.ms;
    status.fix_type = d->have_lock?3:0;
    status.fix_status = d->have_lock?1:0;
    status.differential_status = 0;
    // spoofDetState: 2 (spoofing indicated) when simulated, else 1 (none indicated)
    status.flags2 = (_sitl->gps_spoof[instance] == 1 ? 2U : 1U) << 3;
    status.time_to_first_fix = 0;
    status.uptime = AP_HAL::millis();

    velned.time = gps_tow.ms;
    velned.ned_north = 100.0f * d->speedN;
    velned.ned_east  = 100.0f * d->speedE;
    velned.ned_down  = 100.0f * d->speedD;
    velned.speed_2d = norm(d->speedN, d->speedE) * 100;
    velned.speed_3d = norm(d->speedN, d->speedE, d->speedD) * 100;
    velned.heading_2d = ToDeg(atan2f(d->speedE, d->speedN)) * 100000.0f;
    if (velned.heading_2d < 0.0f) {
        velned.heading_2d += 360.0f * 100000.0f;
    }
    velned.speed_accuracy = d->speed_acc * 100;  // m/s -> cm/s
    velned.heading_accuracy = 4;

    memset(&sol, 0, sizeof(sol));
    sol.fix_type = d->have_lock?3:0;
    sol.fix_status = 221;
    sol.satellites = d->have_lock ? d->num_sats : 3;
    sol.time = gps_tow.ms;
    sol.week = gps_tow.week;

    dop.time = gps_tow.ms;
    dop.gDOP = 65535;
    dop.pDOP = 65535;
    dop.tDOP = 65535;
    dop.vDOP = 200;
    dop.hDOP = 121;
    dop.nDOP = 65535;
    dop.eDOP = 65535;

    pvt.itow = gps_tow.ms;
    pvt.year = 0;
    pvt.month = 0;
    pvt.day = 0;
    pvt.hour = 0;
    pvt.min = 0;
    pvt.sec = 0;
    pvt.valid = 0; // invalid utc date
    pvt.t_acc = 0;
    pvt.nano = 0;
    pvt.fix_type = d->have_lock? 0x3 : 0;
    pvt.flags = 0b10000011; // carrsoln=fixed, psm = na, diffsoln and fixok
    pvt.flags2 =0;
    pvt.num_sv = d->have_lock ? d->num_sats : 3;
    pvt.lon = d->longitude * 1.0e7;
    pvt.lat  = d->latitude * 1.0e7;
    pvt.height = d->altitude * 1000.0f;
    pvt.h_msl = d->altitude * 1000.0f;
    pvt.h_acc = d->horizontal_acc * 1000;
    pvt.v_acc = d->vertical_acc * 1000;
    pvt.velN = 1000.0f * d->speedN;
    pvt.velE = 1000.0f * d->speedE;
    pvt.velD = 1000.0f * d->speedD;
    pvt.gspeed = norm(d->speedN, d->speedE) * 1000;
    pvt.head_mot = ToDeg(atan2f(d->speedE, d->speedN)) * 1.0e5;
    pvt.s_acc = velned.speed_accuracy;
    pvt.head_acc = 38 * 1.0e5;
    pvt.p_dop = 65535;
    memset(pvt.reserved1, '\0', ARRAY_SIZE(pvt.reserved1));
    pvt.headVeh = 0;
    memset(pvt.reserved2, '\0', ARRAY_SIZE(pvt.reserved2));

    if (_sitl->gps_hdg_enabled[instance] > SITL::SIM::GPS_HEADING_NONE) {
        const Vector3f ant1_pos = _sitl->gps_pos_offset[instance^1].get();
        const Vector3f ant2_pos = _sitl->gps_pos_offset[instance].get();
        Vector3f rel_antenna_pos = ant2_pos - ant1_pos;
        Matrix3f rot;
        // project attitude back using gyros to get antenna orientation at time of GPS sample
        Vector3f gyro(radians(_sitl->state.rollRate),
                      radians(_sitl->state.pitchRate),
                      radians(_sitl->state.yawRate));
        rot.from_euler(radians(_sitl->state.rollDeg), radians(_sitl->state.pitchDeg), radians(d->yaw_deg));
        const float lag = _sitl->gps_delay_ms[instance] * 0.001;
        rot.rotate(gyro * (-lag));
        rel_antenna_pos = rot * rel_antenna_pos;
        relposned.version = 1;
        relposned.iTOW = gps_tow.ms;
        relposned.relPosN = rel_antenna_pos.x * 100;
        relposned.relPosE = rel_antenna_pos.y * 100;
        relposned.relPosD = rel_antenna_pos.z * 100;
        relposned.relPosLength = rel_antenna_pos.length() * 100;
        relposned.relPosHeading = degrees(Vector2f(rel_antenna_pos.x, rel_antenna_pos.y).angle()) * 1.0e5;
        relposned.flags = gnssFixOK | diffSoln | carrSolnFixed | isMoving | relPosValid | relPosHeadingValid;
    }

    send_ubx(MSG_POSLLH, (uint8_t*)&pos, sizeof(pos));
    send_ubx(MSG_STATUS, (uint8_t*)&status, sizeof(status));
    send_ubx(MSG_VELNED, (uint8_t*)&velned, sizeof(velned));
    send_ubx(MSG_SOL,    (uint8_t*)&sol, sizeof(sol));
    send_ubx(MSG_DOP,    (uint8_t*)&dop, sizeof(dop));
    send_ubx(MSG_PVT,    (uint8_t*)&pvt, sizeof(pvt));
    if (_sitl->gps_hdg_enabled[instance] > SITL::SIM::GPS_HEADING_NONE) {
        send_ubx(MSG_RELPOSNED,    (uint8_t*)&relposned, sizeof(relposned));
    }

    const bool is_f9p = (_sitl->gps_options[instance] & static_cast<int32_t>(SITL::SIM::GPSOptions::UBX_IS_F9P)) != 0;

    // send MON-HW (or MON-RF for F9P) at 1Hz with the simulated jamming state
    {
        const uint32_t now_ms = AP_HAL::millis();
        if ((int32_t)(now_ms - _next_mon_send_ms) >= 0) {
            _next_mon_send_ms = now_ms + 1000;
            const bool jammed = _sitl->gps_jam[instance] == 1;
            // u-blox jammingState: 0 = unknown or interference monitor disabled,
            // 1 = ok, 3 = critical. Firmware with UBX-SEC-SIG reports jamming
            // only there and leaves the MON-RF/MON-HW field at 0
            const uint8_t jamming_state = (!_itfm_enabled || sec_sig_firmware()) ? 0 : (jammed ? 3 : 1);
            const uint8_t jam_ind = jammed ? 200 : 20;
            const uint16_t noise_per_ms = jammed ? 200 : 50;
            const uint8_t CLASS_MON = 0x0a;
            if (is_f9p) {
                const uint8_t MSG_MON_RF = 0x38;
                struct PACKED ubx_mon_rf {
                    uint8_t  version;
                    uint8_t  nBlocks;
                    uint8_t  reserved0[2];
                    // single repeated block
                    uint8_t  blockId;
                    uint8_t  flags;          // bits 1:0 are jammingState
                    uint8_t  antStatus;
                    uint8_t  antPower;
                    uint32_t postStatus;
                    uint8_t  reserved1[4];
                    uint16_t noisePerMS;
                    uint16_t agcCnt;
                    uint8_t  cwSuppression;
                    int8_t   ofsI;
                    uint8_t  magI;
                    int8_t   ofsQ;
                    uint8_t  magQ;
                    uint8_t  rfBlockGnssBand;
                    uint8_t  reserved2[2];
                } mon_rf {};
                mon_rf.version = 0;
                mon_rf.nBlocks = 1;
                mon_rf.flags = jamming_state;
                mon_rf.antStatus = 2;   // OK
                mon_rf.antPower = 1;    // on
                mon_rf.noisePerMS = noise_per_ms;
                mon_rf.agcCnt = 4000;
                mon_rf.cwSuppression = jam_ind;
                send_ubx(MSG_MON_RF, (uint8_t*)&mon_rf, sizeof(mon_rf), CLASS_MON);
            } else {
                const uint8_t MSG_MON_HW = 0x09;
                struct PACKED ubx_mon_hw_60 {
                    uint32_t pinSel;
                    uint32_t pinBank;
                    uint32_t pinDir;
                    uint32_t pinVal;
                    uint16_t noisePerMS;
                    uint16_t agcCnt;
                    uint8_t aStatus;
                    uint8_t aPower;
                    uint8_t flags;          // bits 3:2 are jammingState
                    uint8_t reserved1;
                    uint32_t usedMask;
                    uint8_t VP[17];
                    uint8_t jamInd;
                    uint16_t reserved3;
                    uint32_t pinIrq;
                    uint32_t pullH;
                    uint32_t pullL;
                } mon_hw {};
                mon_hw.noisePerMS = noise_per_ms;
                mon_hw.agcCnt = 4000;
                mon_hw.aStatus = 2;     // antenna OK
                mon_hw.aPower = 1;      // antenna on
                mon_hw.flags = jamming_state << 2;
                mon_hw.jamInd = jam_ind;
                send_ubx(MSG_MON_HW, (uint8_t*)&mon_hw, sizeof(mon_hw), CLASS_MON);
            }
            if (sec_sig_firmware() && _sec_sig_rate != 0) {
                // UBX-SEC-SIG v2 with the simulated jamming and spoofing states
                const uint8_t CLASS_SEC = 0x27;
                const uint8_t MSG_SEC_SIG = 0x09;
                struct PACKED ubx_sec_sig_v2 {
                    uint8_t version;
                    uint8_t sigSecFlags;     // jamDetEnabled[0] jammingState[2:1] spfDetEnabled[3] spoofingState[5:4]
                    uint8_t reserved0;
                    uint8_t jamNumCentFreqs;
                } sec_sig {};
                sec_sig.version = 2;
                const uint8_t jam_state = jammed ? 3 : 1;   // 1 = no jamming, 3 = critical
                const uint8_t spf_state = (_sitl->gps_spoof[instance] == 1) ? 2 : 1;
                sec_sig.sigSecFlags = 1U | (jam_state << 1) | (1U << 3) | (spf_state << 4);
                send_ubx(MSG_SEC_SIG, (uint8_t*)&sec_sig, sizeof(sec_sig), CLASS_SEC);
            }
        }
    }

    // F9 firmware does not support NAV-SVINFO; the driver takes the
    // hardware generation from it, so a simulated F9 must not send it
    if (!is_f9p && gps_tow.ms > _next_nav_sv_info_time) {
        svinfo.itow = gps_tow.ms;
        svinfo.numCh = 32;
        svinfo.globalFlags = 4; // u-blox 8/M8
        // fill in the SV's with some data even though firmware does not currently use it
        // note that this is not using num_sats as we aren't dynamically creating this to match
        for (uint8_t i = 0; i < SV_COUNT; i++) {
            svinfo.sv[i].chn = i;
            svinfo.sv[i].svid = i;
            svinfo.sv[i].flags = (i < d->num_sats) ? 0x7 : 0x6; // sv used, diff correction data, orbit information
            svinfo.sv[i].quality = 7; // code and carrier lock and time synchronized
            svinfo.sv[i].cno = MAX(20, 30 - i);
            svinfo.sv[i].elev = MAX(30, 90 - i);
            svinfo.sv[i].azim = i;
            // not bothering to fill in prRes
        }
        send_ubx(MSG_SVINFO, (uint8_t*)&svinfo, sizeof(svinfo));
        _next_nav_sv_info_time = gps_tow.ms + 10000; // 10 second delay
    }
}

bool GPS_UBlox::sec_sig_firmware() const
{
    return (_sitl->gps_options[instance] & static_cast<int32_t>(SITL::SIM::GPSOptions::UBX_SEC_SIG)) != 0;
}

void GPS_UBlox::send_ack(uint8_t cls, uint8_t id, bool ack)
{
    uint8_t payload[2] { cls, id };
    send_ubx(ack ? 0x01 : 0x00, payload, sizeof(payload), 0x05);
}

// value length of a CFG key from its size bits (30:28)
static uint8_t ubx_key_size(uint32_t key)
{
    switch ((key >> 28) & 0x07U) {
    case 1:
    case 2:
        return 1;
    case 3:
        return 2;
    case 4:
        return 4;
    case 5:
        return 8;
    default:
        return 0;
    }
}

/*
  parse UBX messages from the autopilot so the simulated receiver honours
  the configuration that matters for GNSS integrity reporting: the
  jamming/interference monitor enable and the UBX-SEC-SIG output rate
 */
void GPS_UBlox::update_read()
{
    char c;
    while (read_from_autopilot(&c, 1) == 1) {
        const uint8_t b = (uint8_t)c;
        switch (_rx_step) {
        case 0:
            if (b == 0xB5) {
                _rx_step = 1;
            }
            break;
        case 1:
            _rx_step = (b == 0x62) ? 2 : 0;
            break;
        case 2:
            _rx_class = b;
            _rx_ck_a = b;
            _rx_ck_b = b;
            _rx_step = 3;
            break;
        case 3:
            _rx_id = b;
            _rx_ck_a += b;
            _rx_ck_b += _rx_ck_a;
            _rx_step = 4;
            break;
        case 4:
            _rx_len = b;
            _rx_ck_a += b;
            _rx_ck_b += _rx_ck_a;
            _rx_step = 5;
            break;
        case 5:
            _rx_len |= (uint16_t)b << 8;
            _rx_ck_a += b;
            _rx_ck_b += _rx_ck_a;
            _rx_count = 0;
            if (_rx_len > sizeof(_rx_buf)) {
                _rx_step = 0;   // too big for us, resync
            } else {
                _rx_step = (_rx_len == 0) ? 7 : 6;
            }
            break;
        case 6:
            _rx_buf[_rx_count++] = b;
            _rx_ck_a += b;
            _rx_ck_b += _rx_ck_a;
            if (_rx_count == _rx_len) {
                _rx_step = 7;
            }
            break;
        case 7:
            _rx_step = (b == _rx_ck_a) ? 8 : 0;
            break;
        case 8:
            _rx_step = 0;
            if (b == _rx_ck_b) {
                handle_ubx_in();
            }
            break;
        default:
            _rx_step = 0;
            break;
        }
    }
}

void GPS_UBlox::handle_ubx_in()
{
    const uint8_t CLASS_CFG = 0x06;
    const uint8_t MSG_CFG_ITFM = 0x39;
    const uint8_t MSG_CFG_VALSET = 0x8A;
    const uint8_t MSG_CFG_VALGET = 0x8B;
    const uint32_t KEY_ITFM_ENABLE = 0x1041000DU;
    const uint32_t KEY_SEC_SIG_UART1 = 0x20910635U;
    const uint32_t KEY_SEC_SIG_UART2 = 0x20910636U;

    const uint8_t CLASS_MON = 0x0A;
    const uint8_t MSG_MON_VER = 0x04;
    const uint8_t MSG_CFG_PRT = 0x00;
    const bool is_f9p = (_sitl->gps_options[instance] & static_cast<int32_t>(SITL::SIM::GPSOptions::UBX_IS_F9P)) != 0;

    if (_rx_class == CLASS_MON && _rx_id == MSG_MON_VER && _rx_len == 0) {
        // version poll: identify as F9 (ZED-F9P) or M8 so the driver picks
        // the matching configuration path
        struct PACKED {
            char swVersion[30];
            char hwVersion[10];
            char extension[2][30];
        } ver {};
        if (is_f9p) {
            strncpy(ver.swVersion, "EXT CORE 1.00 (SITL)", sizeof(ver.swVersion));
            strncpy(ver.hwVersion, "00190000", sizeof(ver.hwVersion));
            strncpy(ver.extension[0], "MOD=ZED-F9P", sizeof(ver.extension[0]));
            strncpy(ver.extension[1], "PROTVER=27.11", sizeof(ver.extension[1]));
            send_ubx(MSG_MON_VER, (uint8_t*)&ver, sizeof(ver), CLASS_MON);
        } else {
            strncpy(ver.swVersion, "ROM CORE 3.01 (107888)", sizeof(ver.swVersion));
            strncpy(ver.hwVersion, "00080000", sizeof(ver.hwVersion));
            send_ubx(MSG_MON_VER, (uint8_t*)&ver, 40, CLASS_MON);
        }
        return;
    }
    if (_rx_class != CLASS_CFG) {
        return;
    }
    switch (_rx_id) {
    case MSG_CFG_PRT:
        if (_rx_len == 0) {
            // port poll: we are UART1. Without this reply the driver's
            // configuration state machine never advances past its first step
            struct PACKED {
                uint8_t portID;
                uint8_t reserved1;
                uint16_t txReady;
                uint32_t mode;
                uint32_t baudRate;
                uint16_t inProtoMask;
                uint16_t outProtoMask;
                uint16_t flags;
                uint8_t reserved2[2];
            } prt {};
            prt.portID = 1;
            prt.mode = 0x08D0;      // 8N1
            prt.baudRate = 230400;
            prt.inProtoMask = 0x07;
            prt.outProtoMask = 0x03;
            send_ubx(MSG_CFG_PRT, (uint8_t*)&prt, sizeof(prt), CLASS_CFG);
        }
        break;
    case MSG_CFG_ITFM:
        if (_rx_len == 0) {
            // poll: reply with the current configuration
            struct PACKED {
                uint32_t config;
                uint32_t config2;
            } itfm;
            itfm.config = 3U | (15U << 4) | (0x16B156U << 9) | (_itfm_enabled ? (1U << 31) : 0U);
            itfm.config2 = 0x31EU;
            send_ubx(MSG_CFG_ITFM, (uint8_t*)&itfm, sizeof(itfm), CLASS_CFG);
        } else if (_rx_len == 8) {
            uint32_t config;
            memcpy(&config, _rx_buf, sizeof(config));
            _itfm_enabled = (config & (1U << 31)) != 0;
            send_ack(CLASS_CFG, MSG_CFG_ITFM, true);
        }
        break;

    case MSG_CFG_VALSET: {
        // version, layers, reserved[2], then key/value pairs. Only answer
        // when a key we model is involved, other requests are ignored as before
        bool known = false, ok = true;
        uint16_t ofs = 4;
        while (ofs + 4 <= _rx_len) {
            uint32_t key;
            memcpy(&key, &_rx_buf[ofs], sizeof(key));
            ofs += 4;
            const uint8_t vlen = ubx_key_size(key);
            if (vlen == 0 || ofs + vlen > _rx_len) {
                ok = false;
                break;
            }
            uint64_t value = 0;
            memcpy(&value, &_rx_buf[ofs], vlen);
            ofs += vlen;
            if (key == KEY_ITFM_ENABLE) {
                known = true;
                _itfm_enabled = (value & 1U) != 0;
            } else if (key == KEY_SEC_SIG_UART1 || key == KEY_SEC_SIG_UART2) {
                known = true;
                if (!sec_sig_firmware()) {
                    ok = false;   // key does not exist on this firmware
                } else {
                    _sec_sig_rate = (uint8_t)value;
                }
            }
        }
        if (known) {
            send_ack(CLASS_CFG, MSG_CFG_VALSET, ok);
        }
        break;
    }

    case MSG_CFG_VALGET: {
        // version, layer, position[2], then keys. Reply with the keys we
        // model; NACK if one of them is unknown on this firmware
        uint8_t reply[4 + (sizeof(_rx_buf) / 4) * 12] {};
        reply[0] = 1;   // version 1 = response
        reply[1] = _rx_len >= 2 ? _rx_buf[1] : 0;
        uint16_t rofs = 4;
        bool known = false, ok = true;
        for (uint16_t ofs = 4; ofs + 4 <= _rx_len; ofs += 4) {
            uint32_t key;
            memcpy(&key, &_rx_buf[ofs], sizeof(key));
            uint64_t value;
            if (key == KEY_ITFM_ENABLE) {
                value = _itfm_enabled ? 1U : 0U;
            } else if ((key == KEY_SEC_SIG_UART1 || key == KEY_SEC_SIG_UART2) && sec_sig_firmware()) {
                value = _sec_sig_rate;
            } else if (key == KEY_SEC_SIG_UART1 || key == KEY_SEC_SIG_UART2) {
                known = true;
                ok = false;   // key does not exist on this firmware
                break;
            } else {
                ok = false;   // not modelled
                break;
            }
            known = true;
            const uint8_t vlen = ubx_key_size(key);
            memcpy(&reply[rofs], &key, sizeof(key));
            rofs += 4;
            memcpy(&reply[rofs], &value, vlen);
            rofs += vlen;
        }
        if (!known) {
            break;   // nothing we model, stay silent as before
        }
        if (ok) {
            send_ubx(MSG_CFG_VALGET, reply, rofs, CLASS_CFG);
            send_ack(CLASS_CFG, MSG_CFG_VALGET, true);
        } else {
            send_ack(CLASS_CFG, MSG_CFG_VALGET, false);
        }
        break;
    }

    default:
        break;
    }
}

#endif  // AP_SIM_GPS_UBLOX_ENABLED
