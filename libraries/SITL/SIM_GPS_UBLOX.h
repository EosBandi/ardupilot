#include "SIM_config.h"

#if AP_SIM_GPS_UBLOX_ENABLED

#include "SIM_GPS.h"

namespace SITL {

class GPS_UBlox : public GPS_Backend {
public:
    CLASS_NO_COPY(GPS_UBlox);

    using GPS_Backend::GPS_Backend;

    void publish(const GPS_Data *d) override;
    void update_read() override;

private:
    uint32_t _next_mon_send_ms;

    void send_ubx(uint8_t msgid, uint8_t *buf, uint16_t size, uint8_t msgclass=0x01);

    // receiver configuration set by the autopilot (see update_read())
    bool _itfm_enabled;      // jamming/interference monitor (CFG-ITFM / CFG-ITFM-ENABLE)
    uint8_t _sec_sig_rate;   // CFG-MSGOUT-UBX_SEC_SIG_UART1/2

    // parser for UBX messages from the autopilot
    uint8_t _rx_step, _rx_class, _rx_id, _rx_ck_a, _rx_ck_b;
    uint16_t _rx_len, _rx_count;
    uint8_t _rx_buf[128];
    void handle_ubx_in();
    void send_ack(uint8_t cls, uint8_t id, bool ack);
    // simulated firmware reports jamming/spoofing via UBX-SEC-SIG
    bool sec_sig_firmware() const;
};

};

#endif  // AP_SIM_GPS_UBLOX_ENABLED
