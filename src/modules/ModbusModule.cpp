/*
 * ModbusModule.cpp — glue between Meshtastic and the shared firmware_core engine.
 *
 * The Modbus transaction, retry/error-frame logic and raw-forward payload are NOT
 * reimplemented here — they are the same portable C the Star (LoRaWAN) version
 * uses, backed in this build by hal_meshtastic.cpp (Serial1 / millis / LittleFS):
 *     firmware_core/src/{modbus,poll,config}.c
 *
 * Differences vs 16_'s PLCModbusModule (which this de-brands + generalizes):
 *   - poll plan / baud / interval come from the config blob, not #defines;
 *   - payload is the variable-length RAW FORWARD (concatenated [addr][func][data],
 *     per-poll error frame on failure) — the cloud decodes it by config. (The 16_
 *     node wrapped this in a fixed 51-byte Rs485Payload for the project-08 USB
 *     receiver; the generic cloud consumes the raw forward directly.)
 *   - a config blob can be pushed over the mesh on our PortNum (companion app),
 *     not only over the serial console.
 *
 * Register in src/modules/Modules.cpp setupModules():
 *     modbusModule = new ModbusModule();
 */
#include "ModbusModule.h"

#ifdef SQC485IV2

#include "MeshService.h"
#include "NodeDB.h"
#include "configuration.h"
#include "main.h"

extern "C" {
#include "config.h"          // sq_config_t, config_load/save, config_from_blob
#include "poll.h"            // poll_collect_raw  (raw-forward payload)
#include "hal/hal_serial.h"  // hal_serial_init (re-init UART on baud change)
}

ModbusModule *modbusModule;

// Shared config (loaded from our LittleFS store via hal_store in hal_meshtastic.cpp).
static sq_config_t g_cfg;

ModbusModule::ModbusModule()
    : SinglePortModule("modbus", SILIQS_MODBUS_PORTNUM),
      concurrency::OSThread("Modbus")
{
    config_load(&g_cfg);     // defaults if first boot / unprovisioned
}

int32_t ModbusModule::runOnce()
{
    // Gateway nodes (CLIENT_MUTE) bridge the mesh to MQTT and must not poll RS485.
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE)
        return disable();

    if (firstTime) {
        firstTime = false;
        // Bring up RS485 at the configured link params (one-shot; not per-poll).
        hal_serial_init(g_cfg.modbus.baud, g_cfg.modbus.parity, g_cfg.modbus.stop_bits);
        // Priming transaction: the very first RS485 read can mis-align while the
        // line/echo settles. Discard it so the first forwarded packet is clean.
        uint8_t prime[64];
        (void)poll_collect_raw(&g_cfg, prime, sizeof(prime));
        LOG_INFO("ModbusModule: RS485 up %u 8N1, %u poll(s); priming done, first uplink in 3s",
                 (unsigned)g_cfg.modbus.baud, g_cfg.poll_count);
        return 3000;
    }

    pollAndSend();
    // Mesh airtime is precious — the configured interval governs cadence.
    return (int32_t)g_cfg.power.uplink_interval_s * 1000;
}

void ModbusModule::pollAndSend()
{
    // SHARED engine: raw-forward payload — concatenated [addr][func][data] across
    // the poll list; a failed poll contributes a same-length Modbus error frame
    // ([slave][func|0x80][err…]). Never empty when poll_count > 0.
    uint8_t payload[meshtastic_Constants_DATA_PAYLOAD_LEN];
    size_t  len = poll_collect_raw(&g_cfg, payload, sizeof(payload));
    if (len == 0)
        return;

    meshtastic_MeshPacket *p = allocDataPacket();   // portnum set to ours by SinglePortModule
    if (!p)
        return;
    p->want_ack = false;
    memcpy(p->decoded.payload.bytes, payload, len);
    p->decoded.payload.size = len;
    LOG_INFO("ModbusModule: tx raw-forward %u bytes on portnum %u", (unsigned)len,
             (unsigned)SILIQS_MODBUS_PORTNUM);
    service->sendToMesh(p);   // broadcast; an MQTT-gateway node forwards it onward
}

ProcessMessage ModbusModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // SinglePortModule already filtered to our PortNum. Ignore our own uplinks
    // (we both send and receive on this port); only act on remote downlinks.
    if (mp.from == 0 || mp.from == nodeDB->getNodeNum())
        return ProcessMessage::CONTINUE;

    // config_from_blob validates 'SQ' magic + version + length + CRC16, so a
    // stray telemetry/text packet on this port is simply rejected here.
    applyConfigBlob(mp.decoded.payload.bytes, mp.decoded.payload.size, mp.from);
    return ProcessMessage::CONTINUE;
}

void ModbusModule::applyConfigBlob(const uint8_t *blob, size_t len, uint32_t from)
{
    sq_config_t incoming = g_cfg;   // preserve fields the blob doesn't carry (LoRaWAN keys)
    if (!config_from_blob(&incoming, blob, len)) {
        LOG_DEBUG("ModbusModule: rx %u bytes on portnum not a valid config blob; ignored",
                  (unsigned)len);
        return;
    }
    if (!config_save(&incoming)) {
        LOG_WARN("ModbusModule: config blob valid but save failed");
        return;
    }
    g_cfg = incoming;
    // Re-init the UART so a changed baud/parity takes effect without a reboot.
    hal_serial_init(g_cfg.modbus.baud, g_cfg.modbus.parity, g_cfg.modbus.stop_bits);
    LOG_INFO("ModbusModule: config updated over mesh from 0x%08x — %u poll(s), baud %u",
             (unsigned)from, g_cfg.poll_count, (unsigned)g_cfg.modbus.baud);
    // Sender requested an ACK at the mesh-routing layer if it wanted delivery proof;
    // no app-level reply on this port (would look like telemetry to the cloud).
}

#endif // SQC485IV2
