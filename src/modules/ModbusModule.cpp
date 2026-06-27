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
    // Telemetry destination (config v3): unicast to a chosen node, or broadcast
    // (the allocDataPacket default); and the chosen mesh channel index.
    if (g_cfg.tx.dest_node)
        p->to = g_cfg.tx.dest_node;
    p->channel = g_cfg.tx.channel;
    memcpy(p->decoded.payload.bytes, payload, len);
    p->decoded.payload.size = len;
    // Log the payload hex (capped) — lets the installer see the actual forwarded
    // bytes on the console; a node's own mesh broadcasts don't always reach the
    // phone API, so this is the reliable bench read-back.
    char hx[2 * 32 + 1];
    size_t hn = len < 32 ? len : 32;
    for (size_t i = 0; i < hn; i++)
        snprintf(hx + 2 * i, 3, "%02x", payload[i]);
    hx[2 * hn] = 0;
    LOG_INFO("ModbusModule: tx raw-forward %u bytes on portnum %u: %s", (unsigned)len,
             (unsigned)SILIQS_MODBUS_PORTNUM, hx);
    // ccToPhone=true so a USB/BLE-connected configurator (which is this node's own
    // "phone") can see this node's reads — its mesh broadcasts don't otherwise reach
    // the local API. Harmless when no client is attached.
    service->sendToMesh(p, RX_SRC_LOCAL, true);   // broadcast + cc to the local client
}

ProcessMessage ModbusModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Config downlinks and our own telemetry uplinks share this PortNum, and we
    // CANNOT tell them apart by source: a companion app provisions by injecting
    // the config on the LOCAL node's Meshtastic API, so its packet is from==self,
    // exactly like our telemetry loopback. So let CONTENT decide — only a valid
    // 'SQ' blob (magic+version+length+CRC16, checked by config_from_blob) is
    // treated as config; raw-forward telemetry fails that and is ignored. A cheap
    // magic pre-check keeps our own uplinks from spamming the debug log.
    const uint8_t *b = mp.decoded.payload.bytes;
    size_t n = mp.decoded.payload.size;

    // Poll-now test request: the 3 bytes 'S','Q','?'. Do an immediate read and
    // send it the SAME way as a periodic read — pollAndSend() broadcasts + cc's to
    // the phone, which reliably reaches a connected configurator. (An addressed
    // self-reply was unreliable; the broadcast path is the one that works.)
    // A config blob is 'S','Q',ver(=2),… so byte[2]='?' can't collide with it.
    if (n >= 3 && b[0] == 'S' && b[1] == 'Q' && b[2] == '?') {
        LOG_INFO("ModbusModule: poll-now request — reading + broadcasting");
        pollAndSend();
        return ProcessMessage::CONTINUE;
    }

    if (n < 4 || b[0] != 'S' || b[1] != 'Q')
        return ProcessMessage::CONTINUE;

    applyConfigBlob(b, n, mp.from);
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
