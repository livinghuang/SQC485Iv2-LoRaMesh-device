/*
 * ModbusModule.h — Siliqs config-driven Modbus-over-mesh module for Meshtastic.
 *
 * The generic ("forward firmware") Mesh version. This is the de-branded
 * generalization of 16_'s PLCModbusModule: instead of hardcoded FC03 / slave 1 /
 * 15 s, the poll plan + link params come from the SHARED firmware_core config
 * (binary blob in our own LittleFS file), and the Modbus + raw-forward payload is
 * the SAME portable engine the Star (LoRaWAN) version uses:
 *     firmware_core/src/{modbus,poll,config}.c   (backed by hal_meshtastic.cpp)
 * So one engine, two radios — differentiation is config, not a code fork.
 *
 * Drops into a Meshtastic fork (needs SinglePortModule / OSThread / MeshService /
 * protobufs). API names follow Meshtastic master as surveyed 2026-06; VERIFY
 * against the pinned tag before building — the engine drifts ~monthly.
 * Mirrors the official EnvironmentTelemetry module shape:
 *   SinglePortModule (raw bytes on our private PortNum) + OSThread (periodic poll).
 *
 * Build guard: compiled only when -DSQC485IV2 is set (the board variant), like
 * the 16_ module, so a stock Meshtastic build is unaffected.
 */
#pragma once

#include "mesh/SinglePortModule.h"
#include "concurrency/OSThread.h"

// Private application PortNum for Siliqs Modbus payloads. Must be in the
// 256–511 "private / experimental" range; fixed so the cloud decoder matches it.
// Same value the 16_ deployment uses (meshtastic_PortNum_PRIVATE_APP == 256).
#define SILIQS_MODBUS_PORTNUM ((meshtastic_PortNum)256)

class ModbusModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    ModbusModule();

  protected:
    // Periodic poll → raw-forward payload → mesh-send. Returns ms until next call.
    virtual int32_t runOnce() override;

    // Accept config-blob downlinks addressed to our PortNum (companion-app
    // provisioning over the mesh). Telemetry vs config is told apart by the
    // 'SQ' blob magic + CRC, which config_from_blob() validates.
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  private:
    bool firstTime = true;
    void pollAndSend();          // uses the shared firmware_core engine
    void applyConfigBlob(const uint8_t *blob, size_t len, uint32_t from);
};

extern ModbusModule *modbusModule;
