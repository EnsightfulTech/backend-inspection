"""
Standalone READ-ONLY Modbus probe for the gantry PLC.

Purpose: verify that the D-register numbers the backend/calibration code uses
actually line up with the AutoShop variable table, and see their live values,
WITHOUT writing anything. It issues no writes at all, so it cannot command a
move or change PLC state.

Run (rig PC, PLC reachable):
    python plc_read_registers.py                 # default range around D900
    python plc_read_registers.py --start 900 --count 12
    python plc_read_registers.py --watch         # poll until Ctrl-C

Known V1.2 mapping (from the AutoShop variable table):
    D902  iUp_PosNum        host -> PLC : commanded stop index (1..iCameraNum)
    D903  iUp_PosNum_Last   PLC -> host : becomes == commanded when the move completes
    D904  iProdrdy_ToUp     PLC -> host : wall/production line in position
    D905  iUp_CameraOk      host -> PLC : capture-done handshake
    D906  iCameraNum        host -> PLC : stop count (PLC derives rCameraDis)  [UNVERIFIED]

Note this reads Modbus holding registers by the same numbers the D-registers use,
which is what the existing backend client assumes. If the values here look
nothing like what AutoShop shows online, that assumption is wrong and the PLC is
applying a Modbus address offset.
"""

import argparse
import sys
import time

try:
    from pyModbusTCP.client import ModbusClient
except ImportError:
    print("[FAIL] pyModbusTCP not installed in this environment.")
    sys.exit(1)

LABELS = {
    902: "iUp_PosNum       (host->PLC commanded stop index)",
    903: "iUp_PosNum_Last  (PLC->host arrived-at index)",
    904: "iProdrdy_ToUp    (PLC->host wall in position)",
    905: "iUp_CameraOk     (host->PLC capture done)",
    906: "iCameraNum       (host->PLC stop count)  [UNVERIFIED]",
}


def read_block(client, start, count):
    regs = client.read_holding_registers(start, count)
    if regs is None:
        print(f"[FAIL] read_holding_registers({start}, {count}) returned None "
              f"-- PLC unreachable, or that range is not readable.")
        return None
    return regs


def show(regs, start):
    for i, val in enumerate(regs):
        addr = start + i
        label = LABELS.get(addr, "")
        mark = " <--" if label else ""
        print(f"  D{addr} = {val:<8}{label}{mark}")


def main():
    ap = argparse.ArgumentParser(description="Read-only PLC register probe (no writes).")
    ap.add_argument("--host", default=None, help="PLC IP (default: config.PLC_HOST)")
    ap.add_argument("--port", type=int, default=None, help="Modbus port (default: config.PLC_PORT)")
    ap.add_argument("--start", type=int, default=900, help="first register (default 900)")
    ap.add_argument("--count", type=int, default=12, help="how many (default 12)")
    ap.add_argument("--watch", action="store_true", help="poll every 0.5s until Ctrl-C")
    args = ap.parse_args()

    host, port = args.host, args.port
    if host is None or port is None:
        try:
            import config
            host = host or config.PLC_HOST
            port = port or config.PLC_PORT
        except Exception as e:
            print(f"[FAIL] could not read config.py for PLC_HOST/PORT ({e}); "
                  f"pass --host/--port explicitly.")
            return 1

    print(f"[INFO] connecting to {host}:{port} (READ-ONLY; this script never writes)")
    client = ModbusClient(host=host, port=port)
    if not client.open():
        print(f"[FAIL] could not open Modbus connection to {host}:{port}. "
              f"Check the PLC is powered, on the network, and reachable (ping).")
        return 1
    print("[ OK ] connected.")

    try:
        if args.watch:
            print("[INFO] polling every 0.5s; Ctrl-C to stop.\n")
            while True:
                regs = read_block(client, args.start, args.count)
                if regs is None:
                    return 1
                print(f"--- {time.strftime('%H:%M:%S')} ---")
                show(regs, args.start)
                print()
                time.sleep(0.5)
        else:
            regs = read_block(client, args.start, args.count)
            if regs is None:
                return 1
            show(regs, args.start)
            print("\n[INFO] Cross-check these against AutoShop's online monitor. If they "
                  "disagree, the D-number != Modbus-address assumption is wrong.")
        return 0
    except KeyboardInterrupt:
        print("\n[INFO] stopped.")
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    sys.exit(main())
