#!/usr/bin/env python3
"""30 FPS configuration / network budget checks, without board-I/O signoff.

The FIFO cores are encrypted.  This test deliberately does not infer their
physical depth from count-port width or substitute the behavioral FIFO for IP.
The service bound assumes two cameras deliver the same row at a bounded skew;
loss, arbitrary frame skew and real PCLK / I/O margins require board checks.
"""
import json
from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


def read(relative):
    return (ROOT / relative).read_text()


def registers():
    # Initialization is ordered: the last literal write to a register wins.
    return {int(a, 16): int(v, 16) for a, v in re.findall(
        r"24'h([0-9a-fA-F]{4})_([0-9a-fA-F]{2})",
        read("camera/ov5640_init_table_raw.v"))}


def metrics():
    regs = registers()
    pll = read("camera/gowin_pll_mod.v")
    odiv = lambda n: int(re.search(
        rf"ODIV{n}_SEL\s*=\s*(\d+)", pll).group(1))
    gmii_hz = 125_000_000.0
    # Existing shared PLL's 125 MHz output is the reference; ODIV0=10 / ODIV2=52.
    xclk_hz = gmii_hz * odiv(0) / odiv(2)
    vco_hz = xclk_hz * regs[0x3036] / (regs[0x3037] & 15)
    sysdiv = (regs[0x3035] >> 4) or 16
    pll_root = ((regs[0x3037] >> 4) & 1) + 1
    bitdiv = (regs[0x3034] & 15) / 2
    sclk_root = 1 << (regs[0x3108] & 3)
    sclk_hz = vco_hz / sysdiv / pll_root * 2 / bitdiv / sclk_root
    hts = (regs[0x380C] << 8) | regs[0x380D]
    vts = (regs[0x380E] << 8) | regs[0x380F]
    fps = sclk_hz / hts / vts
    row_bytes = 1920 + 2
    # Actual frame is 1976 GMII byte clocks (8 preamble + 14 Ethernet +
    # 20 IP + 8 UDP + 1922 payload + 4 FCS); saturated-gap test observes 20.
    wire_bytes = 8 + 14 + 20 + 8 + row_bytes + 4
    saturated_idle_bytes = 20
    service_clocks = wire_bytes + saturated_idle_bytes
    row_period = hts / sclk_hz
    return dict(
        xclk_hz=xclk_hz, vco_hz=vco_hz, sclk_hz=sclk_hz,
        hts=hts, vts=vts, target_fps=fps, raw_bytes_per_frame=1920 * 1080,
        udp_payload_bytes=row_bytes, ethernet_frame_gmii_bytes=wire_bytes,
        saturated_idle_byte_clocks=saturated_idle_bytes,
        wire_mbit_s=(service_clocks * 1080 * fps * 8) / 1e6,
        imu_200hz_wire_mbit_s=(90 + 12 + 2) * 200 * 8 / 1e6,
        active_row_period_us=row_period * 1e6,
        row_service_us=service_clocks / gmii_hz * 1e6,
        active_rows_wire_mbit_s=service_clocks * 8 / row_period / 1e6,
        gmii_hz=gmii_hz,
    )


class StreamBudget(unittest.TestCase):
    def test_same_protocol_and_scheduler(self):
        core = read("camera/ov5640x2_udp_rgmii.v")
        controller = read("camera/controller.v")
        self.assertRegex(core, r"IMAGE_WIDTH\s*=\s*960")
        self.assertRegex(core, r"IMAGE_HEIGHT\s*=\s*1080")
        self.assertRegex(core, r"data_length\s*\(IMAGE_WIDTH\*2\+2")
        self.assertRegex(core, r"des_port\s*\(16'd6102")
        self.assertRegex(controller, r"IMG_WIDTH=480,\s*IMG_HEIGHT=1080")
        self.assertIn("rdusedw1>=IMG_WIDTH*2+2", controller)
        self.assertIn("row1==row2", controller)
        self.assertIn("resync || row1==0", controller)
        sender = read("camera/UDP_Send.v")
        self.assertRegex(sender, r"parameter POST_FRAME_GAP=16")
        self.assertIn("if(tx_allow && rdusedw >= data_length)", sender)
        share = read("rtl/gmii_frame_share.v")
        self.assertIn("camera_allow=!imu_pending && !imu_busy && !imu_valid", share)
        self.assertIn("imu_allow=!camera_busy && !camera_valid", share)

    def test_sensor_and_timing_envelope(self):
        m = metrics()
        self.assertEqual((m["hts"], m["vts"]), (2500, 1120))
        self.assertTrue(29.9 < m["target_fps"] < 30.1)
        # Keep the board-verified 20 FPS sensor PLL and peak DVP clock.
        self.assertTrue(840e6 < m["vco_hz"] < 843e6)
        self.assertTrue(84e6 < m["sclk_hz"] < 85e6)
        sdc = read("imu_camera_dual.sdc")
        for camera in ("cam1_pclk", "cam2_pclk"):
            match = re.search(rf"create_clock -name {camera} -period ([\d.]+)", sdc)
            self.assertIsNotNone(match)
            # The 100 MHz internal timing envelope exceeds approximate 84 MHz.
            # This makes NO claim about external data setup/hold, skew or PCLK duty.
            self.assertLessEqual(float(match.group(1)), 1e9 / m["sclk_hz"])
        self.assertIn("external I/O timing is not signed off", sdc)
        self.assertIn("set_max_delay 8.000", sdc)

    def test_active_burst_and_average_network_budget(self):
        m = metrics()
        self.assertEqual(m["udp_payload_bytes"], 1922)
        self.assertEqual(m["ethernet_frame_gmii_bytes"], 1976)
        self.assertLess(m["wire_mbit_s"] + m["imu_200hz_wire_mbit_s"], 520)
        self.assertLess(m["active_rows_wire_mbit_s"], 550)
        # Conservatively add one IMU frame and arbitration slack to EVERY row,
        # instead of the real ~200 IMU packets/s.  It must still clear before
        # the next active row arrives.  No unbounded source skew is assumed.
        imu_and_arb_us = (90 + 12 + 16) / m["gmii_hz"] * 1e6
        self.assertLess(m["row_service_us"] + imu_and_arb_us,
                        m["active_row_period_us"])
        # Two complete pending rows fit even a conservative 4096-byte model.
        # Physical FIFO depth / overflow remains a separate hardware check.
        self.assertLess(2 * m["udp_payload_bytes"], 4096)


if __name__ == "__main__":
    print(json.dumps(metrics(), ensure_ascii=False, indent=2))
    unittest.main()
