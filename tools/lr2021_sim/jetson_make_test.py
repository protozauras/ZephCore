#!/usr/bin/env python3
"""Upload the lr2021_sim sandbox + rtl_433 vendored sources to the Jetson
(aarch64 gcc pre-verify, -Wall -Wextra -Werror) and run `make test`.

Layout on the Jetson (relative paths preserved):
    /tmp/lr2021_check/tools/lr2021_sim/...   sim sources + Makefile
    /tmp/lr2021_check/zephcore/src/rtl433/...  vendored core + glue
    /tmp/lr2021_check/zephcore/adapters/radio/...  existing headers

Usage: python jetson_make_test.py
"""

import os
import stat

import paramiko

HOST = "192.168.8.133"
USER = "pi"
PASSWORD = "linux1"

REPO = os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))))
SIM_SRC = os.path.join(REPO, "tools", "lr2021_sim")
RTL_SRC = os.path.join(REPO, "zephcore", "src", "rtl433")
RADIO_SRC = os.path.join(REPO, "zephcore", "adapters", "radio")

DEST = "/tmp/lr2021_check"

SIM_FILES = [
    "Makefile",
    "stub_lr2021.c", "stub_lr2021.h",
    "driver_under_test.c", "driver_under_test.h",
    "test_lr2021_driver.c",
    "tdm_wedge_model.c", "tdm_wedge_model.h",
    "test_tdm_wedge.c",
    "test_rtl433.c",
]

RTL_FILES = [
    "bitbuffer.c", "bitbuffer.h",
    "bit_util.c", "bit_util.h",
    "data.c", "data.h",
    "abuf.c", "abuf.h",
    "list.c", "list.h",
    "decoder_util.c", "decoder_util.h",
    "pulse_slicer.c", "pulse_slicer.h",
    "pulse_data.h",
    "r_device.h",
    "decoder.h",
    "c_util.h",
    "fatal.h",
    "logger.h",
    "rtl433_mem.h",
    "sniffer_rtl433.c", "sniffer_rtl433.h",
    "VENDOR.md",
    "devices/acurite.c",
    "devices/lacrosse_tx141x.c",
    "devices/oregon_scientific.c",
    "devices/oregon_scientific_v1.c",
    "devices/fineoffset.c",
    "devices/prologue.c",
    "devices/hideki.c",
]


def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASSWORD)
    sftp = client.open_sftp()

    def mkdir_p(path):
        parts = path.strip("/").split("/")
        cur = ""
        for p in parts:
            cur += "/" + p
            try:
                sftp.stat(cur)
            except FileNotFoundError:
                sftp.mkdir(cur)

    def put(local, remote):
        sftp.put(local, remote)
        sftp.chmod(remote, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP |
                   stat.S_IROTH)

    print("uploading sim sources...")
    mkdir_p(f"{DEST}/tools/lr2021_sim")
    for name in SIM_FILES:
        put(os.path.join(SIM_SRC, name),
            f"{DEST}/tools/lr2021_sim/{name}")

    print("uploading rtl433 sources...")
    mkdir_p(f"{DEST}/zephcore/src/rtl433/devices")
    for name in RTL_FILES:
        put(os.path.join(RTL_SRC, name),
            f"{DEST}/zephcore/src/rtl433/{name}")

    print("uploading adapters/radio headers...")
    mkdir_p(f"{DEST}/zephcore/adapters/radio")
    if os.path.isdir(RADIO_SRC):
        for name in sorted(os.listdir(RADIO_SRC)):
            local = os.path.join(RADIO_SRC, name)
            if os.path.isfile(local):
                put(local, f"{DEST}/zephcore/adapters/radio/{name}")

    print("running make test on Jetson...")
    stdin, stdout, stderr = client.exec_command(
        f"cd {DEST}/tools/lr2021_sim && make clean > /dev/null 2>&1; "
        f"make 2> make_err.txt; echo MAKE_EXIT=$?; ./lr2021_sim_tests; "
        f"echo TEST_EXIT=$?",
        timeout=600)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    print("--- make/test output ---")
    print(out)
    if err.strip():
        print("--- stderr ---")
        print(err)

    _, make_err_out, _ = client.exec_command(
        f"cat {DEST}/tools/lr2021_sim/make_err.txt 2>/dev/null")
    make_err = make_err_out.read().decode("utf-8", errors="replace")
    if make_err.strip():
        print("--- make stderr ---")
        print(make_err)

    sftp.close()
    client.close()
    print("done")


if __name__ == "__main__":
    main()
