"""USB control backend for the XVF3800."""

from __future__ import annotations

import ctypes
import ctypes.util
import math
import struct
import time
from dataclasses import dataclass
from typing import Any

from .commands import COMMANDS, CONTROL_SUCCESS, SERVICER_COMMAND_RETRY


LIBUSB_SUCCESS = 0
LIBUSB_ERROR_NOT_FOUND = -5
LIBUSB_ERROR_BUSY = -6


@dataclass
class DeviceIdentity:
    vid: int
    pid: int
    bus_number: int
    address: int


class XVF3800Error(RuntimeError):
    """Raised when the XVF3800 backend cannot satisfy a request."""


class _LibUSB:
    """Thin ctypes wrapper around libusb-1.0."""

    class libusb_context(ctypes.Structure):
        pass

    class libusb_device(ctypes.Structure):
        pass

    class libusb_device_handle(ctypes.Structure):
        pass

    class libusb_device_descriptor(ctypes.Structure):
        _fields_ = [
            ("bLength", ctypes.c_uint8),
            ("bDescriptorType", ctypes.c_uint8),
            ("bcdUSB", ctypes.c_uint16),
            ("bDeviceClass", ctypes.c_uint8),
            ("bDeviceSubClass", ctypes.c_uint8),
            ("bDeviceProtocol", ctypes.c_uint8),
            ("bMaxPacketSize0", ctypes.c_uint8),
            ("idVendor", ctypes.c_uint16),
            ("idProduct", ctypes.c_uint16),
            ("bcdDevice", ctypes.c_uint16),
            ("iManufacturer", ctypes.c_uint8),
            ("iProduct", ctypes.c_uint8),
            ("iSerialNumber", ctypes.c_uint8),
            ("bNumConfigurations", ctypes.c_uint8),
        ]

    def __init__(self) -> None:
        libname = ctypes.util.find_library("usb-1.0")
        if not libname:
            raise XVF3800Error("libusb-1.0 not found on this system")
        self.lib = ctypes.CDLL(libname)
        self._configure()

    def _configure(self) -> None:
        lib = self.lib
        lib.libusb_init.argtypes = [ctypes.POINTER(ctypes.POINTER(self.libusb_context))]
        lib.libusb_init.restype = ctypes.c_int
        lib.libusb_exit.argtypes = [ctypes.POINTER(self.libusb_context)]
        lib.libusb_get_device_list.argtypes = [
            ctypes.POINTER(self.libusb_context),
            ctypes.POINTER(ctypes.POINTER(ctypes.POINTER(self.libusb_device))),
        ]
        lib.libusb_get_device_list.restype = ctypes.c_ssize_t
        lib.libusb_free_device_list.argtypes = [
            ctypes.POINTER(ctypes.POINTER(self.libusb_device)),
            ctypes.c_int,
        ]
        lib.libusb_get_device_descriptor.argtypes = [
            ctypes.POINTER(self.libusb_device),
            ctypes.POINTER(self.libusb_device_descriptor),
        ]
        lib.libusb_get_device_descriptor.restype = ctypes.c_int
        lib.libusb_get_bus_number.argtypes = [ctypes.POINTER(self.libusb_device)]
        lib.libusb_get_bus_number.restype = ctypes.c_uint8
        lib.libusb_get_device_address.argtypes = [ctypes.POINTER(self.libusb_device)]
        lib.libusb_get_device_address.restype = ctypes.c_uint8
        lib.libusb_open.argtypes = [
            ctypes.POINTER(self.libusb_device),
            ctypes.POINTER(ctypes.POINTER(self.libusb_device_handle)),
        ]
        lib.libusb_open.restype = ctypes.c_int
        lib.libusb_close.argtypes = [ctypes.POINTER(self.libusb_device_handle)]
        lib.libusb_set_auto_detach_kernel_driver.argtypes = [
            ctypes.POINTER(self.libusb_device_handle),
            ctypes.c_int,
        ]
        lib.libusb_claim_interface.argtypes = [
            ctypes.POINTER(self.libusb_device_handle),
            ctypes.c_int,
        ]
        lib.libusb_release_interface.argtypes = [
            ctypes.POINTER(self.libusb_device_handle),
            ctypes.c_int,
        ]
        lib.libusb_detach_kernel_driver.argtypes = [
            ctypes.POINTER(self.libusb_device_handle),
            ctypes.c_int,
        ]
        lib.libusb_control_transfer.argtypes = [
            ctypes.POINTER(self.libusb_device_handle),
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint16,
            ctypes.c_uint16,
            ctypes.c_void_p,
            ctypes.c_uint16,
            ctypes.c_uint,
        ]
        lib.libusb_control_transfer.restype = ctypes.c_int
        lib.libusb_strerror.argtypes = [ctypes.c_int]
        lib.libusb_strerror.restype = ctypes.c_char_p

    def error_name(self, code: int) -> str:
        try:
            return self.lib.libusb_strerror(code).decode("utf-8", "ignore")
        except Exception:
            return f"libusb error {code}"


class XVF3800USBBackend:
    """Vendor-control transport with the official XVF3800 command map."""

    interface_number = 3
    timeout_ms = 100_000

    def __init__(self, vid: int, pid: int, device_index: int = 0, retries: int = 5):
        self.vid = vid
        self.pid = pid
        self.device_index = device_index
        self.retries = retries
        self._libusb = _LibUSB()
        self._ctx = ctypes.POINTER(_LibUSB.libusb_context)()
        self._handle = ctypes.POINTER(_LibUSB.libusb_device_handle)()
        self.identity: DeviceIdentity | None = None
        self._open()

    def close(self) -> None:
        if self._handle:
            try:
                self._libusb.lib.libusb_release_interface(self._handle, self.interface_number)
            except Exception:
                pass
            try:
                self._libusb.lib.libusb_close(self._handle)
            except Exception:
                pass
            self._handle = ctypes.POINTER(_LibUSB.libusb_device_handle)()
        if self._ctx:
            try:
                self._libusb.lib.libusb_exit(self._ctx)
            except Exception:
                pass
            self._ctx = ctypes.POINTER(_LibUSB.libusb_context)()

    def __enter__(self) -> "XVF3800USBBackend":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _open(self) -> None:
        if self._ctx:
            self.close()
        rc = self._libusb.lib.libusb_init(ctypes.byref(self._ctx))
        if rc != LIBUSB_SUCCESS:
            raise XVF3800Error(
                "failed to initialize libusb for XVF3800 "
                f"({self._libusb.error_name(rc)}); check USB permissions and device connection"
            )

        device_ptrs = ctypes.POINTER(ctypes.POINTER(_LibUSB.libusb_device))()
        count = self._libusb.lib.libusb_get_device_list(self._ctx, ctypes.byref(device_ptrs))
        if count < 0:
            raise XVF3800Error(f"libusb_get_device_list failed: {self._libusb.error_name(count)}")

        matches = []
        try:
            idx = 0
            while True:
                device = device_ptrs[idx]
                if not device:
                    break
                desc = _LibUSB.libusb_device_descriptor()
                rc = self._libusb.lib.libusb_get_device_descriptor(device, ctypes.byref(desc))
                if rc == LIBUSB_SUCCESS and desc.idVendor == self.vid and desc.idProduct == self.pid:
                    matches.append((device, desc))
                idx += 1
        finally:
            self._libusb.lib.libusb_free_device_list(device_ptrs, 1)

        if len(matches) <= self.device_index:
            self.close()
            raise XVF3800Error(
                f"XVF3800 device not found for vid=0x{self.vid:04x} pid=0x{self.pid:04x} "
                f"index={self.device_index}"
            )

        device, desc = matches[self.device_index]
        handle = ctypes.POINTER(_LibUSB.libusb_device_handle)()
        rc = self._libusb.lib.libusb_open(device, ctypes.byref(handle))
        if rc != LIBUSB_SUCCESS:
            self.close()
            raise XVF3800Error(f"libusb_open failed: {self._libusb.error_name(rc)}")

        self._handle = handle
        self._libusb.lib.libusb_set_auto_detach_kernel_driver(self._handle, 1)
        try:
            self._libusb.lib.libusb_detach_kernel_driver(self._handle, self.interface_number)
        except Exception:
            pass
        rc = self._libusb.lib.libusb_claim_interface(self._handle, self.interface_number)
        if rc not in (LIBUSB_SUCCESS, LIBUSB_ERROR_BUSY):
            self.close()
            raise XVF3800Error(f"libusb_claim_interface failed: {self._libusb.error_name(rc)}")

        bus = self._libusb.lib.libusb_get_bus_number(device)
        address = self._libusb.lib.libusb_get_device_address(device)
        self.identity = DeviceIdentity(desc.idVendor, desc.idProduct, bus, address)

    def reconnect(self) -> None:
        self.close()
        time.sleep(0.2)
        self._open()

    def _ensure_open(self) -> None:
        if not self._handle:
            self._open()

    def _control_transfer(self, direction: int, cmdid: int, resid: int, payload_or_length: Any):
        self._ensure_open()
        request_type = direction | 0x40 | 0x80  # vendor/device; overwritten below
        if direction == 0x80:
            request_type = 0xC0
        else:
            request_type = 0x40
        if direction == 0x80:
            buffer = ctypes.create_string_buffer(int(payload_or_length))
            rc = self._libusb.lib.libusb_control_transfer(
                self._handle,
                request_type,
                0,
                cmdid,
                resid,
                buffer,
                int(payload_or_length),
                self.timeout_ms,
            )
            if rc < 0:
                raise XVF3800Error(f"USB read failed: {self._libusb.error_name(rc)}")
            return buffer.raw[:rc]
        data = payload_or_length or b""
        if not isinstance(data, (bytes, bytearray)):
            data = bytes(data)
        buf = ctypes.create_string_buffer(data, len(data))
        rc = self._libusb.lib.libusb_control_transfer(
            self._handle,
            request_type,
            0,
            cmdid,
            resid,
            buf,
            len(data),
            self.timeout_ms,
        )
        if rc < 0:
            raise XVF3800Error(f"USB write failed: {self._libusb.error_name(rc)}")
        return rc

    @staticmethod
    def _pack_value(data_type: str, value: Any) -> bytes:
        if data_type in {"float", "radians"}:
            return struct.pack("<f", float(value))
        if data_type == "uint8":
            return struct.pack("<B", int(value))
        if data_type == "uint16":
            return struct.pack("<H", int(value))
        if data_type == "uint32":
            return struct.pack("<I", int(value))
        if data_type == "int32":
            return struct.pack("<i", int(value))
        if data_type == "char":
            if isinstance(value, str):
                return value.encode("utf-8")
            return bytes(value)
        raise XVF3800Error(f"Unsupported command type: {data_type}")

    def write_command(self, name: str, values: list[Any]) -> None:
        spec = COMMANDS[name]
        if spec.access == "ro":
            raise XVF3800Error(f"{name} is read-only")
        if len(values) != spec.count:
            raise XVF3800Error(f"{name} expects {spec.count} value(s), got {len(values)}")
        payload = b"".join(self._pack_value(spec.data_type, value) for value in values)
        self._control_transfer(0x40, spec.cmdid, spec.resid, payload)

    def read_command(self, name: str) -> Any:
        spec = COMMANDS[name]
        if spec.access == "wo":
            raise XVF3800Error(f"{name} is write-only")
        length = 1
        if spec.data_type in {"float", "radians", "uint32", "int32"}:
            length += 4 * spec.count
        elif spec.data_type == "uint16":
            length += 2 * spec.count
        elif spec.data_type in {"uint8", "char"}:
            length += spec.count
        else:
            raise XVF3800Error(f"Unsupported command type: {spec.data_type}")

        attempts = 0
        while True:
            attempts += 1
            response = self._control_transfer(0x80, 0x80 | spec.cmdid, spec.resid, length)
            if not response:
                raise XVF3800Error(f"{name} returned no data")
            status = response[0]
            if status == CONTROL_SUCCESS:
                break
            if status != SERVICER_COMMAND_RETRY or attempts >= 100:
                raise XVF3800Error(f"{name} failed with status {status}")
            time.sleep(0.01)

        payload = response[1:]
        if spec.data_type == "char":
            return payload.rstrip(b"\x00").decode("utf-8", "ignore")
        if spec.data_type == "uint8":
            return list(payload[: spec.count])
        fmt_map = {
            "float": "f",
            "radians": "f",
            "uint16": "H",
            "uint32": "I",
            "int32": "i",
        }
        fmt = "<" + fmt_map[spec.data_type] * spec.count
        return list(struct.unpack(fmt, payload[: struct.calcsize(fmt)]))

    def read_optional(self, name: str) -> Any | None:
        try:
            return self.read_command(name)
        except Exception:
            return None

    def set_mute(self, mute: bool) -> None:
        self.write_command("GPO_WRITE_VALUE", [30, 1 if mute else 0])

    def set_led(
        self,
        effect: int | None = None,
        brightness: int | None = None,
        gammify: bool | None = None,
        speed: int | None = None,
        color: int | None = None,
        doa_colors: list[int] | None = None,
        ring_colors: list[int] | None = None,
    ) -> None:
        if effect is not None:
            self.write_command("LED_EFFECT", [effect])
        if brightness is not None:
            self.write_command("LED_BRIGHTNESS", [brightness])
        if gammify is not None:
            self.write_command("LED_GAMMIFY", [1 if gammify else 0])
        if speed is not None:
            self.write_command("LED_SPEED", [speed])
        if color is not None:
            self.write_command("LED_COLOR", [color])
        if doa_colors is not None:
            if len(doa_colors) != 2:
                raise XVF3800Error("doa_colors must contain exactly 2 values")
            self.write_command("LED_DOA_COLOR", doa_colors)
        if ring_colors is not None:
            if len(ring_colors) != 12:
                raise XVF3800Error("ring_colors must contain exactly 12 values")
            self.write_command("LED_RING_COLOR", ring_colors)

    def set_gain(self, mic_gain: float | None = None, ref_gain: float | None = None) -> None:
        if mic_gain is not None:
            self.write_command("AUDIO_MGR_MIC_GAIN", [mic_gain])
        if ref_gain is not None:
            self.write_command("AUDIO_MGR_REF_GAIN", [ref_gain])

    def set_audio_output_muxes(
        self,
        left_mux: list[int] | None = None,
        right_mux: list[int] | None = None,
    ) -> None:
        if left_mux is not None:
            if len(left_mux) != 2:
                raise XVF3800Error("left_mux must contain exactly 2 values")
            self.write_command("AUDIO_MGR_OP_L", left_mux)
        if right_mux is not None:
            if len(right_mux) != 2:
                raise XVF3800Error("right_mux must contain exactly 2 values")
            self.write_command("AUDIO_MGR_OP_R", right_mux)

    def set_agc(self, max_gain: float | None = None, gain: float | None = None) -> None:
        if max_gain is not None:
            self.write_command("PP_AGCMAXGAIN", [max_gain])
        if gain is not None:
            self.write_command("PP_AGCGAIN", [gain])

    def set_asr_output_gain(self, gain: float | None = None) -> None:
        if gain is not None:
            self.write_command("AEC_ASROUTGAIN", [gain])

    def save_configuration(self) -> None:
        self.write_command("SAVE_CONFIGURATION", [1])

    def reboot(self) -> None:
        self.write_command("REBOOT", [1])

    def reset_device(self) -> None:
        self.write_command("CLEAR_CONFIGURATION", [1])
        time.sleep(0.05)
        self.write_command("REBOOT", [1])

    def doa_snapshot(self) -> dict[str, Any]:
        """Fast read of just the data needed for voice DOA tracking."""
        doa = self.read_optional("DOA_VALUE") or [None, None]
        azimuths = self.read_optional("AEC_AZIMUTH_VALUES") or []
        energies = self.read_optional("AEC_SPENERGY_VALUES") or []
        return {
            "doa_deg": float(doa[0]) if doa and doa[0] is not None else None,
            "vad": bool(doa[1]) if doa and len(doa) > 1 else None,
            "azimuth_values": azimuths,
            "speech_energy": energies,
        }

    def snapshot(self) -> dict[str, Any]:
        fields = {
            "version": self.read_optional("VERSION"),
            "build_message": self.read_optional("BLD_MSG"),
            "build_host": self.read_optional("BLD_HOST"),
            "repo_hash": self.read_optional("BLD_REPO_HASH"),
            "boot_status": self.read_optional("BOOT_STATUS"),
            "doa_value": self.read_optional("DOA_VALUE"),
            "azimuth_values": self.read_optional("AEC_AZIMUTH_VALUES"),
            "speech_energy": self.read_optional("AEC_SPENERGY_VALUES"),
            "selected_azimuths": self.read_optional("AUDIO_MGR_SELECTED_AZIMUTHS"),
            "selected_channels": self.read_optional("AUDIO_MGR_SELECTED_CHANNELS"),
            "gpi": self.read_optional("GPI_READ_VALUES"),
            "gpo": self.read_optional("GPO_READ_VALUES"),
            "mic_gain": self.read_optional("AUDIO_MGR_MIC_GAIN"),
            "ref_gain": self.read_optional("AUDIO_MGR_REF_GAIN"),
            "output_left_mux": self.read_optional("AUDIO_MGR_OP_L"),
            "output_right_mux": self.read_optional("AUDIO_MGR_OP_R"),
            "agc_enabled": self.read_optional("PP_AGCONOFF"),
            "echo_enabled": self.read_optional("PP_ECHOONOFF"),
            "aec_converged": self.read_optional("AEC_AECCONVERGED"),
            "i2s_inactive": self.read_optional("I2S_INACTIVE"),
            "asr_output_enabled": self.read_optional("AEC_ASROUTONOFF"),
            "sys_delay": self.read_optional("AUDIO_MGR_SYS_DELAY"),
        }
        doa = fields.get("doa_value") or [None, None]
        gpo = fields.get("gpo") or []
        gpi = fields.get("gpi") or []
        fields.update(
            {
                "mute": bool(gpo[1]) if len(gpo) > 1 else None,
                "led_power": bool(gpo[3]) if len(gpo) > 3 else None,
                "mute_button": bool(gpi[0]) if len(gpi) > 0 else None,
                "doa_deg": float(doa[0]) if doa and doa[0] is not None else None,
                "vad": bool(doa[1]) if doa and len(doa) > 1 else None,
            }
        )
        return fields
