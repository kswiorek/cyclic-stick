import ctypes
import os
import time
from ctypes import wintypes
from enum import IntEnum

# Constants
DEV_ID = 1

# Load vJoyInterface.dll from script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
dll_path = os.path.join(script_dir, "vJoyInterface.dll")
vjoy = ctypes.CDLL(dll_path)

# Enums
class VjdStat(IntEnum):
    VJD_STAT_OWN = 0
    VJD_STAT_FREE = 1
    VJD_STAT_BUSY = 2
    VJD_STAT_MISS = 3
    VJD_STAT_UNKN = 4

class FFBPType(IntEnum):
    PT_EFFREP   = 0x01  # HID_ID_EFFREP
    PT_ENVREP   = 0x02  # HID_ID_ENVREP
    PT_CONDREP  = 0x03  # HID_ID_CONDREP
    PT_PRIDREP  = 0x04  # HID_ID_PRIDREP
    PT_CONSTREP = 0x05  # HID_ID_CONSTREP
    PT_RAMPREP  = 0x06  # HID_ID_RAMPREP
    PT_CSTMREP  = 0x07  # HID_ID_CSTMREP
    PT_SMPLREP  = 0x08  # HID_ID_SMPLREP
    PT_EFOPREP  = 0x0A  # HID_ID_EFOPREP
    PT_BLKFRREP = 0x0B  # HID_ID_BLKFRREP
    PT_CTRLREP  = 0x0C  # HID_ID_CTRLREP
    PT_GAINREP  = 0x0D  # HID_ID_GAINREP
    PT_SETCREP  = 0x0E  # HID_ID_SETCREP

    # Feature reports (offset +0x10)
    PT_NEWEFREP = 0x11  # HID_ID_NEWEFREP + 0x10
    PT_BLKLDREP = 0x12  # HID_ID_BLKLDREP + 0x10
    PT_POOLREP  = 0x13  # HID_ID_POOLREP + 0x10

class FFBEType(IntEnum):
    ET_NONE = 0
    ET_CONST = 1
    ET_RAMP = 2
    ET_SQR = 3
    ET_SINE = 4
    ET_TRNGL = 5
    ET_STUP = 6
    ET_STDN = 7
    ET_SPRNG = 8
    ET_DMPR = 9
    ET_INRT = 10
    ET_FRCTN = 11
    ET_CSTM = 12

class FFB_CTRL(IntEnum):
    CTRL_ENACT = 1
    CTRL_DISACT = 2
    CTRL_STOPALL = 3
    CTRL_DEVRST = 4
    CTRL_DEVPAUSE = 5
    CTRL_DEVCONT = 6

class FFBOP(IntEnum):
    EFF_START = 1
    EFF_SOLO = 2
    EFF_STOP = 3

# Structures
class FFB_DATA(ctypes.Structure):
    _fields_ = [
        ("size", wintypes.DWORD),
        ("cmd", wintypes.DWORD),
        ("data", ctypes.POINTER(ctypes.c_ubyte))  # pointer, not array!
    ]

class JOYSTICK_POSITION_V2(ctypes.Structure):
    _fields_ = [
        ("bDevice", ctypes.c_byte),
        ("wThrottle", wintypes.LONG),
        ("wRudder", wintypes.LONG),
        ("wAileron", wintypes.LONG),
        ("wAxisX", wintypes.LONG),
        ("wAxisY", wintypes.LONG),
        ("wAxisZ", wintypes.LONG),
        ("wAxisXRot", wintypes.LONG),
        ("wAxisYRot", wintypes.LONG),
        ("wAxisZRot", wintypes.LONG),
        ("wSlider", wintypes.LONG),
        ("wDial", wintypes.LONG),
        ("wWheel", wintypes.LONG),
        ("wAxisVX", wintypes.LONG),
        ("wAxisVY", wintypes.LONG),
        ("wAxisVZ", wintypes.LONG),
        ("wAxisVBRX", wintypes.LONG),
        ("wAxisVBRY", wintypes.LONG),
        ("wAxisVBRZ", wintypes.LONG),
        ("lButtons", wintypes.LONG),
        ("bHats", wintypes.DWORD),
        ("bHatsEx1", wintypes.DWORD),
        ("bHatsEx2", wintypes.DWORD),
        ("bHatsEx3", wintypes.DWORD)
    ]

# Structures for FFB packets (based on vJoy SDK)
class FFB_EFF_COND(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),      # BYTE
        ("isY", wintypes.BOOL),                    # BOOL
        ("CenterPointOffset", wintypes.LONG),      # LONG
        ("PosCoeff", wintypes.LONG),               # LONG
        ("NegCoeff", wintypes.LONG),               # LONG
        ("PosSatur", wintypes.DWORD),              # DWORD
        ("NegSatur", wintypes.DWORD),              # DWORD
        ("DeadBand", wintypes.LONG)                # LONG
    ]

class FFB_EFF_REPORT(ctypes.Structure):
    _fields_ = [
        ("EffectBlockIndex", ctypes.c_ubyte),   # BYTE
        ("EffectType", ctypes.c_ubyte),         # FFBEType (BYTE)
        ("Duration", wintypes.WORD),            # WORD
        ("TrigerRpt", wintypes.WORD),           # WORD
        ("SamplePrd", wintypes.WORD),           # WORD
        ("Gain", ctypes.c_ubyte),               # BYTE
        ("TrigerBtn", ctypes.c_ubyte),          # BYTE
        ("Polar", wintypes.BOOL),               # BOOL
        ("Direction", ctypes.c_ubyte),          # BYTE (union: Direction/DirX)
        ("DirY", ctypes.c_ubyte)                # BYTE
    ]

# Function prototypes
vjoy.vJoyEnabled.restype = wintypes.BOOL
vjoy.GetvJoyManufacturerString.restype = ctypes.c_wchar_p
vjoy.GetvJoyProductString.restype = ctypes.c_wchar_p
vjoy.GetvJoySerialNumberString.restype = ctypes.c_wchar_p
vjoy.GetVJDStatus.argtypes = [wintypes.UINT]
vjoy.GetVJDStatus.restype = ctypes.c_int
vjoy.AcquireVJD.argtypes = [wintypes.UINT]
vjoy.AcquireVJD.restype = wintypes.BOOL
vjoy.RelinquishVJD.argtypes = [wintypes.UINT]
vjoy.FfbStart.argtypes = [wintypes.UINT]
vjoy.FfbStart.restype = wintypes.BOOL
vjoy.UpdateVJD.argtypes = [wintypes.UINT, ctypes.POINTER(JOYSTICK_POSITION_V2)]
vjoy.UpdateVJD.restype = wintypes.BOOL

# FFB callback type
FFB_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)
vjoy.FfbRegisterGenCB.argtypes = [FFB_CALLBACK, ctypes.c_void_p]

# Global FFB data
ffb_direction = 0
ffb_strength = 0

# Helper functions
def packet_type_to_str(ptype):
    types = {
        FFBPType.PT_EFFREP: "Effect Report",
        FFBPType.PT_ENVREP: "Envelope Report",
        FFBPType.PT_CONDREP: "Condition Report",
        FFBPType.PT_PRIDREP: "Periodic Report",
        FFBPType.PT_CONSTREP: "Constant Force Report",
        FFBPType.PT_RAMPREP: "Ramp Force Report",
        FFBPType.PT_CSTMREP: "Custom Force Data Report",
        FFBPType.PT_SMPLREP: "Download Force Sample",
        FFBPType.PT_EFOPREP: "Effect Operation Report",
        FFBPType.PT_BLKFRREP: "PID Block Free Report",
        FFBPType.PT_CTRLREP: "PID Device Control",
        FFBPType.PT_GAINREP: "Device Gain Report",
        FFBPType.PT_SETCREP: "Set Custom Force Report",
        FFBPType.PT_NEWEFREP: "Create New Effect Report",
        FFBPType.PT_BLKLDREP: "Block Load Report",
        FFBPType.PT_POOLREP: "PID Pool Report"
    }
    return types.get(ptype, f"Unknown ({ptype})")

def effect_type_to_str(etype):
    types = {
        FFBEType.ET_CONST: "Constant Force",
        FFBEType.ET_RAMP: "Ramp",
        FFBEType.ET_SQR: "Square",
        FFBEType.ET_SINE: "Sine",
        FFBEType.ET_TRNGL: "Triangle",
        FFBEType.ET_STUP: "Sawtooth Up",
        FFBEType.ET_STDN: "Sawtooth Down",
        FFBEType.ET_SPRNG: "Spring",
        FFBEType.ET_DMPR: "Damper",
        FFBEType.ET_INRT: "Inertia",
        FFBEType.ET_FRCTN: "Friction",
        FFBEType.ET_CSTM: "Custom Force"
    }
    return types.get(etype, f"Unknown ({etype})")

def polar_to_deg(polar):
    """Convert polar values (0x00-0xFF) to degrees (0-360)"""
    return (polar * 360) // 255

def byte_to_percent(byte_val):
    """Convert range 0x00-0xFF to 0%-100%"""
    return (byte_val * 100) // 255

def to_signed_16(val):
    """Convert unsigned 16-bit integer to signed 16-bit integer."""
    if val >= 0x8000:
        return val - 0x10000
    return val

# Declare vJoy FFB helper prototypes
vjoy.Ffb_h_DeviceID.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
vjoy.Ffb_h_DeviceID.restype = wintypes.DWORD

vjoy.Ffb_h_Type.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
vjoy.Ffb_h_Type.restype = wintypes.DWORD

vjoy.Ffb_h_EBI.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
vjoy.Ffb_h_EBI.restype = wintypes.DWORD

vjoy.Ffb_h_EffNew.argtypes = [ctypes.POINTER(FFB_DATA), ctypes.POINTER(ctypes.c_int)]
vjoy.Ffb_h_EffNew.restype = wintypes.DWORD

# FFB Callback function
def ffb_callback(data_ptr, userdata):
    try:
        ffb_data = ctypes.cast(data_ptr, ctypes.POINTER(FFB_DATA)).contents

        print(f"\n{'='*50}")
        print(f"FFB Packet size: {ffb_data.size}")
        print(f"Cmd: 0x{ffb_data.cmd:08X}")

        # Device ID
        device_id = ctypes.c_int()
        if vjoy.Ffb_h_DeviceID(ctypes.byref(ffb_data), ctypes.byref(device_id)) == 0:
            print(f"Device ID: {device_id.value}")

        # Packet Type
        packet_type = ctypes.c_int()
        if vjoy.Ffb_h_Type(ctypes.byref(ffb_data), ctypes.byref(packet_type)) == 0:
            print(f"Packet Type: {packet_type_to_str(packet_type.value)}")

        # Effect Block Index
        block_index = ctypes.c_int()
        if vjoy.Ffb_h_EBI(ctypes.byref(ffb_data), ctypes.byref(block_index)) == 0:
            print(f"Effect Block Index: {block_index.value}")

        if packet_type.value == FFBPType.PT_CONDREP:
            condition = FFB_EFF_COND()
            if vjoy.Ffb_h_Eff_Cond(ctypes.byref(ffb_data), ctypes.byref(condition)) == 0:
                axis = "Y Axis" if condition.isY else "X Axis"
                print(f"Condition: {axis}")
                print(f"Center Point Offset: {to_signed_16(condition.CenterPointOffset)/100}")
                print(f"Positive Coefficient: {to_signed_16(condition.PosCoeff)/100}")
                print(f"Negative Coefficient: {to_signed_16(condition.NegCoeff)/100}")
                print(f"Positive Saturation: {condition.PosSatur}")
                print(f"Negative Saturation: {condition.NegSatur}")
                print(f"Dead Band: {to_signed_16(condition.DeadBand)/100}")
        elif packet_type.value == FFBPType.PT_EFFREP:
            effect_report = FFB_EFF_REPORT()
            # print(vjoy.Ffb_h_Eff_Report(ctypes.byref(ffb_data), ctypes.byref(effect_report)))
            if vjoy.Ffb_h_Eff_Report(ctypes.byref(ffb_data), ctypes.byref(effect_report)) == 0:
                print(f"Effect Report: {effect_type_to_str(effect_report.EffectType)}")
                if effect_report.Polar:
                    print(f"Direction: {polar_to_deg(effect_report.Direction)} deg ({effect_report.Direction:02x})")
                else:
                    print(f"X Direction: {effect_report.DirX:02x}")
                    print(f"Y Direction: {effect_report.DirY:02x}")
                if effect_report.Duration == 0xFFFF:
                    print("Duration: Infinite")
                else:
                    print(f"Duration: {effect_report.Duration} ms")
                if effect_report.TrigerRpt == 0xFFFF:
                    print("Trigger Repeat: Infinite")
                else:
                    print(f"Trigger Repeat: {effect_report.TrigerRpt}")
                if effect_report.SamplePrd == 0xFFFF:
                    print("Sample Period: Infinite")
                else:
                    print(f"Sample Period: {effect_report.SamplePrd}")
                print(f"Gain: {byte_to_percent(effect_report.Gain)}%")

        # Raw Data
        data_size = ffb_data.size - 8
        if data_size > 0:
            data_array = ctypes.cast(ffb_data.data, ctypes.POINTER(ctypes.c_ubyte * data_size)).contents
            print("Raw Data:", end=" ")
            for i in range(min(data_size, 20)):
                print(f"{data_array[i]:02X}", end=" ")
            print()
        print('='*50)
    except Exception as e:
        print(f"Error in FFB callback: {e}")

# Create callback wrapper
ffb_callback_wrapper = FFB_CALLBACK(ffb_callback)

def main():
    global ffb_direction, ffb_strength
    
    dev_id = DEV_ID
    
    # Check if vJoy is enabled
    if not vjoy.vJoyEnabled():
        print("vJoy is not enabled or installed!")
        return -2
    
    print(f"Vendor: {vjoy.GetvJoyManufacturerString()}")
    print(f"Product: {vjoy.GetvJoyProductString()}")
    print(f"Version: {vjoy.GetvJoySerialNumberString()}")
    
    # Get device status
    status = vjoy.GetVJDStatus(dev_id)
    
    if status == VjdStat.VJD_STAT_OWN:
        print(f"vJoy device {dev_id} is already owned by this feeder")
    elif status == VjdStat.VJD_STAT_FREE:
        print(f"vJoy device {dev_id} is free")
    elif status == VjdStat.VJD_STAT_BUSY:
        print(f"vJoy device {dev_id} is already owned by another feeder")
        return -3
    elif status == VjdStat.VJD_STAT_MISS:
        print(f"vJoy device {dev_id} is not installed or disabled")
        return -4
    else:
        print(f"vJoy device {dev_id} general error")
        return -1
    
    # Acquire device
    if not vjoy.AcquireVJD(dev_id):
        print(f"Failed to acquire vJoy device {dev_id}")
        return -1
    
    print(f"Acquired device {dev_id} - OK")
    
    # Start FFB
    if not vjoy.FfbStart(dev_id):
        print(f"Failed to start FFB on vJoy device {dev_id}")
        vjoy.RelinquishVJD(dev_id)
        return -3
    
    print(f"Started FFB on vJoy device {dev_id} - OK")
    
    # Register FFB callback
    vjoy.FfbRegisterGenCB(ffb_callback_wrapper, None)
    print("FFB callback registered")
    
    # Main loop - send position data
    i_report = JOYSTICK_POSITION_V2()
    i_report.bDevice = dev_id
    z = 0
    
    try:
        print("\nStarting main loop (Ctrl+C to exit)...")
        while True:
            # Update position data
            if z > 35000:
                z = 0
            z += 200
            
            i_report.wAxisZ = z
            i_report.wAxisX = 32000 - z
            i_report.wAxisY = z // 2 + 7000
            
            # Update buttons
            i_report.lButtons = 1 << (z // 4000)
            
            # Send data to vJoy device
            if not vjoy.UpdateVJD(dev_id, ctypes.byref(i_report)):
                print(f"Feeding vJoy device {dev_id} failed")
                time.sleep(1)
                vjoy.AcquireVJD(dev_id)
            
            time.sleep(0.002)  # 2ms delay
            
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        vjoy.RelinquishVJD(dev_id)
        print("Device released")
    
    return 0

if __name__ == "__main__":
    exit(main())