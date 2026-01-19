import serial
import struct
import os
import sys
import glob

# ---------------- Bootloader status codes ----------------
Flash_HAL_OK        = 0x00
Flash_HAL_ERROR     = 0x01
Flash_HAL_BUSY      = 0x02
Flash_HAL_TIMEOUT   = 0x03
Flash_HAL_INV_ADDR  = 0x04

# ---------------- Bootloader Commands -------------------
COMMAND_BL_GET_VER                  = 0x51
COMMAND_BL_GET_HELP                 = 0x52
COMMAND_BL_GET_CID                  = 0x53
COMMAND_BL_GET_RDP_STATUS           = 0x54
COMMAND_BL_GO_TO_ADDR               = 0x55
COMMAND_BL_FLASH_ERASE              = 0x56
COMMAND_BL_MEM_WRITE                = 0x57
COMMAND_BL_EN_R_W_PROTECT           = 0x58
COMMAND_BL_READ_SECTOR_P_STATUS     = 0x5A
COMMAND_BL_DIS_R_W_PROTECT          = 0x5C

# ---------------- Command lengths -----------------------
COMMAND_BL_GET_VER_LEN              = 6
COMMAND_BL_GET_HELP_LEN             = 6
COMMAND_BL_GET_CID_LEN              = 6
COMMAND_BL_GET_RDP_STATUS_LEN       = 6
COMMAND_BL_GO_TO_ADDR_LEN           = 10
COMMAND_BL_FLASH_ERASE_LEN          = 8
COMMAND_BL_MEM_WRITE_LEN            = 11
COMMAND_BL_EN_R_W_PROTECT_LEN       = 8
COMMAND_BL_READ_SECTOR_P_STATUS_LEN = 6
COMMAND_BL_DIS_R_W_PROTECT_LEN      = 6

verbose_mode = 1
mem_write_active = 0

# ---------------- File operations -----------------------

def calc_file_len():
    return os.path.getsize("user_app.bin")

def open_the_file():
    global bin_file
    bin_file = open("user_app.bin", "rb")

def close_the_file():
    bin_file.close()

# ---------------- Utilities -----------------------------

def word_to_byte(addr, index):
    return (addr >> (8 * (index - 1))) & 0xFF

def get_crc(buff, length):
    Crc = 0xFFFFFFFF
    for data in buff[0:length]:
        Crc ^= data
        for _ in range(32):
            if (Crc & 0x80000000):
                Crc = (Crc << 1) ^ 0x04C11DB7
            else:
                Crc = (Crc << 1)
    return Crc & 0xFFFFFFFF

# ---------------- Serial Handling -----------------------

def serial_ports():
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux'):
        ports = glob.glob('/dev/tty[A-Za-z]*')
    else:
        return []
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except:
            pass
    return result

def Serial_Port_Configuration(port):
    global ser
    try:
        ser = serial.Serial(port, 115200, timeout=2)
    except:
        print("\n   Invalid port!")
        print("   Available ports:", serial_ports())
        return -1

    if ser.is_open:
        print("\n   Port opened successfully")
        return 0
    else:
        print("\n   Failed to open port")
        return -1

def read_serial_port(length):
    return ser.read(length)

def purge_serial_port():
    ser.reset_input_buffer()

def Write_to_serial_port(value):
    data = struct.pack('>B', value)
    if verbose_mode:
        print("0x{:02X}".format(value), end=' ')
    ser.write(data)

# ---------------- Reply Processing ----------------------

def process_GET_VER(length):
    value = read_serial_port(length)
    print("\n   Bootloader Version:", hex(value[0]))

def process_GET_HELP(length):
    value = read_serial_port(length)
    print("\n   Supported Commands:", [hex(x) for x in value])

def process_GET_CID(length):
    value = read_serial_port(length)
    cid = (value[1] << 8) | value[0]
    print("\n   Chip ID:", hex(cid))

def process_GET_RDP(length):
    value = read_serial_port(length)
    print("\n   RDP Level:", hex(value[0]))

def process_GO_TO_ADDR(length):
    value = read_serial_port(length)
    print("\n   Address Status:", hex(value[0]))

def process_FLASH_ERASE(length):
    value = read_serial_port(length)
    status = value[0]

    if status == Flash_HAL_OK:
        print("\n   Flash Erase: SUCCESS")
    else:
        print("\n   Flash Erase: FAIL  Code:", hex(status))

def process_MEM_WRITE(length):
    value = read_serial_port(length)
    status = value[0]

    if status == Flash_HAL_OK:
        print("\n   Write: SUCCESS")
    else:
        print("\n   Write: FAIL  Code:", hex(status))

def process_READ_SECTOR_STATUS(length):
    value = read_serial_port(length)
    status = value[0]
    print("\n   Sector Protection Status:", hex(status))

def process_EN_PROTECT(length):
    value = read_serial_port(length)
    print("\n   Protection Enabled" if value[0] == 0 else "\n   Protection Failed")

def process_DIS_PROTECT(length):
    value = read_serial_port(length)
    print("\n   Protection Disabled" if value[0] == 0 else "\n   Disable Failed")

# ---------------- Bootloader Reply Handler ---------------

def read_bootloader_reply(command_code):
    ack = read_serial_port(2)
    if not ack:
        print("\n   Timeout: No response from bootloader")
        return -1

    if ack[0] == 0xA5:
        length = ack[1]
        print("\n   CRC OK, Reply length:", length)

        if command_code == COMMAND_BL_GET_VER:
            process_GET_VER(length)
        elif command_code == COMMAND_BL_GET_HELP:
            process_GET_HELP(length)
        elif command_code == COMMAND_BL_GET_CID:
            process_GET_CID(length)
        elif command_code == COMMAND_BL_GET_RDP_STATUS:
            process_GET_RDP(length)
        elif command_code == COMMAND_BL_GO_TO_ADDR:
            process_GO_TO_ADDR(length)
        elif command_code == COMMAND_BL_FLASH_ERASE:
            process_FLASH_ERASE(length)
        elif command_code == COMMAND_BL_MEM_WRITE:
            process_MEM_WRITE(length)
        elif command_code == COMMAND_BL_READ_SECTOR_P_STATUS:
            process_READ_SECTOR_STATUS(length)
        elif command_code == COMMAND_BL_EN_R_W_PROTECT:
            process_EN_PROTECT(length)
        elif command_code == COMMAND_BL_DIS_R_W_PROTECT:
            process_DIS_PROTECT(length)

        return 0

    elif ack[0] == 0x7F:
        print("\n   CRC FAIL (NACK)")
        return -1

# ---------------- Menu Command Handling ------------------

def send_simple_command(cmd, length):
    buf = [0] * 20
    buf[0] = length - 1
    buf[1] = cmd
    crc = get_crc(buf, length - 4)

    buf[2] = word_to_byte(crc, 1)
    buf[3] = word_to_byte(crc, 2)
    buf[4] = word_to_byte(crc, 3)
    buf[5] = word_to_byte(crc, 4)

    Write_to_serial_port(buf[0])
    for i in range(1, length):
        Write_to_serial_port(buf[i])

    read_bootloader_reply(cmd)

# ---------------- MAIN PROGRAM ---------------------------

port = input("\nEnter COM port (Ex: COM3): ")
if Serial_Port_Configuration(port) < 0:
    sys.exit()

while True:

    print("\n +==========================================+")
    print(" |        STM32F4 UART Bootloader Tool       |")
    print(" +==========================================+")

    print("\n   1  → BL_GET_VER")
    print("   2  → BL_GET_HELP")
    print("   3  → BL_GET_CID")
    print("   4  → BL_GET_RDP_STATUS")
    print("   5  → BL_GO_TO_ADDR")
    print("   6  → BL_FLASH_ERASE")
    print("   7  → BL_MEM_WRITE")
    print("   8  → BL_EN_R_W_PROTECT")
    print("   9  → BL_READ_SECTOR_P_STATUS")
    print("   10 → BL_DIS_R_W_PROTECT")
    print("   0  → EXIT")

    cmd = input("\nEnter command: ")

    if not cmd.isdigit():
        continue

    cmd = int(cmd)

    if cmd == 0:
        print("\nExiting...")
        break

    elif cmd == 1:
        send_simple_command(COMMAND_BL_GET_VER, COMMAND_BL_GET_VER_LEN)

    elif cmd == 2:
        send_simple_command(COMMAND_BL_GET_HELP, COMMAND_BL_GET_HELP_LEN)

    elif cmd == 3:
        send_simple_command(COMMAND_BL_GET_CID, COMMAND_BL_GET_CID_LEN)

    elif cmd == 4:
        send_simple_command(COMMAND_BL_GET_RDP_STATUS, COMMAND_BL_GET_RDP_STATUS_LEN)

    elif cmd == 5:
        addr = int(input("Enter jump address (hex): "), 16)
        buf = [0]*20
        buf[0] = COMMAND_BL_GO_TO_ADDR_LEN - 1
        buf[1] = COMMAND_BL_GO_TO_ADDR

        buf[2] = word_to_byte(addr,1)
        buf[3] = word_to_byte(addr,2)
        buf[4] = word_to_byte(addr,3)
        buf[5] = word_to_byte(addr,4)

        crc = get_crc(buf, COMMAND_BL_GO_TO_ADDR_LEN - 4)
        buf[6] = word_to_byte(crc,1)
        buf[7] = word_to_byte(crc,2)
        buf[8] = word_to_byte(crc,3)
        buf[9] = word_to_byte(crc,4)

        Write_to_serial_port(buf[0])
        for i in range(1, COMMAND_BL_GO_TO_ADDR_LEN):
            Write_to_serial_port(buf[i])

        read_bootloader_reply(COMMAND_BL_GO_TO_ADDR)

    elif cmd == 6:
        sector = int(input("Enter start sector (hex or dec): "), 0)
        nsec   = int(input("Enter number of sectors: "))

        buf = [0]*20
        buf[0] = COMMAND_BL_FLASH_ERASE_LEN - 1
        buf[1] = COMMAND_BL_FLASH_ERASE
        buf[2] = sector
        buf[3] = nsec

        crc = get_crc(buf, COMMAND_BL_FLASH_ERASE_LEN - 4)
        buf[4] = word_to_byte(crc,1)
        buf[5] = word_to_byte(crc,2)
        buf[6] = word_to_byte(crc,3)
        buf[7] = word_to_byte(crc,4)

        Write_to_serial_port(buf[0])
        for i in range(1, COMMAND_BL_FLASH_ERASE_LEN):
            Write_to_serial_port(buf[i])

        read_bootloader_reply(COMMAND_BL_FLASH_ERASE)

    elif cmd == 7:
        print("\nFlashing user_app.bin ...")

        open_the_file()
        size = calc_file_len()

        base_addr = int(input("Enter flash start address (hex): "), 16)

        sent = 0
        while sent < size:

            chunk = min(128, size - sent)
            buf = [0]*300

            buf[1] = COMMAND_BL_MEM_WRITE
            buf[2] = word_to_byte(base_addr,1)
            buf[3] = word_to_byte(base_addr,2)
            buf[4] = word_to_byte(base_addr,3)
            buf[5] = word_to_byte(base_addr,4)
            buf[6] = chunk

            for i in range(chunk):
                buf[7+i] = ord(bin_file.read(1))

            total_len = COMMAND_BL_MEM_WRITE_LEN + chunk
            buf[0] = total_len - 1

            crc = get_crc(buf, total_len - 4)
            buf[7+chunk]  = word_to_byte(crc,1)
            buf[8+chunk]  = word_to_byte(crc,2)
            buf[9+chunk]  = word_to_byte(crc,3)
            buf[10+chunk] = word_to_byte(crc,4)

            Write_to_serial_port(buf[0])
            for i in range(1, total_len):
                Write_to_serial_port(buf[i])

            read_bootloader_reply(COMMAND_BL_MEM_WRITE)

            sent += chunk
            base_addr += chunk
            print("   Sent:", sent, "/", size)

        close_the_file()

    elif cmd == 8:
        sector = int(input("Enter sector number to protect: "))
        buf = [0]*20
        buf[0] = COMMAND_BL_EN_R_W_PROTECT_LEN - 1
        buf[1] = COMMAND_BL_EN_R_W_PROTECT
        buf[2] = (1 << sector)
        buf[3] = 1

        crc = get_crc(buf, COMMAND_BL_EN_R_W_PROTECT_LEN - 4)
        buf[4] = word_to_byte(crc,1)
        buf[5] = word_to_byte(crc,2)
        buf[6] = word_to_byte(crc,3)
        buf[7] = word_to_byte(crc,4)

        Write_to_serial_port(buf[0])
        for i in range(1, COMMAND_BL_EN_R_W_PROTECT_LEN):
            Write_to_serial_port(buf[i])

        read_bootloader_reply(COMMAND_BL_EN_R_W_PROTECT)

    elif cmd == 9:
        send_simple_command(COMMAND_BL_READ_SECTOR_P_STATUS, COMMAND_BL_READ_SECTOR_P_STATUS_LEN)

    elif cmd == 10:
        send_simple_command(COMMAND_BL_DIS_R_W_PROTECT, COMMAND_BL_DIS_R_W_PROTECT_LEN)

    input("\nPress Enter to continue...")
    purge_serial_port()

