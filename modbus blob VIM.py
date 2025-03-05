from pyModbusTCP.client import ModbusClient
import time
import csv
class ModbusClientWrapper:
    def __init__(self, host, port, unit_id):
        self.client = ModbusClient(host=host, port=port, unit_id=unit_id, auto_open=True, auto_close=True)

    def read_register(self, register_address, number_of_registers=1):
        try:
            value = self.client.read_holding_registers(register_address, number_of_registers)
            if value:
                #print(f"Read value: {value} from register: {register_address}")
                return value 
            else:
                print(f"Failed to read from register: {register_address}")
                return None
        except Exception as e:
            print(f"Error reading register: {e}")
            return None

    def write_register(self, register_address, value):
        try:
            success = self.client.write_single_register(register_address, value)
            if success:
                #print(f"Wrote value: {value} to register: {register_address}")
                pass
            else:
                print(f"Failed to write to register: {register_address}")
        except Exception as e:
            print(f"Error writing register: {e}")

    def write_multiple_register(self, register_address, *value):
        try:
            success = self.client.write_multiple_registers(register_address, *value)
            if success:
                #print(f"Wrote value: {value} to register: {register_address}")
                pass
            else:
                print(f"Failed to write to register: {register_address}")
        except Exception as e:
            print(f"Error writing register: {e}")


def read_blob_id_status(port_number):
    write_register = port_number*1000 + 300
    read_register = port_number*1000 + 100
    client.write_multiple_register(write_register, [1, 49, 0, 2])
    client.write_multiple_register(write_register, [1, 49, 0, 2])
    
    response = client.read_register(read_register, 6)

    status = response[0]
    index = response[1]
    subindex = response[2]
    length = response[3]
    data = response[4]
    blob_id = data - 65536 if data > 32767 else data

    return {
        "status": status,
        "index": index,
        "subindex": subindex,
        "length": length,
        "data": data,
        "blob_id": blob_id
    }

def trigger_Blob_capture(port_number):
    write_register = port_number*1000 + 300
    read_register = port_number*1000 + 100
    client.write_multiple_register(write_register, [2, 50, 0, 3, 0xf1f0,0x00])

def Blob_data_capture(port_number):
    write_register = port_number*1000 + 300
    read_register = port_number*1000 + 100
    client.write_multiple_register(write_register, [1, 50, 0, 201])
    time.sleep(0.2)
    response = client.read_register(read_register, 105)
    #print(response)
    try:
        if response[4] == 4096:
            Total_data_size = (response[5] << 16) + response[6]
            print(f'The total data size is {Total_data_size} bytes')
            print(f'The total sample captured is {Total_data_size/1024} samples')
            input("Press Enter to continue...")
        else:
            Blob_data_parser(response[4:])
    except:
        pass
    return response[7]

def Blob_finish_capture(port_number):
    write_register = port_number*1000 + 300
    read_register = port_number*1000 + 100
    client.write_multiple_register(write_register, [2, 50, 0, 2, 0x00f2])
   

def Get_Blob_pdi_status(port_number):
    read_register = port_number*1000
    response = client.read_register(read_register, 15)

    if response:
        byte_value = response[11] & 0xFF  # Get the lower byte (00 part of fe00)
        bit_6_8_value = (byte_value >> 5) & 0x07  # Extract bits 6, 7, 8
        decimal_value = bit_6_8_value  # Convert to decimal number (0 to 3)
        #print(decimal_value)
        return decimal_value
    else:
        print("No response received")

def Blob_data_parser(data):
    # Data is a list of 16-bit integers
    hex_data = [format(x, '04x') for x in data]  # Convert each 16-bit value to hex
    counter = int(hex_data[0][:2], 16)  # Higher 8-bit value is the counter (upper byte of the first 16-bit value)
    g_values = []

    for i in range(0, len(hex_data), 2):
        if i + 2 < len(hex_data):
            lower_byte = hex_data[i][2:]  # Lower byte of hex_data[i]
            next_value = hex_data[i + 1]
            higher_byte = hex_data[i + 2][:2]  # Higher byte of hex_data[i + 2]
            combined_value = lower_byte + next_value + higher_byte
            #print(combined_value)
            g_values.append(int(combined_value, 16))
    
    if g_values and g_values[1] != 0:
        print(f"Counter: {counter}")
        #print(f"G values: {g_values}")
        actual_g_values = [(g_value / 1969.3568) - 50 for g_value in g_values]
        print(f"Actual G values: {actual_g_values}")
        with open("g_values.csv", "a", newline='') as file:
            writer = csv.writer(file)
            #writer.writerow(["G Value", "Counter"])
            for g_value in actual_g_values:
                writer.writerow([g_value, counter])
        pass
def blob_config(port_number,ssc_trigger=0, raw_data_sampling_rate=5, raw_data_memmory_size=10):

    write_register = port_number*1000 + 300
    read_register = port_number*1000 + 100
    # 0:Inactive
    # 1:SSC1.1 - vRMS
    # 2:SSC1.2 - vRMS
    # 3:SSC2.1 - aRMS
    # 4:SSC2.2 - aRMS
    # 5:SSC3.1 - aPeak
    # 6:SSC3.2 - aPeak
    # 7:SSC4.1 - Temperature
    # 8:SSC4.2 - Temperature
    # 9:SSC5.1 - Crest
    # 10:SSC5.2 - Crest
    # 11:SSC6.1 - Bearing Condition
    # 12:SSC6.2 - Bearing Condition
    client.write_multiple_register(write_register, [2, 101, 0, 1, int(f"0x{ssc_trigger:02x}00", 16)])
    client.write_multiple_register(write_register, [2, 101, 0, 1, int(f"0x{ssc_trigger:02x}00", 16)])
    time.sleep(1)
    client.write_multiple_register(write_register, [1, 101, 0, 1])
    client.write_multiple_register(write_register, [1, 101, 0, 1])
    time.sleep(1)
    response = client.read_register(read_register, 6)
    ssc_trigger_status = (response[4] >> 8) & 0xFF
    print(f'SSC trigger is set to {ssc_trigger_status}')

    # 0:64 kHz sampling rate, 12 kHz low-pass
    # 1:32 kHz sampling rate, 6 kHz low-pass
    # 2:16 kHz sampling rate, 3 kHz low-pass
    # 3:8 kHz sampling rate, 1.5 kHz low-pass
    # 4:4 kHz sampling rate, 0.75 kHz low-pass
    # 5:2 kHz sampling rate, 0.3 kHz low-pass
    
  
    client.write_multiple_register(write_register, [2, 97, 0, 1, int(f"0x{raw_data_sampling_rate:02x}00", 16)])
    client.write_multiple_register(write_register, [2, 97, 0, 1, int(f"0x{raw_data_sampling_rate:02x}00", 16)])
    time.sleep(1)
    client.write_multiple_register(write_register, [1, 97, 0, 1])
    client.write_multiple_register(write_register, [1, 97, 0, 1])
    time.sleep(1)
    response = client.read_register(read_register, 5)
    raw_data_sampling_rate = (response[4] >> 8) & 0xFF
    print(f'Raw Data Sampling Rate is set to {raw_data_sampling_rate}')

    # Raw Data Memory Size

    
    client.write_multiple_register(write_register, [2, 98, 0, 2, int(f"{raw_data_memmory_size}")])
    client.write_multiple_register(write_register, [2, 98, 0, 2, int(f"{raw_data_memmory_size}")])
    
    time.sleep(1)
    client.write_multiple_register(write_register, [1, 98, 0, 2])
    client.write_multiple_register(write_register, [1, 98, 0, 2])
    time.sleep(1)
    response = client.read_register(read_register, 5)
    raw_data_memmory_size = response[4]
    print(f'Raw Data Memory Size is set to {raw_data_memmory_size}')

    # BLOB – Raw Data Recording Time

    client.write_multiple_register(write_register, [1, 99, 0, 4])
    client.write_multiple_register(write_register, [1, 99, 0, 4])
    time.sleep(1)
    response = client.read_register(read_register, 6)
    recording_time = (response[4] << 16) + response[5]
    print(f'BLOB Raw Data Recording Time is {recording_time:.2f} seconds or {recording_time / 60:.2f} minutes')

    # BLOB – Raw Data Transfer Time

    client.write_multiple_register(write_register, [1, 100, 0, 2])
    client.write_multiple_register(write_register, [1, 100, 0, 2])
    time.sleep(0.5)
    response = client.read_register(read_register, 5)
    print(f'BLOB Raw Data Transfer Time is {response[4]:.2f} seconds or {response[4]/60:.2f} minutes') 

if __name__ == "__main__":
    host = '192.168.137.21'  # Update with your Modbus TCP server IP
    port = 502  # Default Modbus TCP port
    unit_id = 1  # Update with your unit ID

    client = ModbusClientWrapper(host, port, unit_id)

    port_number = 4 #port on ICE2 which VIM is connected to
    ssc_trigger = 0
    # 0:Inactive
    # 1:SSC1.1 - vRMS
    # 2:SSC1.2 - vRMS
    # 3:SSC2.1 - aRMS
    # 4:SSC2.2 - aRMS
    # 5:SSC3.1 - aPeak
    # 6:SSC3.2 - aPeak
    # 7:SSC4.1 - Temperature
    # 8:SSC4.2 - Temperature
    # 9:SSC5.1 - Crest
    # 10:SSC5.2 - Crest
    # 11:SSC6.1 - Bearing Condition
    # 12:SSC6.2 - Bearing Condition
    raw_data_sampling_rate = 0
    # 0:64 kHz sampling rate, 12 kHz low-pass
    # 1:32 kHz sampling rate, 6 kHz low-pass
    # 2:16 kHz sampling rate, 3 kHz low-pass
    # 3:8 kHz sampling rate, 1.5 kHz low-pass
    # 4:4 kHz sampling rate, 0.75 kHz low-pass
    # 5:2 kHz sampling rate, 0.3 kHz low-pass
    raw_data_memmory_size = 5

    blob_config(port_number,ssc_trigger, raw_data_sampling_rate, raw_data_memmory_size) #configuring the blob parameters
    

    time.sleep(1)
    blob_id_response = read_blob_id_status(port_number)
    print(blob_id_response["blob_id"])
    time.sleep(1)
    trigger_Blob_capture(port_number)
    print("Waiting for blob capture to start")
    while Get_Blob_pdi_status(port_number) != 1:
        pass
        
        
    print("Raw data recording started")

    print("Waiting for blob capture to finish")
    while Get_Blob_pdi_status(port_number) != 2:
        pass
    
    print("Recording finished, wait for transfer")
    
    blob_id_response = read_blob_id_status(port_number)
    print(blob_id_response["blob_id"])

    Blob_data_capture(port_number)
    while Blob_data_capture(port_number) != 0:
        Blob_data_capture(port_number)
        #print(f'status is  {Get_Blob_pdi_status(4)}') 


    
    Blob_finish_capture(port_number)
    time.sleep(1)
    print(f'status is  {Get_Blob_pdi_status(port_number)}')
    blob_id_response = read_blob_id_status(port_number)
    print(blob_id_response["blob_id"])
    

