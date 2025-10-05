import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('dac_log_small.csv')

x_vals = df[['x_byte0', 'x_byte1', 'dac1']]
y_vals = df[['y_byte0', 'y_byte1', 'dac2']]

def decodeMSBY(byte1):
    ret = byte1 ^ 0x89
    ret = (ret & 0x80) | ((ret & 0x0F) << 3)
    return ret << 4

def decodeMSBX(byte1):
    ret = byte1 ^ 0x8E
    ret = (ret & 0x80) | ((ret & 0x0F) << 3)
    return ret << 4

def decodeLSBY(byte0):
    ret = (byte0 ^ 0xD8)
    ret = (ret & 0xFE) >> 1      
    return ret

def decodeLSBX(byte0):
    ret = (byte0 ^ 0xC8)
    ret = (ret & 0xFE) >> 1    
    return ret

# plt.plot(y_vals['y_byte1'].apply(decodeMSB), label='y_byte1_inverted')
plt.plot((y_vals['y_byte0'].apply(decodeLSBY))|y_vals['y_byte1'].apply(decodeMSBY), label='y')
# plt.plot((x_vals['x_byte0'].apply(decodeLSBX))|x_vals['x_byte1'].apply(decodeMSBX), label='x')
# plt.plot(y_vals['y_byte1'].apply(decodeMSB), label='aa')
# plt.plot(y_vals['y_byte1'].apply(lambda b: b&0x01), label='dac')
# plt.plot(y_vals['y_byte0'].apply(lambda b: (b&0x80)>>7), label='dac')
# plt.plot(y_vals['y_byte0'].apply(lambda b: (b&0x40)>>6), label='dac')
# plt.plot(y_vals['dac2'], label='dac')
plt.show()