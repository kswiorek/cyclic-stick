import pandas as pd
from itertools import combinations

# Load the processed DataFrame with bit columns
df = pd.read_csv('dac_log.csv')

# Add bit columns if not already present
for axis, prefix in zip([['x_byte0', 'x_byte1'], ['y_byte0', 'y_byte1']], ['x', 'y']):
    for i in range(2):
        byte_col = axis[i]
        for bit in range(8):
            col_name = f'{prefix}_byte{i}_bit{bit}'
            if col_name not in df.columns:
                df[col_name] = (df[byte_col].to_numpy() >> bit) & 1

# --- User parameters ---
ranges = [
    (60, 75),
    (98, 115),
    # Add more (start, end) tuples as needed
]
# -----------------------

bit_cols = [col for col in df.columns if '_bit' in col]

for idx, (start, end) in enumerate(ranges):
    print(f"\nRange {idx+1}: rows {start} to {end}")
    range_df = df.iloc[start:end]
    for bit in bit_cols:
        zeros = (range_df[bit] == 0).sum()
        ones = (range_df[bit] == 1).sum()
        print(f"{bit}: zeros = {zeros}, ones = {ones}")