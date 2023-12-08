import numpy as np
import matplotlib.pyplot as plt


def generate_sine_table():
    table_size = 512 # 9-bit table
    
    sine_table_step_size = (2*np.pi) / table_size 
    
    lut_idx = np.arange(0, (2*np.pi), sine_table_step_size)
    sine_table_vals = np.sin(lut_idx) + 1
    
    max_val = np.amax(sine_table_vals)
    min_val = np.amin(sine_table_vals)
    interpolated_vals = np.interp(sine_table_vals, [min_val, max_val], [2, 510])
    
    sine_table_vals = np.round(interpolated_vals, 0).astype(int)
    
    print(list(sine_table_vals))
    print(sine_table_vals.size)
    
    plt.plot(np.arange(0, table_size, 1), interpolated_vals)
    plt.grid()
    plt.show()
    
    return sine_table_vals
    
def main():
    table = generate_sine_table()

if __name__ == "__main__":
    main()
