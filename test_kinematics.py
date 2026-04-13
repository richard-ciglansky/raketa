from kinematics import transform_to_kinematics, normalize_data, transform_to_si, average_data


def test_average_data():
    # Test data: (time_ms, ax, ay, az, rx, ry, rz)
    measurements = [
        (0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        (10, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0),
        (20, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0),
        (30, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0)
    ]
    
    # n = 2
    # Sample 0: avg(1.0) = 1.0
    # Sample 1: avg(1.0, 2.0) = 1.5
    # Sample 2: avg(2.0, 3.0) = 2.5
    # Sample 3: avg(3.0, 4.0) = 3.5
    
    averaged = average_data(measurements, 2)
    
    print("\n--- average_data (n=2) ---")
    for i, res in enumerate(averaged):
        print(f"Sample {i} (Time {res[0]}):")
        print(f"  Accel: ({res[1]:.2f}, {res[2]:.2f}, {res[3]:.2f})")
        
    expected_ax1 = 1.5
    if abs(averaged[1][1] - expected_ax1) > 0.001:
        print(f"FAILURE: Expected ax[1] {expected_ax1}, got {averaged[1][1]}")
    else:
        print("SUCCESS: average_data (n=2) test passed")

def test_transform():
    # Test data: (time_ms, accelX, accelY, accelZ, rotX, rotY, rotZ)
    measurements = [
        (0, 0, 0, 0, 0, 0, 0),
        (1000, 10, 0, 0, 0, 0, 0), # 1s later, ax = 10 m/s^2
        (2000, 0, 0, 0, 0, 0, 0)   # 1s later, ax = 0 m/s^2 (constant velocity)
    ]
    
    transformed = transform_to_kinematics(measurements)
    
    print("--- transform_to_kinematics ---")
    for i, res in enumerate(transformed):
        print(f"Time {res[0]}:")
        print(f"  Pos: ({res[7]:.2f}, {res[8]:.2f}, {res[9]:.2f})")
        print(f"  Vel: ({res[10]:.2f}, {res[11]:.2f}, {res[12]:.2f})")

def test_normalize():
    # Test data: (time_ms, accelX, accelY, accelZ, rotX, rotY, rotZ)
    measurements = [
        (0, 1.0, 2.0, 3.0, 0.1, 0.2, 0.3),
        (10, 1.2, 2.2, 3.2, 0.1, 0.2, 0.3),
        (20, 5.0, 6.0, 7.0, 1.0, 2.0, 3.0)
    ]
    
    # Normalize with SampleSize = 2
    # Averages of first 2:
    # ax: (1.0 + 1.2) / 2 = 1.1
    # ay: (2.0 + 2.2) / 2 = 2.1
    # az: (3.0 + 3.2) / 2 = 3.1
    # rx: (0.1 + 0.1) / 2 = 0.1
    # ry: (0.2 + 0.2) / 2 = 0.2
    # rz: (0.3 + 0.3) / 2 = 0.3
    
    normalized = normalize_data(measurements, 2)
    
    print("\n--- normalize_data (SampleSize=2) ---")
    for i, res in enumerate(normalized):
        print(f"Sample {i} (Time {res[0]}):")
        print(f"  Accel: ({res[1]:.2f}, {res[2]:.2f}, {res[3]:.2f})")
        print(f"  Rot: ({res[4]:.2f}, {res[5]:.2f}, {res[6]:.2f})")
        
    # Quick check for first sample
    expected_ax0 = 1.0 - 1.1 # -0.1
    if abs(normalized[0][1] - expected_ax0) > 0.001:
        print(f"FAILURE: Expected ax[0] {expected_ax0}, got {normalized[0][1]}")

def test_transform_to_si():
    # Test data with 16-bit signed integers
    # MAX_16BIT = 32767
    measurements = [
        (0, 32767, 0, -32768, 32767, 0, -32768)
    ]
    
    accel_range = 16 # 16G
    rot_range = 2000.0 # 2000 deg/s
    
    si_data = transform_to_si(measurements, accel_range, rot_range)
    
    print("\n--- transform_to_si ---")
    res = si_data[0]
    print(f"Time {res[0]}:")
    print(f"  Accel SI: ({res[1]:.2f}, {res[2]:.2f}, {res[3]:.2f})")
    print(f"  Rot SI: ({res[4]:.2f}, {res[5]:.2f}, {res[6]:.2f})")
    
    # Expected values
    expected_ax = 16 * 9.81 # 156.96
    expected_rx = 2000.0
    
    if abs(res[1] - expected_ax) > 0.01:
         print(f"FAILURE: Expected ax {expected_ax}, got {res[1]}")
    if abs(res[4] - expected_rx) > 0.01:
         print(f"FAILURE: Expected rx {expected_rx}, got {res[4]}")

if __name__ == "__main__":
    test_average_data()
    test_transform()
    test_normalize()
    test_transform_to_si()
