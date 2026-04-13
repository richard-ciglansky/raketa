from kinematics import transform_to_kinematics, normalize_data, transform_to_si

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
        print(f"Time {res['time']}:")
        print(f"  Pos: ({res['positionX']:.2f}, {res['positionY']:.2f}, {res['positionZ']:.2f})")
        print(f"  Vel: ({res['velocityX']:.2f}, {res['velocityY']:.2f}, {res['velocityZ']:.2f})")

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
    # Test data with 20-bit signed integers
    # MAX_20BIT = 524287
    measurements = [
        (0, 524287, 0, -524288, 524287, 0, -524288)
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
    test_transform()
    test_normalize()
    test_transform_to_si()
