import math

def transform_to_kinematics(measurements):
    """
    Transforms a list of 7-number tuples (time, accelX, accelY, accelZ, rotX, rotY, rotZ)
    into a list of lists with {time, ax, ay, az, rx, ry, rz, posX, posY, posZ, velX, velY, velZ, roll, pitch, yaw}.
    
    Assumptions:
    - time is in milliseconds and is converted to seconds for integration.
    - rx, ry, rz are in degrees/second.
    - We'll use simple Euler integration for both orientation and position/velocity.
    - Rotation order: Z then Y then X (Yaw, Pitch, Roll).
    - Transformation from body to global frame is applied to acceleration.
    """
    result = []
    
    if not measurements:
        return result
    
    # Initialize state
    prev_time = measurements[0][0]
    velX, velY, velZ = 0.0, 0.0, 0.0
    posX, posY, posZ = 0.0, 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0 # Orientation in radians
    
    # First point at t0
    time, ax, ay, az, rx, ry, rz = measurements[0]
    # At t0, we assume global frame aligns with body frame, or we start from initial orientation 0.
    # Initial record: include orientation
    result.append([time, ax, ay, az, rx, ry, rz, posX, posY, posZ, velX, velY, velZ, 
                   math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
    
    for i in range(1, len(measurements)):
        time, ax, ay, az, rx, ry, rz = measurements[i]
        delta_t = (time - prev_time) / 1000.0
        
        # 1. Update orientation (integrate angular velocity)
        # Convert deg/s to rad/s
        roll += math.radians(rx) * delta_t
        pitch += math.radians(ry) * delta_t
        yaw += math.radians(rz) * delta_t
        
        # 2. Transform body acceleration (ax, ay, az) to global acceleration (gax, gay, gaz)
        # Using ZYX (Yaw-Pitch-Roll) intrinsic rotation or XYZ extrinsic.
        # R = Rz(yaw) * Ry(pitch) * Rx(roll)
        
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        
        # Rotation matrix R:
        # R = [ cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr ]
        #     [ sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr ]
        #     [ -sp,   cp*sr,            cp*cr           ]
        
        gax = ax * (cy * cp) + ay * (cy * sp * sr - sy * cr) + az * (cy * sp * cr + sy * sr)
        gay = ax * (sy * cp) + ay * (sy * sp * sr + cy * cr) + az * (sy * sp * cr - cy * sr)
        gaz = ax * (-sp)     + ay * (cp * sr)                + az * (cp * cr)
        
        # 3. Simple Euler integration for velocity
        velX += gax * delta_t
        velY += gay * delta_t
        velZ += gaz * delta_t
        
        # 4. Simple Euler integration for position
        posX += velX * delta_t
        posY += velY * delta_t
        posZ += velZ * delta_t
        
        result.append([time, ax, ay, az, rx, ry, rz, posX, posY, posZ, velX, velY, velZ, 
                       math.degrees(roll), math.degrees(pitch), math.degrees(yaw)])
        
        prev_time = time
        
    return result

def normalize_data(measurements, SampleSize):
    """
    Normalizes a list of 7-number tuples (time, accelX, accelY, accelZ, rotX, rotY, rotZ)
    by calculating the average of the first SampleSize values for each acceleration 
    and rotation component and then subtracting this average from all samples.
    """
    if not measurements or SampleSize <= 0:
        return measurements
    
    # Ensure SampleSize doesn't exceed the number of available measurements
    actual_sample_size = min(SampleSize, len(measurements))
    
    # Initialize sums for averaging
    sum_ax = sum_ay = sum_az = 0.0
    sum_rx = sum_ry = sum_rz = 0.0
    
    # Calculate sums for the first actual_sample_size elements
    for i in range(actual_sample_size):
        _, ax, ay, az, rx, ry, rz = measurements[i]
        sum_ax += ax
        sum_ay += ay
        sum_az += az
        sum_rx += rx
        sum_ry += ry
        sum_rz += rz
        
    # Calculate averages
    avg_ax = sum_ax / actual_sample_size
    avg_ay = sum_ay / actual_sample_size
    avg_az = sum_az / actual_sample_size
    avg_rx = sum_rx / actual_sample_size
    avg_ry = sum_ry / actual_sample_size
    avg_rz = sum_rz / actual_sample_size

    # -2048 is the offset for Z axis of the accelerometer (1G range)
    return [avg_ax, avg_ay, avg_az - 2048, avg_rx, avg_ry, avg_rz]


def transform_to_si(measurements, AccelerationRange, RotationRange):
    """
    Transforms a list of 7-number tuples (time, accelX, accelY, accelZ, rotX, rotY, rotZ)
    where acceleration and rotation are 16-bit signed integers, into SI units.
    - Acceleration: m/s^2 (converted from G-range)
    - Rotation: degrees/s

    Parameters:
    - measurements: List of (time, ax, ay, az, rx, ry, rz)
    - AccelerationRange: Integer (max acceleration in G)
    - RotationRange: Double (max rotation in degrees/s)

    16-bit signed integer range: -32768 to 32767 (2^15)
    """
    if not measurements:
        return []

    # Max value for 16-bit signed integer is 2^16 - 1 = 32767
    MAX_16BIT = 32767.0

    # Conversion factors
    accel_factor = (AccelerationRange * 9.81) / MAX_16BIT
    rot_factor = RotationRange / MAX_16BIT

    transformed_result = []
    for time, ax, ay, az, rx, ry, rz in measurements:
        transformed_result.append((
            time,
            ax * accel_factor,
            ay * accel_factor,
            az * accel_factor,
            rx * rot_factor,
            ry * rot_factor,
            rz * rot_factor
        ))

    return transformed_result


def average_data(measurements, n):
    """
    Transforms input measurements into a sliding average of respective values
    except for time, for a specified number of successive samples (n).
    
    If n is less than 1, returns the original measurements.
    """
    if not measurements or n <= 1:
        return measurements

    result = []
    
    # We use a sliding window of size n ending at the current sample.
    for i in range(len(measurements)):
        start_idx = max(0, i - n + 1)
        window = measurements[start_idx : i + 1]
        window_size = len(window)
        
        sum_ax = sum(m[1] for m in window)
        sum_ay = sum(m[2] for m in window)
        sum_az = sum(m[3] for m in window)
        sum_rx = sum(m[4] for m in window)
        sum_ry = sum(m[5] for m in window)
        sum_rz = sum(m[6] for m in window)
        
        result.append((
            measurements[i][0], # Keep current time
            sum_ax / window_size,
            sum_ay / window_size,
            sum_az / window_size,
            sum_rx / window_size,
            sum_ry / window_size,
            sum_rz / window_size
        ))
        
    return result


def normalize_time(measurements):
    if not measurements:
        return []

    transformed_result = []
    startTime = measurements[0][0]
    for time, ax, ay, az, rx, ry, rz in measurements:
        transformed_result.append((
            time-startTime,
            ax,
            ay,
            az,
            rx,
            ry,
            rz
        ))

    return transformed_result

