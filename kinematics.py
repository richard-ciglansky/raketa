def transform_to_kinematics(measurements):
    """
    Transforms a list of 7-number tuples (time, accelX, accelY, accelZ, rotX, rotY, rotZ)
    into a list of dictionaries with {time, posX, posY, posZ, velX, velY, velZ}.
    
    Assumptions:
    - time is in the same unit as the input (e.g., milliseconds or seconds).
    - If time is in milliseconds, delta_t should be converted to seconds if needed.
    - Here we'll treat time as seconds for simplicity or use the difference directly.
    - We'll use simple Euler integration: v = v + a*dt, p = p + v*dt.
    - For rotation, without a clear coordinate frame or conversion rule, 
      this implementation treats acceleration as already in the global frame.
    """
    result = []
    
    if not measurements:
        return result
    
    # Initialize state
    prev_time = measurements[0][0]
    velX, velY, velZ = 0.0, 0.0, 0.0
    posX, posY, posZ = 0.0, 0.0, 0.0
    
    # First point at t0
    time, ax, ay, az, rx, ry, rz = measurements[0]
    result.append([prev_time, ax, ay, az, rx, ry, rz, posX, posY, posZ, velX, velY, velZ])
    
    for i in range(1, len(measurements)):
        time, ax, ay, az, rx, ry, rz = measurements[i]
        # Calculate time difference (assuming time is in ms, convert to s)
        delta_t = (time - prev_time) / 1000.0
        
        # Simple Euler integration for velocity
        velX += ax * delta_t
        velY += ay * delta_t
        velZ += az * delta_t
        
        # Simple Euler integration for position
        posX += velX * delta_t
        posY += velY * delta_t
        posZ += velZ * delta_t
        
        result.append([time, ax, ay, az, rx, ry, rz, posX, posY, posZ, velX, velY, velZ])
        
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
    
    # Normalize all measurements
    normalized_result = []
    for time, ax, ay, az, rx, ry, rz in measurements:
        normalized_result.append((
            time,
            ax - avg_ax,
            ay - avg_ay,
            az - avg_az,
            rx - avg_rx,
            ry - avg_ry,
            rz - avg_rz
        ))
        
    return normalized_result


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

