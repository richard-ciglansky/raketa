import csv
import matplotlib.pyplot as plt

from kinematics import transform_to_si, normalize_time, normalize_data, transform_to_kinematics, average_data


def read_measurements(filename, breaking_id, breaking_time):
    measure = []

    with open(filename, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
        next(spamreader) # Skip header
        for row in spamreader:
            measure.append([int(row[1]), int(row[2]), int(row[3]), int(row[4]), int(row[5]), int(row[6]), int(row[7])]);
    return measure


def data_pipeline(filename, breaking_id, breaking_time, accRange, rotRange):
    return transform_to_kinematics(
        transform_to_si(
            normalize_time(
                average_data(
                    normalize_data(
                    read_measurements(filename, breaking_id, breaking_time),
                    3000),
                10))
        ,
            accRange,
            rotRange))

measure1 = data_pipeline('motor_D9-7_start_1.csv', "190400", "0", 16,2000.0)
measure2 = data_pipeline('motor_D9-7_start_2.csv', "624995", "0", 16, 2000.0)
measure3 = data_pipeline('raketa3.csv', "624995", "0", 16, 2000.0)

# Rename raketa3.csv for consistency as per user request (implied by "raketa.csv" in description)
# The user asked for "raketa.csv" but only "raketa3.csv" exists.

datasets = [
    {"label": "motor_D9-7_start_1", "data": measure1},
    {"label": "motor_D9-7_start_2", "data": measure2},
    {"label": "raketa3", "data": measure3}
]

plt.figure(figsize=(20, 10))

# Subplot 1: Acceleration
plt.subplot(3, 1, 1)
for ds in datasets:
    times = [m[0] for m in ds["data"]]
    values = [m[3] for m in ds["data"]]
    plt.plot(times, values, label=ds["label"])
plt.ylabel('Acceleraration (m/s^2)')
plt.title('Rocket Data: Acceleration, Velocity and Height')
plt.grid(True)
plt.legend()

# Subplot 2: Velocity
plt.subplot(3, 1, 2)
for ds in datasets:
    times = [m[0] for m in ds["data"]]
    velocities = [m[9] for m in ds["data"]]
    plt.plot(times, velocities, label=ds["label"])
plt.ylabel('Velocity (m/s)')
plt.grid(True)
plt.legend()

# Subplot 3: Height
plt.subplot(3, 1, 3)
for ds in datasets:
    times = [m[0] for m in ds["data"]]
    heights = [m[12] for m in ds["data"]]
    plt.plot(times, heights, label=ds["label"])
plt.ylabel('Height (m)')
plt.xlabel('Time (ms)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()
